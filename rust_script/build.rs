use std::env;
use std::fs;
use std::path::Path;
use serde::Deserialize;
use indexmap::IndexMap;

const CONFIG_FILE: &str = "DeviceOpFuncs/DCMotor.toml";

#[derive(Deserialize, Debug)]
struct CommandDef {
    command: String,
    op: u8,
    #[serde(default)]
    args: IndexMap<String, String>,
    #[serde(default)]
    ret: IndexMap<String, String>,
}

#[derive(Deserialize, Debug)]
struct Config {
    commands: Option<Vec<CommandDef>>,
}

fn map_type(t: &str) -> (&'static str, &'static str, &'static str) {
    match t {
        "u8"  => ("u8",  "write_u8({arg})", "read_u8()"),
        "u16" => ("u16", "write_u16::<LittleEndian>({arg})", "read_u16::<LittleEndian>()"),
        "u32" => ("u32", "write_u32::<LittleEndian>({arg})", "read_u32::<LittleEndian>()"),
        "i32" => ("i32", "write_i32::<LittleEndian>({arg})", "read_i32::<LittleEndian>()"),
        "f32" => ("f32", "write_f32::<LittleEndian>({arg})", "read_f32::<LittleEndian>()"),
        "u64" => ("u64", "write_u64::<LittleEndian>({arg})", "read_u64::<LittleEndian>()"),
        _ => panic!("Unsupported type in TOML: {}", t),
    }
}

fn main() {    
    println!("cargo:rerun-if-changed={}", CONFIG_FILE);
    println!("cargo:rustc-env=CONFIG_FILE={}", CONFIG_FILE);
    let toml_content = fs::read_to_string(CONFIG_FILE)
        .unwrap_or_else(|_| panic!("Failed to read {}", CONFIG_FILE));
    let config: Config = toml::from_str(&toml_content).expect("Failed to parse TOML");

    let mut generated_code = String::new();
    generated_code.push_str("use byteorder::{LittleEndian, ReadBytesExt, WriteBytesExt};\n\n");
    generated_code.push_str("#[allow(unused)] \n");
    generated_code.push_str("impl Pico {\n");

    if let Some(commands) = config.commands {
        for cmd in commands {
            // 1. Build Function Signature
            let mut arg_list = vec!["&mut self".to_string()];
            for (arg_name, arg_type) in &cmd.args {
                arg_list.push(format!("{}: {}", arg_name, map_type(arg_type).0));
            }
            
            let ret_types: Vec<_> = cmd.ret.values().map(|t| map_type(t).0).collect();
            let ret_signature = match ret_types.len() {
                0 => "()".to_string(),
                1 => ret_types[0].to_string(),
                _ => format!("({})", ret_types.join(", ")), 
            };

            generated_code.push_str(&format!(
                "    pub fn {}({}) -> Result<{}, String> {{\n",
                cmd.command, arg_list.join(", "), ret_signature
            ));

            if cmd.args.is_empty() {
                generated_code.push_str("        let payload = Vec::new();\n");
            } else {
                generated_code.push_str("        let mut payload = Vec::new();\n");
            }

            // 2. Build Argument Packing (Write to bytes)
            for (arg_name, arg_type) in &cmd.args {
                let write_macro = map_type(arg_type).1.replace("{arg}", arg_name);
                generated_code.push_str(&format!(
                    "        payload.{}.map_err(|e| e.to_string())?;\n", write_macro
                ));
            }

            // 3. Execute Command
            let has_ret = !cmd.ret.is_empty();
            if has_ret {
                generated_code.push_str(&format!(
                    "        let response_bytes = self.execute_command({}, &payload, true)?.ok_or(\"No response\")?;\n", cmd.op
                ));
                generated_code.push_str("        let mut cursor = std::io::Cursor::new(response_bytes);\n");

                // 4. Build Return Unpacking (Read from bytes)
                let mut ret_vars = Vec::new();
                for (i, (_ret_name, ret_type)) in cmd.ret.iter().enumerate() {
                    let var_name = format!("ret_{}", i);
                    let read_macro = map_type(ret_type).2;
                    generated_code.push_str(&format!(
                        "        let {} = cursor.{}.map_err(|e| e.to_string())?;\n", var_name, read_macro
                    ));
                    ret_vars.push(var_name);
                }
                
                if ret_vars.len() == 1 {
                    generated_code.push_str(&format!("        Ok({})\n", ret_vars[0]));
                } else {
                    generated_code.push_str(&format!("        Ok(({}))\n", ret_vars.join(", ")));
                }
            } else {
                generated_code.push_str(&format!(
                    "        self.execute_command({}, &payload, false)?;\n        Ok(())\n", cmd.op
                ));
            }
            generated_code.push_str("    }\n\n");
        }
    }
    generated_code.push_str("}\n");

    let out_dir = env::var_os("OUT_DIR").unwrap();
    let dest_path = Path::new(&out_dir).join("generated_commands.rs");
    fs::write(&dest_path, generated_code).unwrap();
}