use indexmap::IndexMap;
use serde::Deserialize;
use std::env;
use std::fs;
use std::path::Path;

const CONFIG_FILE: &str = "DeviceOpFuncs/DCMotor.toml";
const PROGRAM_FILE: &str = "src/program/script.rs";

#[derive(Deserialize, Debug)]
struct TelemetryFlag {
    name: String,
    label: String,
    scale: String,
    offset: String,
}

#[derive(Deserialize, Debug)]
struct TelemetryConfig {
    flags: Vec<TelemetryFlag>,
}

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
    telemetry: Option<TelemetryConfig>,
    commands: Option<Vec<CommandDef>>,
}

fn map_type(t: &str) -> (&'static str, &'static str, &'static str) {
    match t {
        "u8" => ("u8", "write_u8({arg})", "read_u8()"),
        "u16" => (
            "u16",
            "write_u16::<LittleEndian>({arg})",
            "read_u16::<LittleEndian>()",
        ),
        "u32" => (
            "u32",
            "write_u32::<LittleEndian>({arg})",
            "read_u32::<LittleEndian>()",
        ),
        "i32" => (
            "i32",
            "write_i32::<LittleEndian>({arg})",
            "read_i32::<LittleEndian>()",
        ),
        "f32" => (
            "f32",
            "write_f32::<LittleEndian>({arg})",
            "read_f32::<LittleEndian>()",
        ),
        "f64" => (
            "f64",
            "write_f64::<LittleEndian>({arg})",
            "read_f64::<LittleEndian>()",
        ),
        "u64" => (
            "u64",
            "write_u64::<LittleEndian>({arg})",
            "read_u64::<LittleEndian>()",
        ),
        _ => panic!("Unsupported type in TOML: {}", t),
    }
}

fn main() {
    println!("cargo:rerun-if-changed={}", CONFIG_FILE);
    println!("cargo:rerun-if-changed={}", PROGRAM_FILE);
    println!("cargo:rustc-env=CONFIG_FILE={}", CONFIG_FILE);

    let mut generated_code = String::new();

    let toml_content = fs::read_to_string(CONFIG_FILE)
        .unwrap_or_else(|_| panic!("Failed to read {}", CONFIG_FILE));
    let config: Config = toml::from_str(&toml_content).expect("Failed to parse TOML");

    // PART 0: Generate Telemetry Bitflags, Name Lookups, and scale/offset maps
    if let Some(telemetry_cfg) = config.telemetry {
        generated_code.push_str("use bitflags::bitflags;\n\n");
        generated_code.push_str("bitflags! {\n");
        generated_code.push_str("    #[derive(Debug, Clone, Copy, PartialEq, Eq)]\n");
        generated_code.push_str("    pub struct LogMask: u32 {\n");
        generated_code.push_str("        const NoLog = 0;\n");

        for (i, flag) in telemetry_cfg.flags.iter().enumerate() {
            generated_code.push_str(&format!("        const {} = 1 << {};\n", flag.name, i));
        }
        generated_code.push_str("    }\n}\n\n");

        // Generate ALL_FLAGS
        generated_code.push_str("pub const ALL_FLAGS: &[(LogMask, &str)] = &[\n");
        for flag in &telemetry_cfg.flags {
            generated_code.push_str(&format!(
                "    (LogMask::{}, \"{}\"),\n",
                flag.name, flag.label
            ));
        }
        generated_code.push_str("];\n\n");

        // Generate LOG_CONFIG_MAP utilizing the raw formula strings directly
        generated_code.push_str("pub const LOG_CONFIG_MAP: &[(LogMask, f64, f64)] = &[\n");
        for flag in &telemetry_cfg.flags {
            generated_code.push_str(&format!(
                "    (LogMask::{}, {}, {}),\n",
                flag.name, flag.scale, flag.offset
            ));
        }
        generated_code.push_str("];\n\n");

        generated_code.push_str(
            r#"impl LogMask {
            pub fn get_active_names(&self) -> Vec<&'static str> {
                let mut names = vec!["Timestamp(ms)"];
                for &(flag, name) in ALL_FLAGS {
                    if self.contains(flag) {
                        names.push(name);
                    }
                }
                names
            }

            pub fn get_scale_offset(&self) -> (f64, f64) {
                LOG_CONFIG_MAP
                    .iter()
                    .find(|&&(flag, _, _)| flag == *self)
                    .map(|&(_, scale, offset)| (scale, offset))
                    .unwrap_or((1.0, 0.0))
            }
        }

        "#,
        );
    }

    // PART 1: Generate Pico Command Implementations from TOML
    generated_code.push_str("use byteorder::{LittleEndian, ReadBytesExt, WriteBytesExt};\n\n");
    generated_code.push_str("#[allow(unused)] \n");
    generated_code.push_str("impl Pico {\n");

    if let Some(commands) = config.commands {
        for cmd in commands {
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
                cmd.command,
                arg_list.join(", "),
                ret_signature
            ));

            if cmd.args.is_empty() {
                generated_code.push_str("        let payload = Vec::new();\n");
            } else {
                generated_code.push_str("        let mut payload = Vec::new();\n");
            }

            for (arg_name, arg_type) in &cmd.args {
                let write_macro = map_type(arg_type).1.replace("{arg}", arg_name);
                generated_code.push_str(&format!(
                    "        payload.{}.map_err(|e| e.to_string())?;\n",
                    write_macro
                ));
            }

            let has_ret = !cmd.ret.is_empty();
            if has_ret {
                generated_code.push_str(&format!(
                    "        let response_bytes = self.execute_command({}, &payload)?.ok_or(\"No response\")?;\n", cmd.op
                ));
                generated_code
                    .push_str("        let mut cursor = std::io::Cursor::new(response_bytes);\n");

                let mut ret_vars = Vec::new();
                for (i, (_ret_name, ret_type)) in cmd.ret.iter().enumerate() {
                    let var_name = format!("ret_{}", i);
                    let read_macro = map_type(ret_type).2;
                    generated_code.push_str(&format!(
                        "        let {} = cursor.{}.map_err(|e| e.to_string())?;\n",
                        var_name, read_macro
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
                    "        self.execute_command({}, &payload)?;\n        Ok(())\n",
                    cmd.op
                ));
            }
            generated_code.push_str("    }\n\n");
        }
    }
    generated_code.push_str("}\n\n");

    // PART 2: Generate CLI Menu Routines from script.rs (Handles Multi-line Signatures)
    let program_content = fs::read_to_string(PROGRAM_FILE).unwrap_or_else(|_| "".to_string());
    let mut functions: Vec<(String, Vec<String>, bool, bool)> = Vec::new();

    for block in program_content.split("pub fn ") {
        let block = block.trim();
        if block.is_empty() {
            continue;
        }

        if let (Some(start_idx), Some(end_idx)) = (block.find('('), block.find(')')) {
            let name = block[..start_idx].trim().to_string();
            let args_blob = &block[start_idx + 1..end_idx].replace("\n", " ");

            let post_args = &block[end_idx + 1..].split('{').next().unwrap_or("");
            let returns_result = post_args.contains("Result") || post_args.contains("->");

            let needs_motor = args_blob.contains("motor") || args_blob.contains("Motor");

            let mut param_types = Vec::new();
            if !args_blob.trim().is_empty() {
                for arg in args_blob.split(',') {
                    let parts: Vec<&str> = arg.split(':').collect();
                    if parts.len() == 2 {
                        let ty = parts[1].trim().to_string();
                        if !ty.contains("Motor") && !ty.contains("motor") {
                            param_types.push(ty);
                        }
                    }
                }
            }

            functions.push((name, param_types, needs_motor, returns_result));
        }
    }

    generated_code.push_str("pub const PROGRAM_FUNCTIONS: &[&str] = &[\n");
    for (name, _, _, _) in &functions {
        generated_code.push_str(&format!("    \"{}\",\n", name));
    }
    generated_code.push_str("];\n\n");

    generated_code.push_str("#[allow(unused_variables)]\n");
    generated_code.push_str("#[allow(clippy::useless_format)]\n");
    generated_code.push_str("#[allow(clippy::get_first)]\n");
    generated_code.push_str(
        "pub fn execute_program_routine(name: &str, args: &[&str]) -> Result<(), String> {\n",
    );
    generated_code.push_str("    match name {\n");

    for (name, param_types, needs_motor, returns_result) in &functions {
        generated_code.push_str(&format!("        \"{}\" => {{\n", name));

        if *needs_motor {
            generated_code.push_str("            let resources = crate::SHARED.get().ok_or(\"Global shared resources not initialized\")?;\n");
            generated_code.push_str("            let mut motor_lock = resources.m0.lock().map_err(|e| e.to_string())?;\n");
        }

        // 1. Strict parsing sequence block
        for (i, ty) in param_types.iter().enumerate() {
            generated_code.push_str(&format!(
                "            let param_{}: {} = args.get({})\n",
                i, ty, i
            ));
            generated_code.push_str(&format!(
                "                .ok_or_else(|| format!(\"Missing argument at index {}\"))?\n",
                i
            ));
            generated_code.push_str(&format!("                .parse::<{}>()\n", ty));
            generated_code.push_str(&format!(
                "                .map_err(|_| format!(\"Invalid type for argument {}: expected {}\"))?;\n", i, ty
            ));
        }

        // 2. Exact function dispatch execution call based on return layout
        if *returns_result {
            generated_code.push_str(&format!("            crate::program::script::{}(\n", name));
            if *needs_motor {
                generated_code.push_str("                &mut *motor_lock,\n");
            }
            for (i, _) in param_types.iter().enumerate() {
                generated_code.push_str(&format!("                param_{},\n", i));
            }
            generated_code.push_str("            ).map_err(|e| e.to_string())?;\n");
        } else {
            generated_code.push_str(&format!("            crate::program::script::{}(\n", name));
            if *needs_motor {
                generated_code.push_str("                &mut *motor_lock,\n");
            }
            for (i, _) in param_types.iter().enumerate() {
                generated_code.push_str(&format!("                param_{},\n", i));
            }
            generated_code.push_str("            );\n");
        }

        generated_code.push_str("            Ok(())\n");
        generated_code.push_str("        }\n");
    }

    generated_code
        .push_str("        _ => Err(format!(\"Routine '{}' not found in script.rs\", name)),\n");
    generated_code.push_str("    }\n");
    generated_code.push_str("}\n");

    generated_code.push_str("\npub fn get_routine_arg_count(name: &str) -> Option<usize> {\n");
    generated_code.push_str("    match name {\n");
    for (name, param_types, _, _) in &functions {
        generated_code.push_str(&format!(
            "        \"{}\" => Some({}),\n",
            name,
            param_types.len()
        ));
    }
    generated_code.push_str("        _ => None,\n");
    generated_code.push_str("    }\n");
    generated_code.push_str("}\n");

    let out_dir = env::var_os("OUT_DIR").unwrap();
    let dest_path = Path::new(&out_dir).join("generated_commands.rs");
    fs::write(&dest_path, generated_code).unwrap();
}
