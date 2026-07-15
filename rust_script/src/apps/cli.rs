use super::*;

#[derive(Parser, Debug)]
#[command(no_binary_name = true)] 
pub struct ReplCli {
    #[command(subcommand)]
    pub command: ReplCommands,
}

#[derive(Subcommand, Debug)]
pub enum ReplCommands {
    /// Execute raw API commands. Use -a or --all to list available APIs.
    Dev {
        /// List all available dynamic TOML commands
        #[arg(short, long)]
        all: bool,

        /// The name of the API command to run
        #[arg(required_unless_present = "all")]
        command_name: Option<String>,

        /// Arguments for the target API command
        #[arg(trailing_var_arg = true, allow_hyphen_values = true, num_args = 0..)]
        args: Vec<String>,
    },

    /// Run the Custom Program on the program/script.rs
    Run {
        /// List all available dynamic TOML commands
        #[arg(short, long)]
        all: bool,

        /// Name of Program
        #[arg(required_unless_present = "all")]
        routine_name: Option<String>,

        /// Arguments for the target API command
        #[arg(trailing_var_arg = true, allow_hyphen_values = true, num_args = 0..)]
        args: Vec<String>,
    },

    /// Exit the Program
    #[command(alias = "quit", alias = "q")]
    Exit,
}
