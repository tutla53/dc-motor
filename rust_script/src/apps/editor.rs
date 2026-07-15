use super::*;

#[derive(Helper)]
pub struct CommandCompleter {
    pub commands: Vec<String>,
    pub routines: Vec<String>,
}

impl CommandCompleter {
    fn parse_line_context<'a>(
        &'a self,
        line: &str,
    ) -> Option<(&'static str, usize, &'a Vec<String>)> {
        if line.starts_with("dev ") {
            Some(("dev ", 4, &self.commands))
        } else if line.starts_with("run ") {
            Some(("run ", 4, &self.routines))
        } else {
            None
        }
    }
}

impl Completer for CommandCompleter {
    type Candidate = Pair;

    fn complete(
        &self,
        line: &str,
        pos: usize,
        _ctx: &Context<'_>,
    ) -> rustyline::Result<(usize, Vec<Pair>)> {
        if let Some((_prefix, offset, target_list)) = self.parse_line_context(line)
            && pos >= offset
        {
            let sub_str = &line[offset..pos];
            let mut candidates = Vec::new();

            for item in target_list {
                if item.starts_with(sub_str) {
                    candidates.push(Pair {
                        display: item.to_string(),
                        replacement: item.to_string(),
                    });
                }
            }
            return Ok((offset, candidates));
        }

        Ok((pos, Vec::new()))
    }
}

impl Hinter for CommandCompleter {
    type Hint = String;

    fn hint(&self, line: &str, pos: usize, _ctx: &Context<'_>) -> Option<Self::Hint> {
        let (_prefix, offset, target_list) = self.parse_line_context(line)?;

        if pos < offset {
            return None;
        }

        let sub_str = &line[offset..pos];
        if sub_str.is_empty() {
            return None;
        }

        let matches: Vec<&String> = target_list
            .iter()
            .filter(|item| item.starts_with(sub_str))
            .collect();

        if matches.len() == 1 {
            let full_match = matches[0];
            return Some(full_match[sub_str.len()..].to_string());
        }

        if matches.len() > 1 {
            return Some(format!("   ({} possibilities)", matches.len()));
        }

        None
    }
}

impl Highlighter for CommandCompleter {}

impl Validator for CommandCompleter {
    fn validate(
        &self,
        _ctx: &mut rustyline::validate::ValidationContext<'_>,
    ) -> rustyline::Result<rustyline::validate::ValidationResult> {
        Ok(rustyline::validate::ValidationResult::Valid(None))
    }
}

pub fn initialized_editor(
    available_commands: &[String],
    available_routines: &[String],
) -> Result<Editor<CommandCompleter, DefaultHistory>, Box<dyn std::error::Error>> {
    let config = Config::builder().auto_add_history(false).build();
    let mut editor = Editor::with_config(config)?;

    editor.set_helper(Some(CommandCompleter {
        commands: available_commands.to_owned(),
        routines: available_routines.to_owned(),
    }));

    Ok(editor)
}
