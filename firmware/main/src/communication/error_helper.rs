use super::*;

/* --------------------------- Decode OpCode -------------------------- */
impl CommandOpcode for OpCode {
    fn decode(value: u8) -> Option<Self> {
        let opcode = Self::try_from(value).ok()?;

        match opcode {
            Self::None => Option::None,
            _ => Some(opcode),
        }
    }
}

/* --------------------------- Generic Parser Integration -------------------------- */
impl From<PacketError> for ErrorCode {
    fn from(error: PacketError) -> Self {
        match error {
            PacketError::BufferFull => Self::PacketBufferFull,
        }
    }
}

impl From<CommandError> for ErrorCode {
    fn from(error: CommandError) -> Self {
        match error {
            CommandError::InvalidHeader => Self::InvalidHeaderCode,

            CommandError::UnknownOpcode(_) => Self::OpCodeNotFound,

            CommandError::Read(ReadError::NonFiniteFloat) => Self::NonFiniteFloat,

            CommandError::Read(ReadError::UnexpectedEnd)
            | CommandError::Read(ReadError::InvalidValue)
            | CommandError::UnexpectedTrailingBytes => Self::ReadByteError,
        }
    }
}
