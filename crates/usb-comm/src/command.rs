/*
* Generic Command Reader
*/
use crate::reader::ByteReader;
use crate::reader::FromLeBytes;
use crate::reader::ReadError;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CommandError {
    Read(ReadError),
    InvalidHeader,
    UnknownOpcode(u8),
    UnexpectedTrailingBytes,
}

impl From<ReadError> for CommandError {
    fn from(error: ReadError) -> Self {
        Self::Read(error)
    }
}

pub trait CommandOpcode: Copy {
    /// Return None for unknown or reserved opcodes.
    fn decode(value: u8) -> Option<Self>;
}

pub struct CommandReader<'a, O: CommandOpcode> {
    opcode: O,
    reader: ByteReader<'a>,
}

impl<'a, O: CommandOpcode> CommandReader<'a, O> {
    pub fn new(data: &'a [u8], expected_header: u8) -> Result<Self, CommandError> {
        let mut reader = ByteReader::new(data);
        if reader.read::<u8>()? != expected_header {
            return Err(CommandError::InvalidHeader);
        }
        let raw_opcode = reader.read::<u8>()?;
        let opcode = O::decode(raw_opcode).ok_or(CommandError::UnknownOpcode(raw_opcode))?;
        Ok(Self { opcode, reader })
    }

    pub fn opcode(&self) -> O {
        self.opcode
    }
    pub fn read<T: FromLeBytes>(&mut self) -> Result<T, CommandError> {
        Ok(self.reader.read::<T>()?)
    }
    pub fn read_f32(&mut self) -> Result<f32, CommandError> {
        Ok(self.reader.read_f32()?)
    }
    pub fn finish(self) -> Result<(), CommandError> {
        if !self.reader.is_finished() {
            return Err(CommandError::UnexpectedTrailingBytes);
        }
        Ok(())
    }
}
