/*
* USB Resources
* Command:  [0xFF, opcode, led_id: i32 little-endian]
* Response: [0xFF, error, opcode (when available), optional status: u8]
*/
use usb_comm::CommandError;
use usb_comm::CommandOpcode;

#[derive(PartialEq)]
#[repr(u8)]
pub enum UsbHeader {
    Command = 0xff,
}

usb_comm::create_opcode_enum! {
    #[derive(PartialEq, Clone, Copy, Debug)]
    #[repr(u8)]
    pub enum OpCode {
        None = 0,
        TurnOnLed = 1,
        TurnOffLed = 2,
        GetLedStatus = 3,
    }
}

#[derive(Clone, Copy)]
#[repr(u8)]
pub enum ErrorCode {
    NoError = 0,
    OpCodeNotFound = 1,
    ReadByteError = 2,
    InvalidHeaderCode = 3,
    InvalidLedId = 4,
}

/* --------------------------- Generic Parser Integration -------------------------- */
impl CommandOpcode for OpCode {
    fn decode(value: u8) -> Option<Self> {
        let opcode = Self::try_from(value).ok()?;
        if opcode == Self::None {
            return Option::None;
        }
        Some(opcode)
    }
}

impl From<CommandError> for ErrorCode {
    fn from(error: CommandError) -> Self {
        match error {
            CommandError::InvalidHeader => Self::InvalidHeaderCode,
            CommandError::UnknownOpcode(_) => Self::OpCodeNotFound,
            CommandError::Read(_) | CommandError::UnexpectedTrailingBytes => Self::ReadByteError,
        }
    }
}
