/*
* USB Resources
    Input Command Pattern
        [HEADER] [OP_CODE] [PARAMETERS]
    Output Pattern
        [HEADER] [ERROR_CODE] [OP_CODE] [DATA]
    Event Pattern
        [HEADER] [EVENT_CODE] [ID]
*/

/* --------------------------- HEADER -------------------------- */
#[derive(PartialEq)]
#[repr(u8)]
pub enum HEADER {
    COMMAND = 0xFF,
    EVENT = 0xFE,
    LOGGER = 0xFD,
}

/* --------------------------- OP_CODE -------------------------- */
#[derive(PartialEq, Clone)]
#[repr(u8)]
pub enum OpCode {
    None = 0,
    StartLogger = 1,
    StopLogger = 2,
    MoveMotorSpeed = 3,
    MoveMotorAbsPos = 4,
    StopMotor = 5,
    SetMotorPosPidParam = 6,
    GetMotorPosPidParam = 7,
    SetMotorSpeedPidParam = 8,
    GetMotorSpeedPidParam = 9,
    MoveMotorAbsPosTrapezoid = 10,
    GetMotorPos = 11,
    GetMotorSpeed = 12,
    MoveMotorOpenLoop = 13,
    SaveConfiguration = 14,
    SetToDefaultConfig = 15,
}

// DON'T FORGET TO EDIT HERE TOO
impl TryFrom<u8> for OpCode {
    type Error = ();

    fn try_from(v: u8) -> Result<Self, Self::Error> {
        match v {
            0 => Ok(OpCode::None),
            1 => Ok(OpCode::StartLogger),
            2 => Ok(OpCode::StopLogger),
            3 => Ok(OpCode::MoveMotorSpeed),
            4 => Ok(OpCode::MoveMotorAbsPos),
            5 => Ok(OpCode::StopMotor),
            6 => Ok(OpCode::SetMotorPosPidParam),
            7 => Ok(OpCode::GetMotorPosPidParam),
            8 => Ok(OpCode::SetMotorSpeedPidParam),
            9 => Ok(OpCode::GetMotorSpeedPidParam),
            10 => Ok(OpCode::MoveMotorAbsPosTrapezoid),
            11 => Ok(OpCode::GetMotorPos),
            12 => Ok(OpCode::GetMotorSpeed),
            13 => Ok(OpCode::MoveMotorOpenLoop),
            14 => Ok(OpCode::SaveConfiguration),
            15 => Ok(OpCode::SetToDefaultConfig),
            _ => Err(()),
        }
    }
}

/* --------------------------- Error Code -------------------------- */
#[repr(u8)]
pub enum ErrorCode {
    NoError = 0,
    OpCodeNotFound = 1,
    NonFiniteFloat = 2,
    ReadByteError = 3,
    InvalidMotorId = 4,
    InvalidHeaderCode = 5,
    InvalidTimeSampling = 6,
}

/* --------------------------- Event List -------------------------- */
#[derive(Clone, Copy)]
pub enum EventList {
    MotorMoveDone(u8),
}