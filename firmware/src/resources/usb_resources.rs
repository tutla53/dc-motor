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