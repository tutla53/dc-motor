/*
* USB Handler
*/

use super::*;

/* --------------------------- Command Handler-------------------------- */
pub struct CommandHandler<'a> {
    pub data: &'a [u8],
    pub cursor: usize,
    header: u8,
}

impl<'a> CommandHandler<'a> {
    pub fn new(data: &'a [u8]) -> Self {
        Self {
            data,
            cursor: 0,
            header: UsbHeader::Command as u8,
        }
    }

    async fn send_error_code(&self, op_code: Option<u8>, error_code: ErrorCode) {
        let mut buffer = Packet::new();

        match op_code {
            Some(value) => {
                buffer.push(self.header).push(error_code as u8).push(value);
            }
            None => {
                buffer.push(self.header).push(error_code as u8);
            }
        };

        CMD_CHANNEL.send(buffer).await;
    }

    pub fn read<T: FromLeBytes>(&mut self) -> Option<T> {
        let size = T::SIZE;
        let bytes = self.data.get(self.cursor..self.cursor + size)?;
        let val = T::from_le_bytes(bytes)?;

        self.cursor += size;
        Some(val)
    }

    #[allow(unused)]
    pub fn read_f32(&mut self) -> Option<f32> {
        let val: f32 = self.read()?;
        if val.is_finite() { Some(val) } else { None }
    }

    pub async fn process_command(&mut self) {
        let Some(header): Option<u8> = self.read() else {
            self.send_error_code(None, ErrorCode::ReadByteError).await;
            return;
        };

        if header != self.header {
            self.send_error_code(None, ErrorCode::InvalidHeaderCode)
                .await;
            return;
        };

        let Some(op_code): Option<u8> = self.read() else {
            self.send_error_code(None, ErrorCode::ReadByteError).await;
            return;
        };

        let Ok(op_enum) = OpCode::try_from(op_code) else {
            self.send_error_code(Some(op_code), ErrorCode::OpCodeNotFound)
                .await;
            return;
        };

        match op_enum {
            OpCode::None => {
                self.send_error_code(Some(op_code), ErrorCode::OpCodeNotFound)
                    .await;
            }
            OpCode::TurnOnLed => {
                let Some(led_id): Option<i32> = self.read() else {
                    self.send_error_code(Some(op_code), ErrorCode::ReadByteError)
                        .await;
                    return;
                };

                if led_id == ONBOARD_LED_ID {
                    let snd = LED_STATUS.sender();
                    snd.send(true);
                    self.send_error_code(Some(op_code), ErrorCode::NoError)
                        .await;
                } else {
                    self.send_error_code(Some(op_code), ErrorCode::InvalidLedId)
                        .await;
                }
            }
            OpCode::TurnOffLed => {
                let Some(led_id): Option<i32> = self.read() else {
                    self.send_error_code(Some(op_code), ErrorCode::ReadByteError)
                        .await;
                    return;
                };

                if led_id == ONBOARD_LED_ID {
                    let snd = LED_STATUS.sender();
                    snd.send(false);
                    self.send_error_code(Some(op_code), ErrorCode::NoError)
                        .await;
                } else {
                    self.send_error_code(Some(op_code), ErrorCode::InvalidLedId)
                        .await;
                }
            }
            OpCode::GetLedStatus => {
                let Some(led_id): Option<i32> = self.read() else {
                    self.send_error_code(Some(op_code), ErrorCode::ReadByteError)
                        .await;
                    return;
                };

                if led_id == ONBOARD_LED_ID {
                    let mut receiver = LED_STATUS.dyn_receiver().unwrap();
                    let led_status = receiver.get().await;

                    let mut buffer = Packet::new();
                    buffer
                        .push(self.header)
                        .push(ErrorCode::NoError as u8)
                        .push(op_code)
                        .push(led_status as u8);

                    CMD_CHANNEL.send(buffer).await;
                } else {
                    self.send_error_code(Some(op_code), ErrorCode::InvalidLedId)
                        .await;
                }
            }
        }
    }
}

/* --------------------------- Command Read Trait -------------------------- */
pub trait FromLeBytes: Sized {
    const SIZE: usize;
    fn from_le_bytes(bytes: &[u8]) -> Option<Self>;
}

impl FromLeBytes for u8 {
    const SIZE: usize = 1;
    fn from_le_bytes(bytes: &[u8]) -> Option<Self> {
        bytes.first().copied()
    }
}

impl FromLeBytes for i32 {
    const SIZE: usize = 4;
    fn from_le_bytes(bytes: &[u8]) -> Option<Self> {
        bytes.try_into().ok().map(i32::from_le_bytes)
    }
}

impl FromLeBytes for f32 {
    const SIZE: usize = 4;
    fn from_le_bytes(bytes: &[u8]) -> Option<Self> {
        bytes.try_into().ok().map(f32::from_le_bytes)
    }
}

impl FromLeBytes for u64 {
    const SIZE: usize = 8;
    fn from_le_bytes(bytes: &[u8]) -> Option<Self> {
        bytes.try_into().ok().map(u64::from_le_bytes)
    }
}
