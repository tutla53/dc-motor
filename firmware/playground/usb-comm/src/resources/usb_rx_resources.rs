/*
* USB Command Handler
*/
use super::*;

pub struct CommandHandler<'a> {
    data: &'a [u8],
    command_sender: ChannelSender<'static, CriticalSectionRawMutex, Packet, DATA_CHANNEL_SIZE>,
}

impl<'a> CommandHandler<'a> {
    pub fn new(
        data: &'a [u8],
        command_sender: ChannelSender<'static, CriticalSectionRawMutex, Packet, DATA_CHANNEL_SIZE>,
    ) -> Self {
        Self {
            data,
            command_sender,
        }
    }

    /* --------------------------- Command Parsing -------------------------- */
    fn decode_command(&self) -> Result<OpCode, ErrorCode> {
        let mut command = CommandReader::<OpCode>::new(self.data, UsbHeader::Command as u8)?;
        let opcode = command.opcode();
        let led_id = command.read::<i32>()?;
        if led_id != ONBOARD_LED_ID {
            return Err(ErrorCode::InvalidLedId);
        }
        command.finish()?;
        Ok(opcode)
    }

    /* --------------------------- Response Builder -------------------------- */
    async fn send_response(&self, opcode: Option<u8>, error: ErrorCode, status: Option<bool>) {
        let mut packet = Packet::new();
        // All responses fit in four bytes; the configured capacity is 64.
        packet
            .push(UsbHeader::Command as u8)
            .expect("response fits")
            .push(error as u8)
            .expect("response fits");
        if let Some(opcode) = opcode {
            packet.push(opcode).expect("response fits");
        }
        if let Some(status) = status {
            packet.push(status).expect("response fits");
        }
        self.command_sender.send(packet).await;
    }

    async fn send_error_code(&self, opcode: Option<u8>, error: ErrorCode) {
        self.send_response(opcode, error, None).await;
    }

    /* --------------------------- Command Processing -------------------------- */
    pub async fn process_command(&mut self) {
        let opcode = match self.decode_command() {
            Ok(value) => value,
            Err(error) => {
                self.send_error_code(self.data.get(1).copied(), error).await;
                return;
            }
        };
        match opcode {
            OpCode::TurnOnLed | OpCode::TurnOffLed => {
                LED_STATUS.sender().send(opcode == OpCode::TurnOnLed);
                self.send_error_code(Some(opcode as u8), ErrorCode::NoError)
                    .await;
            }
            OpCode::GetLedStatus => {
                let mut receiver = LED_STATUS.dyn_receiver().unwrap();
                let status = receiver.get().await;
                self.send_response(Some(opcode as u8), ErrorCode::NoError, Some(status))
                    .await;
            }
            OpCode::None => {
                self.send_error_code(Some(opcode as u8), ErrorCode::OpCodeNotFound)
                    .await;
            }
        }
    }
}
