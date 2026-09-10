/*
* USB Handler
    Input Command Pattern
        [HEADER] [OP_CODE] [PARAMETERS]
    Output Pattern
        [HEADER] [ERROR_CODE] [OP_CODE] [DATA]
*/

// Resources
use super::*;

/* --------------------------- Command Handler-------------------------- */
pub struct CommandHandler<'a> {
    pub data: &'a [u8],
    header: u8,
    command_sender: ChannelSender<'static, CriticalSectionRawMutex, Packet, DATA_CHANNEL_SIZE>,
}

impl<'a> CommandHandler<'a> {
    pub fn new(
        data: &'a [u8],
        command_sender: ChannelSender<'static, CriticalSectionRawMutex, Packet, DATA_CHANNEL_SIZE>,
    ) -> Self {
        Self {
            data,
            header: UsbHeader::Command as u8,
            command_sender,
        }
    }

    async fn send_error_code(&self, op_code: Option<u8>, error_code: ErrorCode) {
        let mut buffer = Packet::new();

        match op_code {
            Some(value) => {
                buffer
                    .push(self.header)
                    .expect("Data Fits")
                    .push(error_code as u8)
                    .expect("Data Fits")
                    .push(value)
                    .expect("Data Fits");
            }
            None => {
                buffer
                    .push(self.header)
                    .expect("Data Fits")
                    .push(error_code as u8)
                    .expect("Data Fits");
            }
        };

        self.command_sender.send(buffer).await;
    }

    fn select_motor(&self, motor_id: u8) -> Result<&'static MotorHandler, ErrorCode> {
        MOTOR
            .get(motor_id as usize)
            .ok_or(ErrorCode::InvalidMotorId)
    }

    pub async fn process_command(&mut self) {
        if let Err(error) = self.execute_command().await {
            let op_code = if self.data.first().copied() == Some(self.header) {
                self.data.get(1).copied()
            } else {
                None
            };

            self.send_error_code(op_code, error).await;
        }
    }

    async fn execute_command(&self) -> Result<(), ErrorCode> {
        let mut command = CommandReader::<OpCode>::new(self.data, self.header)?;

        let op_enum = command.opcode();
        let op_code = op_enum as u8;

        if op_enum == OpCode::SaveConfiguration {
            command.finish()?;

            for (id, motor) in MOTOR.iter().enumerate() {
                let current_speed_pid: PIDConfig = motor.get_speed_pid().await;
                let current_pos_pid: PIDConfig = motor.get_pos_pid().await;
                let current_max_speed: u32 = motor.get_max_speed();

                let stored_max_speed = StoredMaxSpeed::try_from(current_max_speed)
                    .map_err(|_| ErrorCode::MaxSpeedOutOfRange)?;

                let saved_speed =
                    save_config(id as u8, ConfigType::SpeedPID, &current_speed_pid).await;
                let saved_pos =
                    save_config(id as u8, ConfigType::PositionPID, &current_pos_pid).await;
                let saved_max_speed =
                    save_config(id as u8, ConfigType::MaxSpeed, &stored_max_speed).await;

                if saved_speed.is_err() || saved_pos.is_err() || saved_max_speed.is_err() {
                    return Err(ErrorCode::FlashStorageError);
                }
            }

            self.send_error_code(Some(op_code), ErrorCode::NoError)
                .await;
            return Ok(());
        } else if op_enum == OpCode::SetToDefaultConfig {
            command.finish()?;

            for (id, motor) in MOTOR.iter().enumerate() {
                let default_speed_pid = motor.default_speed_pid;
                let default_pos_pid = motor.default_pos_pid;
                let default_max_speed = motor.default_max_speed;

                let stored_default_max_speed = StoredMaxSpeed::try_from(default_max_speed)
                    .map_err(|_| ErrorCode::MaxSpeedOutOfRange)?;

                motor.set_speed_pid(default_speed_pid).await;
                motor.set_pos_pid(default_pos_pid).await;
                motor.set_max_speed(default_max_speed);

                let saved_speed =
                    save_config(id as u8, ConfigType::SpeedPID, &default_speed_pid).await;
                let saved_pos =
                    save_config(id as u8, ConfigType::PositionPID, &default_pos_pid).await;
                let saved_max_speed =
                    save_config(id as u8, ConfigType::MaxSpeed, &stored_default_max_speed).await;

                if saved_speed.is_err() || saved_pos.is_err() || saved_max_speed.is_err() {
                    return Err(ErrorCode::FlashStorageError);
                }
            }

            self.send_error_code(Some(op_code), ErrorCode::NoError)
                .await;
            return Ok(());
        } else if op_enum == OpCode::GetFirmwareVersion {
            command.finish()?;

            let mut buffer = Packet::new();

            buffer
                .push(self.header)?
                .push(ErrorCode::NoError as u8)?
                .push(op_code)?
                .push(FW_VERSION_MAJOR)?
                .push(FW_VERSION_MINOR)?
                .push(FW_VERSION_PATCH)?;

            self.command_sender.send(buffer).await;
            return Ok(());
        }

        let motor_id = command.read::<u8>()?;
        let motor = self.select_motor(motor_id)?;

        match op_enum {
            OpCode::StartLogger => {
                /* start_logger
                    time_sampling (u64) = 8
                */
                let time_sampling_ms = command.read::<u64>()?;
                command.finish()?;

                if time_sampling_ms == 0 {
                    return Err(ErrorCode::InvalidTimeSampling);
                }

                LOGGER.set_motor_id(motor.id);
                LOGGER.set_logging_time_sampling(time_sampling_ms);
                LOGGER.set_logging_state(true);
                self.send_error_code(Some(op_code), ErrorCode::NoError)
                    .await;
            }
            OpCode::StopLogger => {
                /* stop_logger */
                command.finish()?;

                LOGGER.set_logging_state(false);
                LOGGER.set_motor_id(255); // Set 255 for None
                LOGGER.log_tx_buffer.clear();

                self.send_error_code(Some(op_code), ErrorCode::NoError)
                    .await;
            }
            OpCode::MoveMotorSpeed => {
                /* move_motor_speed
                    speed (i32) = 4
                */
                let speed = command.read::<i32>()?;
                command.finish()?;

                if !motor.is_motion_enabled() {
                    return Err(ErrorCode::MotorIsDisabled);
                }

                motor
                    .try_set_motor_command(MotorCommand::SpeedControl(speed))
                    .map_err(|_| ErrorCode::MotorCommandChannelFull)?;

                self.send_error_code(Some(op_code), ErrorCode::NoError)
                    .await;
            }
            OpCode::MoveMotorAbsPos => {
                /* move_motor_abs_pos
                    pos (i32) = 4
                */
                let pos = command.read::<i32>()?;
                command.finish()?;

                if !motor.is_motion_enabled() {
                    return Err(ErrorCode::MotorIsDisabled);
                }

                motor
                    .try_set_motor_command(MotorCommand::PositionControl(Shape::Step(pos)))
                    .map_err(|_| ErrorCode::MotorCommandChannelFull)?;

                motor.set_move_done(false);

                self.send_error_code(Some(op_code), ErrorCode::NoError)
                    .await;
            }
            OpCode::StopMotor => {
                /* stop_motor */
                command.finish()?;

                motor
                    .try_set_motor_command(MotorCommand::Stop)
                    .map_err(|_| ErrorCode::MotorCommandChannelFull)?;

                self.send_error_code(Some(op_code), ErrorCode::NoError)
                    .await;
            }
            OpCode::SetMotorPosPidParam => {
                /* set_motor_pos_pid_param
                    kp (f32) = 4
                    ki (f32) = 4
                    kd (f32) = 4
                    i_limit (f32) = 4
                */
                let kp = command.read_f32()?;
                let ki = command.read_f32()?;
                let kd = command.read_f32()?;
                let i_limit = command.read_f32()?;
                command.finish()?;

                let config = PIDConfig {
                    kp,
                    ki,
                    kd,
                    i_limit,
                };

                if !motor.set_pos_pid(config).await {
                    return Err(ErrorCode::InvalidPidValue);
                }

                self.send_error_code(Some(op_code), ErrorCode::NoError)
                    .await;
            }
            OpCode::GetMotorPosPidParam => {
                /* get_motor_pos_pid_param */
                command.finish()?;
                let pid = motor.get_pos_pid().await;

                let mut buffer = Packet::new();
                buffer
                    .push(self.header)?
                    .push(ErrorCode::NoError as u8)?
                    .push(op_code)?
                    .push(pid.kp)?
                    .push(pid.ki)?
                    .push(pid.kd)?
                    .push(pid.i_limit)?;

                self.command_sender.send(buffer).await;
            }
            OpCode::SetMotorSpeedPidParam => {
                /* set_motor_speed_pid_param
                    kp (f32) = 4
                    ki (f32) = 4
                    kd (f32) = 4
                    i_limit (f32) = 4
                */
                let kp = command.read_f32()?;
                let ki = command.read_f32()?;
                let kd = command.read_f32()?;
                let i_limit = command.read_f32()?;
                command.finish()?;

                let config = PIDConfig {
                    kp,
                    ki,
                    kd,
                    i_limit,
                };

                if !motor.set_speed_pid(config).await {
                    return Err(ErrorCode::InvalidPidValue);
                }

                self.send_error_code(Some(op_code), ErrorCode::NoError)
                    .await;
            }
            OpCode::GetMotorSpeedPidParam => {
                /* get_motor_speed_pid_param */
                command.finish()?;
                let pid = motor.get_speed_pid().await;

                let mut buffer = Packet::new();
                buffer
                    .push(self.header)?
                    .push(ErrorCode::NoError as u8)?
                    .push(op_code)?
                    .push(pid.kp)?
                    .push(pid.ki)?
                    .push(pid.kd)?
                    .push(pid.i_limit)?;

                self.command_sender.send(buffer).await;
            }
            OpCode::MoveMotorAbsPosTrapezoid => {
                /* move_motor_abs_pos_trapezoid
                    target (i32) = 4
                    velocity (i32) = 4
                    acceleration (i32) = 4
                */
                let target = command.read::<i32>()?;
                let velocity = command.read::<i32>()?;
                let acceleration = command.read::<i32>()?;
                command.finish()?;

                if velocity == 0 || acceleration == 0 {
                    // Unacceptable Velocity and Acceleration
                    return Err(ErrorCode::ZeroDivisionError);
                }

                let velocity = velocity
                    .checked_abs()
                    .ok_or(ErrorCode::InvalidMotionParameter)?;

                let acceleration = acceleration
                    .checked_abs()
                    .ok_or(ErrorCode::InvalidMotionParameter)?;

                if !motor.is_motion_enabled() {
                    return Err(ErrorCode::MotorIsDisabled);
                }

                motor
                    .try_set_motor_command(MotorCommand::PositionControl(Shape::Trapezoidal(
                        I32F32::from_num(target),
                        I32F32::from_num(velocity),
                        I32F32::from_num(acceleration),
                    )))
                    .map_err(|_| ErrorCode::MotorCommandChannelFull)?;

                motor.set_move_done(false);

                self.send_error_code(Some(op_code), ErrorCode::NoError)
                    .await;
            }
            OpCode::GetMotorPos => {
                /* get_motor_pos */
                command.finish()?;
                let motor_pos = motor.get_current_pos();

                let mut buffer = Packet::new();
                buffer
                    .push(self.header)?
                    .push(ErrorCode::NoError as u8)?
                    .push(op_code)?
                    .push(motor_pos)?;

                self.command_sender.send(buffer).await;
            }
            OpCode::GetMotorSpeed => {
                /* get_motor_speed */
                command.finish()?;
                let motor_speed = motor.get_current_speed();

                let mut buffer = Packet::new();
                buffer
                    .push(self.header)?
                    .push(ErrorCode::NoError as u8)?
                    .push(op_code)?
                    .push(motor_speed)?;

                self.command_sender.send(buffer).await;
            }
            OpCode::MoveMotorOpenLoop => {
                /* move_motor_open_loop
                    pwm (i32) = 4
                */
                let pwm = command.read::<i32>()?;
                command.finish()?;

                if !motor.is_motion_enabled() {
                    return Err(ErrorCode::MotorIsDisabled);
                }

                motor
                    .try_set_motor_command(MotorCommand::OpenLoop(pwm))
                    .map_err(|_| ErrorCode::MotorCommandChannelFull)?;

                self.send_error_code(Some(op_code), ErrorCode::NoError)
                    .await;
            }
            OpCode::GetMotorMaxSpeed => {
                command.finish()?;
                let motor_max_speed = motor.get_max_speed();

                let mut buffer = Packet::new();
                buffer
                    .push(self.header)?
                    .push(ErrorCode::NoError as u8)?
                    .push(op_code)?
                    .push(motor_max_speed)?;

                self.command_sender.send(buffer).await;
            }
            OpCode::SetMotorMaxSpeed => {
                let motor_max_speed = command.read::<u32>()?;
                command.finish()?;

                if !motor.set_max_speed(motor_max_speed) {
                    return Err(ErrorCode::MaxSpeedOutOfRange);
                }

                self.send_error_code(Some(op_code), ErrorCode::NoError)
                    .await;
            }
            OpCode::SetMotorEnable => {
                command.finish()?;
                motor.request_motor_enable();
                self.send_error_code(Some(op_code), ErrorCode::NoError)
                    .await;
            }
            OpCode::SetMotorDisable => {
                command.finish()?;
                motor.request_motor_disable();
                self.send_error_code(Some(op_code), ErrorCode::NoError)
                    .await;
            }
            OpCode::GetMotorEnable => {
                command.finish()?;
                let motor_enable = motor.is_motion_enabled();

                let mut buffer = Packet::new();
                buffer
                    .push(self.header)?
                    .push(ErrorCode::NoError as u8)?
                    .push(op_code)?
                    .push(u8::from(motor_enable))?;

                self.command_sender.send(buffer).await;
            }
            _ => {
                return Err(ErrorCode::OpCodeNotFound);
            }
        }

        Ok(())
    }
}
