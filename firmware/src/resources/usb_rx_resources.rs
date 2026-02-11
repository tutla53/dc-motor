/*
* USB Handler
    Input Command Pattern
        [HEADER] [OP_CODE] [PARAMETERS]
    Output Pattern
        [HEADER] [ERROR_CODE] [OP_CODE] [DATA]
*/

// Resources
use super::*;

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
            _ => Err(()),
        }
    }
}

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
            header : HEADER::COMMAND as u8,
         }
    }

    async fn send_error_code(&self, op_code: Option<u8>, error_code:ErrorCode) {
        let mut buffer = Packet::new();

        match op_code {
            Some(value) => {
                buffer.push(self.header)
                    .push(error_code as u8)
                    .push(value);
            },
            None => {
                buffer.push(self.header)
                    .push(error_code as u8);
            }
        };

        CMD_CHANNEL.send(buffer).await;
    }

    pub fn read <T:FromLeBytes> (&mut self) -> Option<T> {
        let size = T::SIZE;
        let bytes = self.data.get(self.cursor..self.cursor + size)?;
        let val = T::from_le_bytes(bytes)?;
        
        self.cursor += size;
        Some(val)
    }

    pub fn read_f32(&mut self) -> Option<f32> {
        let val: f32 = self.read()?;
        if val.is_finite() {
            Some(val)
        } else {
            None
        }
    }

    fn select_motor(&self, motor_id: Option<u8>) -> Option<&'static MotorHandler> {
        match motor_id {
            Some(id) if (id as usize) < N_MOTOR => { Some(&MOTOR[id as usize]) },
            Some(_) | None => { None },
        }
    }

    pub async fn process_command(&mut self) {
        
        let Some(header): Option<u8> = self.read() else { 
            self.send_error_code(None, ErrorCode::ReadByteError).await;
            return;
        };
        
        if header != self.header {
            self.send_error_code(None, ErrorCode::InvalidHeaderCode).await;
            return;
        };

        let Some(op_code): Option<u8> = self.read() else { 
            self.send_error_code(None, ErrorCode::ReadByteError).await;
            return;
         };

        let Ok(op_enum) = OpCode::try_from(op_code) else {
            self.send_error_code(Some(op_code), ErrorCode::OpCodeNotFound).await;
            return;
        };

        let motor_id: Option<u8> = self.read();
        let Some(motor) = self.select_motor(motor_id) else {
            self.send_error_code(Some(op_code), ErrorCode::InvalidMotorId).await;
            return;
        };

        match op_enum {
            OpCode::StartLogger => {
                /* start_logger
                    time_sampling (u64) = 8
                */
                let time_sampling_ms: u64 =  match self.read() {
                    None => {
                        self.send_error_code(Some(op_code), ErrorCode::ReadByteError).await;
                        return;
                    }, 
                    Some(0) => {
                        self.send_error_code(Some(op_code), ErrorCode::InvalidTimeSampling).await;
                        return;
                    },
                    Some(val) => {
                        val
                    },
                };
                
                LOGGER.set_motor_id(motor.id);
                LOGGER.set_logging_time_sampling(time_sampling_ms);
                LOGGER.set_logging_state(true);
                self.send_error_code(Some(op_code), ErrorCode::NoError).await;
            },
            OpCode::StopLogger => { 
                /* stop_logger */

                LOGGER.set_logging_state(false);
                LOGGER.set_motor_id(255); // Set 255 for None
                LOGGER.log_tx_buffer.clear();

                self.send_error_code(Some(op_code), ErrorCode::NoError).await;
            },
            OpCode::MoveMotorSpeed => {
                /* move_motor_speed
                    speed (i32) = 4
                */
                let Some(speed): Option<i32> = self.read() else { 
                    self.send_error_code(Some(op_code), ErrorCode::ReadByteError).await;
                    return;
                };

                motor.set_motor_command(
                    MotorCommand::SpeedControl(speed)
                );

                self.send_error_code(Some(op_code), ErrorCode::NoError).await;
            },
            OpCode::MoveMotorAbsPos => {
                /* move_motor_abs_pos
                    pos (i32) = 4
                */
                let Some(pos): Option<i32> = self.read() else { 
                    self.send_error_code(Some(op_code), ErrorCode::ReadByteError).await;
                    return;
                };

                motor.set_motor_command(
                    MotorCommand::PositionControl(Shape::Step(pos))
                );
                
                self.send_error_code(Some(op_code), ErrorCode::NoError).await;
            },
            OpCode::StopMotor => {
                /* stop_motor */
                motor.set_motor_command(MotorCommand::Stop);
                self.send_error_code(Some(op_code), ErrorCode::NoError).await;
            },
            OpCode::SetMotorPosPidParam => {
                /* set_motor_pos_pid_param
                    kp (f32) = 4
                    ki (f32) = 4
                    kd (f32) = 4
                    kp_speed (f32) = 4
                    ki_speed (f32) = 4
                    kd_speed (f32) = 4
                */

                if let (Some(kp), Some(ki), Some(kd), Some(kp_speed), Some(ki_speed), Some(kd_speed)) = (
                    self.read_f32(), self.read_f32(), self.read_f32(),
                    self.read_f32(), self.read_f32(), self.read_f32()
                ) {
                    let config = PosPIDConfig {
                        kp, ki, kd, kp_speed, ki_speed, kd_speed,
                    };
                    motor.set_pos_pid(config).await;
                    self.send_error_code(Some(op_code), ErrorCode::NoError).await;
                }
                else {
                    self.send_error_code(Some(op_code), ErrorCode::NonFiniteFloat).await;
                    return;
                }
            },
            OpCode::GetMotorPosPidParam => {
                /* get_motor_pos_pid_param */
                let pid = motor.get_pos_pid().await;

                let mut buffer = Packet::new();            
                buffer.push(self.header)
                    .push(ErrorCode::NoError as u8)
                    .push(op_code)
                    .push(pid.kp)
                    .push(pid.ki)
                    .push(pid.kd)
                    .push(pid.kp_speed)
                    .push(pid.ki_speed)
                    .push(pid.kd_speed);

                CMD_CHANNEL.send(buffer).await; 
            },
            OpCode::SetMotorSpeedPidParam => {
                /* set_motor_speed_pid_param
                    kp (f32) = 4
                    ki (f32) = 4
                    kd (f32) = 4
                */

                if let (Some(kp), Some(ki), Some(kd)) = (
                    self.read_f32(), self.read_f32(), self.read_f32(),
                ) {
                    let config = PIDConfig {
                        kp, ki, kd,
                    };
                    motor.set_speed_pid(config).await;           
                    self.send_error_code(Some(op_code), ErrorCode::NoError).await;
                }
                else {       
                    self.send_error_code(Some(op_code), ErrorCode::NonFiniteFloat).await;
                    return;
                }
            },
            OpCode::GetMotorSpeedPidParam => {
                /* get_motor_speed_pid_param */
                    let pid = motor.get_speed_pid().await;
                    let mut buffer = Packet::new();            
                    buffer.push(self.header)
                        .push(ErrorCode::NoError as u8)
                        .push(op_code)
                        .push(pid.kp)
                        .push(pid.ki)
                        .push(pid.kd);
                    CMD_CHANNEL.send(buffer).await;                         
            },
            OpCode::MoveMotorAbsPosTrapezoid => {
                /* move_motor_abs_pos_trapezoid
                    target (f32) = 4
                    velocity (f32) = 4
                    acceleration (f32) = 4
                */

                if let (Some(target), Some(velocity), Some(acceleration)) = (
                    self.read_f32(), self.read_f32(), self.read_f32(),
                ) {
                    motor.set_motor_command(
                        MotorCommand::PositionControl(
                            Shape::Trapezoidal(target, velocity, acceleration)
                        )
                    );
                    self.send_error_code(Some(op_code), ErrorCode::NoError).await;
                } 
                else {
                    self.send_error_code(Some(op_code), ErrorCode::NonFiniteFloat).await;
                    return;
                }
            },
            OpCode::GetMotorPos => {
                /* get_motor_pos */
                let motor_pos = motor.get_current_pos();

                let mut buffer = Packet::new();            
                buffer.push(self.header)
                    .push(ErrorCode::NoError as u8)
                    .push(op_code)
                    .push(motor_pos);
                CMD_CHANNEL.send(buffer).await;
            },
            OpCode::GetMotorSpeed => {
                /* get_motor_speed */
                let motor_speed = motor.get_current_speed();

                let mut buffer = Packet::new();            
                buffer.push(self.header)
                    .push(ErrorCode::NoError as u8)
                    .push(op_code)
                    .push(motor_speed);
                CMD_CHANNEL.send(buffer).await;
            },
            OpCode::MoveMotorOpenLoop => {
                /* move_motor_open_loop
                    pwm (i32) = 4
                */
                let Some(pwm): Option<i32> = self.read() else { 
                        self.send_error_code(Some(op_code), ErrorCode::ReadByteError).await;
                    return;
                };

                motor.set_motor_command(MotorCommand::OpenLoop(pwm));
                self.send_error_code(Some(op_code), ErrorCode::NoError).await;
            },
            _ => { 
                self.send_error_code(Some(op_code), ErrorCode::OpCodeNotFound).await;
                return;
            },
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