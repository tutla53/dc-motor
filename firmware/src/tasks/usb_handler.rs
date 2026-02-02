/*
* USB Handler
*/

// Resources
use crate::resources::global_resources::MotorCommand;
use crate::resources::global_resources::Shape;
use crate::resources::global_resources::PIDConfig;
use crate::resources::global_resources::PosPIDConfig;
use crate::resources::global_resources::MotorHandler;
use crate::resources::global_resources::MOTOR_0;
use crate::resources::global_resources::LOGGER;
use crate::resources::global_resources::USB_TX_CHANNEL;
use crate::resources::global_resources::Packet;
use crate::resources::global_resources::USB_RX_BUFFER_SIZE;

// Library
use defmt_rtt as _;
use panic_probe as _;
use embassy_usb::class::cdc_acm::CdcAcmClass;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::Driver;
use embassy_futures::select::select;
use embassy_futures::select::Either;

/* --------------------------- Code -------------------------- */
#[embassy_executor::task]
pub async fn usb_device_task(mut usb: embassy_usb::UsbDevice<'static, Driver<'static, USB>>) {
    usb.run().await;
}

#[embassy_executor::task]
pub async fn usb_communication_task(mut class: CdcAcmClass<'static, Driver<'static, USB>>) {
    let mut rx_buf = [0u8; USB_RX_BUFFER_SIZE];
    let subscriber = USB_TX_CHANNEL.receiver();

    loop {
        class.wait_connection().await;
        
        loop {
            match select(class.read_packet(&mut rx_buf), subscriber.receive()).await {
                Either::First(result) => {
                    match result {
                        Ok(len) => {
                            let data = &rx_buf[..len];
                            if !data.is_empty() {
                                let handler = CommandHandler::new(data);
                                handler.process_command().await;
                            }
                        }
                        Err(_) => break,
                    }
                }
                Either::Second(packet) => {
                    let _ = class.write_packet(&packet.data[..packet.len]).await;
                }
            }
        }
    }
}

pub trait ByteReader {
    fn data(&self) -> &[u8];

    fn read_u8(&self, offset: usize) -> u8 {
        self.data()[offset]
    }

    fn read_i32(&self, offset: usize) -> i32 {
        i32::from_le_bytes(self.data()[offset..offset + 4].try_into().unwrap_or([0; 4]))
    }

    fn read_f32(&self, offset: usize) -> f32 {
        f32::from_le_bytes(self.data()[offset..offset + 4].try_into().unwrap_or([0; 4]))
    }
    
    fn read_u64(&self, offset: usize) -> u64 {
        u64::from_le_bytes(self.data()[offset..offset + 8].try_into().unwrap_or([0; 8]))
    }
}

impl<'a> ByteReader for CommandHandler<'a> {
    fn data(&self) -> &[u8] {
        self.data
    }
}

pub struct CommandHandler<'a> {
    pub data: &'a [u8],
}

impl<'a> CommandHandler<'a> {
    pub fn new(data: &'a [u8]) -> Self {
        Self { data }
    }

    pub async fn process_command(&self) {
        match self.data[0] {
            1 => { self.start_logger().await; },
            2 => { self.stop_logger().await; },
            3 => { self.move_motor_speed().await; },
            4 => { self.move_motor_abs_pos().await; },
            5 => { self.stop_motor().await; },
            6 => { self.set_motor_pos_pid_param().await; },
            7 => { self.get_motor_pos_pid_param().await; },
            8 => { self.set_motor_speed_pid_param().await; },
            9 => { self.get_motor_speed_pid_param(). await; },
            10 => { self.get_logged_item(); },
            11 => { self.clear_logged_item(); },
            12 => { self.move_motor_abs_pos_trapezoid().await; }
            13 => { self.get_motor_pos().await; }
            14 => { self.get_motor_speed().await; }
            15 => { self.move_motor_open_loop().await;}
            _ => { let _ = USB_TX_CHANNEL.sender().try_send(Packet::from_str("ERR: Unknown Cmd\n")); },
        }
    }
    
    fn select_motor(&self, id: u8) -> Option<&'static MotorHandler> {
        if id == 0 {
            return Some(&MOTOR_0);
        }
        else {
            log::info!("selected_motor is not available");
            return None 
        }
    }

    async fn start_logger(&self) {
        /*
            OP = 1
            time_sampling (u64) = 8
        */

        if self.data.len() < 9 {
            log::info!("Insufficient Argument(s): [time_sampling: u64]");
            return;
        }
        
        let time_sampling_ms = self.read_u64(1);
    
        LOGGER.set_logging_time_sampling(time_sampling_ms).await;
        LOGGER.set_logging_state(true).await;
        LOGGER.add_log_request(false); 
    }

    async fn stop_logger(&self) {
        LOGGER.set_logging_state(false).await;
    }

    async fn move_motor_open_loop(&self) {
        /*
            OP = 1
            motor_id (u8) = 1
            pwm (i32) = 4
        */

        if self.data.len() < 6 {
            log::info!("Insufficient Argument(s): [motor_id: u8, pwm: i32]");
            return;
        }

        let motor_id = self.read_u8(1);
        let pwm = self.read_i32(5);
        
        match self.select_motor(motor_id) {
            Some(selected_motor) => {
                selected_motor.set_motor_command(MotorCommand::OpenLoop(pwm)).await; 
            },
            None => {
                let _ = USB_TX_CHANNEL.sender().try_send(Packet::from_str("ERR: Unknown selected_motor ID\n"));
            },
        }
    }

    async fn move_motor_speed(&self) {
        /*
            OP = 1
            motor_id (u8) = 1
            speed (i32) = 4
        */

        if self.data.len() < 6 {
            log::info!("Insufficient Argument(s): [motor_id: u8, speed: i32]");
            return;
        }
        
        let motor_id = self.read_u8(1);
        let speed = self.read_i32(5);
        
        match self.select_motor(motor_id) {
            Some(selected_motor) => {
                selected_motor.set_motor_command(MotorCommand::SpeedControl(Shape::Step(speed))).await; 
            },
            None => {
                let _ = USB_TX_CHANNEL.sender().try_send(Packet::from_str("ERR: Unknown selected_motor ID\n"));
            },
        }
    }

    async fn move_motor_abs_pos(&self) {
        /*
            OP = 1
            motor_id (u8) = 1
            pos (i32) = 4
        */

        if self.data.len() < 6 {
            log::info!("Insufficient Argument(s): [motor_id: u8, pos: i32]");
            return;
        }

        let motor_id = self.read_u8(1);
        let pos = self.read_i32(5);
        
        match self.select_motor(motor_id) {
            Some(selected_motor) => {
                selected_motor.set_motor_command(MotorCommand::PositionControl(Shape::Step(pos))).await; 
            },
            None => {
                let _ = USB_TX_CHANNEL.sender().try_send(Packet::from_str("ERR: Unknown selected_motor ID\n"));
            },
        }
    }

    async fn stop_motor(&self) {
        /*
            OP = 1
            motor_id (u8) = 1
        */

        if self.data.len() < 2 {
            log::info!("Insufficient Argument(s): [motor_id: u8]");
            return;
        }

        let motor_id = self.read_u8(1);

        match self.select_motor(motor_id) {
            Some(selected_motor) => {
                selected_motor.set_motor_command(MotorCommand::Stop).await; 
            },
            None => {
                let _ = USB_TX_CHANNEL.sender().try_send(Packet::from_str("ERR: Unknown selected_motor ID\n"));
            },
        }
    }

    async fn set_motor_pos_pid_param(&self) {
        /*
            OP = 1
            motor_id (u8) = 1
            kp (f32) = 4
            ki (f32) = 4
            kd (f32) = 4
            kp_speed (f32) = 4
            ki_speed (f32) = 4
            kd_speed (f32) = 4
        */

        if self.data.len() < 26 {
            log::info!("Insufficient Argument(s): [motor_id: u8, kp: f32, ki: f32, kd:f32, kp_speed: f32, ki_speed: f32, kd_speed:f32]");
            return;
        }

        let motor_id = self.read_u8(1);

        let config = PosPIDConfig {
            kp: self.read_f32(2),
            ki: self.read_f32(6),
            kd: self.read_f32(10),
            kp_speed: self.read_f32(14),
            ki_speed: self.read_f32(18),
            kd_speed: self.read_f32(22),            
        };

        match self.select_motor(motor_id) {
            Some(selected_motor) => {
                selected_motor.set_pos_pid(config).await;
            },
            None => {
                let _ = USB_TX_CHANNEL.sender().try_send(Packet::from_str("ERR: Unknown selected_motor ID\n"));
            },
        }
    }

    async fn get_motor_pos_pid_param(&self) {
        /*
            OP = 1
            motor_id (u8) = 1
        */

        if self.data.len() < 2 {
            log::info!("Insufficient Argument(s): [motor_id: u8]");
            return;
        }

        let motor_id = self.read_u8(1);

        match self.select_motor(motor_id) {
            Some(selected_motor) => {
                let pid = selected_motor.get_pos_pid().await;

                let mut buffer = Packet::new();            
                buffer.push_bytes(&[0x07]);
                buffer.push_bytes(&pid.kp.to_le_bytes());
                buffer.push_bytes(&pid.ki.to_le_bytes());
                buffer.push_bytes(&pid.kd.to_le_bytes());
                buffer.push_bytes(&pid.kp_speed.to_le_bytes());
                buffer.push_bytes(&pid.ki_speed.to_le_bytes());
                buffer.push_bytes(&pid.kd_speed.to_le_bytes());
                let _ = USB_TX_CHANNEL.sender().try_send(buffer);
            },
            None => {
                let _ = USB_TX_CHANNEL.sender().try_send(Packet::from_str("ERR: Unknown selected_motor ID\n"));
            },
        }
    }

    async fn set_motor_speed_pid_param(&self) {
        /*
            OP = 1
            motor_id (u8) = 1
            kp (f32) = 4
            ki (f32) = 4
            kd (f32) = 4
        */

        if self.data.len() < 14 {
            log::info!("Insufficient Argument(s): [motor_id: u8, kp: f32, ki: f32, kd:f32]");
            return;
        }

        let motor_id = self.read_u8(1);

        let config = PIDConfig {
            kp: self.read_f32(2),
            ki: self.read_f32(6),
            kd: self.read_f32(10),            
        };

        match self.select_motor(motor_id) {
            Some(selected_motor) => {
                selected_motor.set_speed_pid(config).await;
            },
            None => {
                let _ = USB_TX_CHANNEL.sender().try_send(Packet::from_str("ERR: Unknown selected_motor ID\n"));
            },
        }
    }

    async fn get_motor_speed_pid_param(&self) {
        /*
            OP = 1
            motor_id (u8) = 1
        */

        if self.data.len() < 2 {
            log::info!("Insufficient Argument(s): [motor_id: u8]");
            return;
        }

        let motor_id = self.read_u8(1);

        match self.select_motor(motor_id) {
            Some(selected_motor) => {
                let pid = selected_motor.get_speed_pid().await;
                
                let mut buffer = Packet::new();
                buffer.push_bytes(&[0x09]);
                buffer.push_bytes(&pid.kp.to_le_bytes());
                buffer.push_bytes(&pid.ki.to_le_bytes());
                buffer.push_bytes(&pid.kd.to_le_bytes());
                let _ = USB_TX_CHANNEL.sender().try_send(buffer);

            },
            None => {
                let _ = USB_TX_CHANNEL.sender().try_send(Packet::from_str("ERR: Unknown selected_motor ID\n"));
            },
        }
    }

    fn get_logged_item(&self) {
        let _ = LOGGER.add_log_request(true);
    }

    fn clear_logged_item(&self) {
        let _ = LOGGER.add_log_request(false);
    }

    async fn move_motor_abs_pos_trapezoid(&self) {
        /*
            OP = 1
            motor_id (u8) = 1
            target (f32) = 4
            velocity (f32) = 4
            acceleration (f32) = 4
        */

        if self.data.len() < 14 {
            log::info!("Insufficient Argument(s): [motor_id: u8, target: f32, velocity: f32, acceleration: f32]");
            return;
        }
        
        let motor_id = self.read_u8(1);
        let target = self.read_f32(2);
        let velocity = self.read_f32(6);
        let acceleration = self.read_f32(10);

        match self.select_motor(motor_id) {
            Some(selected_motor) => {
                selected_motor.set_motor_command(MotorCommand::PositionControl(Shape::Trapezoidal(target, velocity, acceleration))).await;
            },
            None => {
                let _ = USB_TX_CHANNEL.sender().try_send(Packet::from_str("ERR: Unknown selected_motor ID\n"));
            },
        }
    }

    async fn get_motor_pos(&self) {
        /*
            OP = 1
            motor_id (u8) = 1
        */

        if self.data.len() < 2 {
            log::info!("Insufficient Argument(s): [motor_id: u8]");
            return;
        }

        let motor_id = self.read_u8(1);
        
        match self.select_motor(motor_id) {
            Some(selected_motor) => {
                let motor_pos = selected_motor.get_current_pos().await;

                let mut buffer = Packet::new();            
                buffer.push_bytes(&[0x13]);
                buffer.push_bytes(&motor_pos.to_le_bytes());
                let _ = USB_TX_CHANNEL.sender().try_send(buffer);
            },
            None => {
                let _ = USB_TX_CHANNEL.sender().try_send(Packet::from_str("ERR: Unknown selected_motor ID\n"));
            },
        }
    }

    async fn get_motor_speed(&self) {
        /*
            OP = 1
            motor_id (u8) = 1
        */

        if self.data.len() < 2 {
            log::info!("Insufficient Argument(s): [motor_id: u8]");
            return;
        }

        let motor_id = self.read_u8(1);

        match self.select_motor(motor_id) {
            Some(selected_motor) => {
                let motor_speed = selected_motor.get_current_speed().await;

                let mut buffer = Packet::new();            
                buffer.push_bytes(&[0x14]);
                buffer.push_bytes(&motor_speed.to_le_bytes());
                let _ = USB_TX_CHANNEL.sender().try_send(buffer);
            },
            None => {
                let _ = USB_TX_CHANNEL.sender().try_send(Packet::from_str("ERR: Unknown selected_motor ID\n"));
            },
        }
    }
}