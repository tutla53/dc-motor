use usb_comm::CommandOpcode;
use usb_comm::CommandReader;

// Exercise the exported macro from a separate crate, like the playground.
usb_comm::create_opcode_enum! {
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    #[repr(u8)]
    pub enum OpCode {
        None = 0,
        Read = 3,
        Write = 17,
    }
}

impl CommandOpcode for OpCode {
    fn decode(value: u8) -> Option<Self> {
        match Self::try_from(value).ok()? {
            Self::None => Option::None,
            opcode => Some(opcode),
        }
    }
}

// Also accepts private enums and no trailing comma.
usb_comm::create_opcode_enum! {
    #[derive(Debug, PartialEq, Eq)]
    enum PrivateOpcode { Ping = 255 }
}

#[test]
fn exported_macro_converts_wire_values() {
    assert_eq!(OpCode::try_from(3), Ok(OpCode::Read));
    assert_eq!(OpCode::try_from(17), Ok(OpCode::Write));
    assert_eq!(OpCode::try_from(4), Err(()));
    assert_eq!(PrivateOpcode::try_from(255), Ok(PrivateOpcode::Ping));
}

#[test]
fn application_controls_reserved_opcode_policy() {
    assert_eq!(OpCode::try_from(0), Ok(OpCode::None));
    assert!(CommandReader::<OpCode>::new(&[0xff, 0], 0xff).is_err());
    let mut command = CommandReader::<OpCode>::new(&[0xff, 17, 42], 0xff).unwrap();
    assert_eq!(command.opcode(), OpCode::Write);
    assert_eq!(command.read::<u8>(), Ok(42));
    assert_eq!(command.finish(), Ok(()));
}
