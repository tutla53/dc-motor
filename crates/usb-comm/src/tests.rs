use super::*;
use crate::packet::PacketError;
use crate::reader::ReadError;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Op {
    Led,
}
impl CommandOpcode for Op {
    fn decode(value: u8) -> Option<Self> {
        match value {
            1 => Some(Self::Led),
            _ => None,
        }
    }
}

#[test]
fn led_wire_packet_round_trip() {
    let mut packet = Packet::<6>::new();
    packet
        .push(0xff_u8)
        .unwrap()
        .push(1_u8)
        .unwrap()
        .push(25_i32)
        .unwrap();
    assert_eq!(packet.as_slice(), &[0xff, 1, 25, 0, 0, 0]);
    let mut command = CommandReader::<Op>::new(packet.as_slice(), 0xff).unwrap();
    assert_eq!(command.opcode(), Op::Led);
    assert_eq!(command.read::<i32>(), Ok(25));
    assert_eq!(command.finish(), Ok(()));
}

#[test]
fn overflowing_write_is_atomic() {
    let mut packet = Packet::<4>::from_slice(&[7]).unwrap();
    assert_eq!(packet.push(42_u32).unwrap_err(), PacketError::BufferFull);
    assert_eq!(packet.as_slice(), &[7]);
    packet.clear();
    packet.push(u32::MAX).unwrap();
    assert_eq!(packet.as_slice(), &[255; 4]);
}

#[test]
fn truncated_reads_do_not_advance() {
    let mut reader = ByteReader::new(&[1, 2]);
    assert_eq!(reader.read::<u32>(), Err(ReadError::UnexpectedEnd));
    assert_eq!(reader.position(), 0);
    assert_eq!(reader.read::<u16>(), Ok(513));
}

#[test]
fn invalid_values_do_not_advance() {
    let mut reader = ByteReader::new(&[2]);
    assert_eq!(reader.read::<bool>(), Err(ReadError::InvalidValue));
    assert_eq!(reader.position(), 0);
    for value in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
        let bytes = value.to_le_bytes();
        let mut reader = ByteReader::new(&bytes);
        assert_eq!(reader.read_f32(), Err(ReadError::NonFiniteFloat));
        assert_eq!(reader.position(), 0);
    }
}

#[test]
fn invalid_headers_and_reserved_opcodes_are_rejected() {
    assert!(matches!(
        CommandReader::<Op>::new(&[], 0xff),
        Err(CommandError::Read(ReadError::UnexpectedEnd))
    ));
    assert!(matches!(
        CommandReader::<Op>::new(&[0xff], 0xff),
        Err(CommandError::Read(ReadError::UnexpectedEnd))
    ));
    assert!(matches!(
        CommandReader::<Op>::new(&[0xfe, 1], 0xff),
        Err(CommandError::InvalidHeader)
    ));
    for opcode in [0, 2, 255] {
        assert!(
            matches!(CommandReader::<Op>::new(&[0xff, opcode], 0xff), Err(CommandError::UnknownOpcode(value)) if value == opcode)
        );
    }
}

#[test]
fn payload_length_must_match() {
    let mut command = CommandReader::<Op>::new(&[0xff, 1, 25], 0xff).unwrap();
    assert_eq!(
        command.read::<i32>(),
        Err(CommandError::Read(ReadError::UnexpectedEnd))
    );
    let command = CommandReader::<Op>::new(&[0xff, 1, 0], 0xff).unwrap();
    assert_eq!(command.finish(), Err(CommandError::UnexpectedTrailingBytes));
}

#[test]
fn signed_float_and_boolean_encoding() {
    let mut packet = Packet::<13>::new();
    packet
        .push(-123_i64)
        .unwrap()
        .push(1.25_f32)
        .unwrap()
        .push(true)
        .unwrap();
    let mut reader = ByteReader::new(packet.as_slice());
    assert_eq!(reader.read::<i64>(), Ok(-123));
    assert_eq!(reader.read_f32(), Ok(1.25));
    assert_eq!(reader.read::<bool>(), Ok(true));
    assert!(reader.is_finished());
}
