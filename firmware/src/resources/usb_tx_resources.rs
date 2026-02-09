/*
    USB Communication Resources
*/

use super::*;

/* --------------------------- Packet -------------------------- */
pub struct Packet {
    pub data: [u8; USB_BUFFER_SIZE],
    pub len: usize,
}

impl Packet {
    pub fn new() -> Self {
        Self { data: [0u8; USB_BUFFER_SIZE], len: 0 }
    }
    
    fn push_bytes(&mut self, bytes: &[u8]) {
        let remain = 64 - self.len;
        let to_copy = bytes.len().min(remain);
        self.data[self.len..self.len + to_copy].copy_from_slice(&bytes[..to_copy]);
        self.len += to_copy;
    }
    
    pub fn push<T: ToLeBytes>(&mut self, value: T) -> &mut Self {
        value.push_to_packet(self);
        self
    }
    
    pub fn as_slice(&self) -> &[u8] {
        &self.data[..self.len]
    }
}

/* --------------------------- Packet Push Trait -------------------------- */
pub trait ToLeBytes {
    fn push_to_packet(self, packet: &mut Packet);
}

impl ToLeBytes for u8 {
    fn push_to_packet(self, packet: &mut Packet) {
        packet.push_bytes(&[self]);
    }
}

impl ToLeBytes for i32 {
    fn push_to_packet(self, packet: &mut Packet) {
        packet.push_bytes(&self.to_le_bytes());
    }
}

impl ToLeBytes for f32 {
    fn push_to_packet(self, packet: &mut Packet) {
        packet.push_bytes(&self.to_le_bytes());
    }
}

impl ToLeBytes for u64 {
    fn push_to_packet(self, packet: &mut Packet) {
        packet.push_bytes(&self.to_le_bytes());
    }
}



