/*
* Bounded Packet Writer
*/

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PacketError {
    BufferFull,
}

#[derive(Debug, Clone)]
pub struct Packet<const N: usize> {
    data: [u8; N],
    len: usize,
}

impl<const N: usize> Packet<N> {
    pub const fn new() -> Self {
        Self {
            data: [0; N],
            len: 0,
        }
    }

    pub fn from_slice(bytes: &[u8]) -> Result<Self, PacketError> {
        let mut packet = Self::new();
        packet.push_bytes(bytes)?;
        Ok(packet)
    }

    pub fn len(&self) -> usize {
        self.len
    }
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }
    pub const fn capacity(&self) -> usize {
        N
    }
    pub fn remaining_capacity(&self) -> usize {
        N - self.len
    }
    pub fn as_slice(&self) -> &[u8] {
        &self.data[..self.len]
    }
    pub fn clear(&mut self) {
        self.len = 0;
    }

    /// Append the complete value or leave the packet unchanged.
    pub fn push_bytes(&mut self, bytes: &[u8]) -> Result<&mut Self, PacketError> {
        if bytes.len() > self.remaining_capacity() {
            return Err(PacketError::BufferFull);
        }
        let end = self.len + bytes.len();
        self.data[self.len..end].copy_from_slice(bytes);
        self.len = end;
        Ok(self)
    }

    pub fn push<T: ToLeBytes>(&mut self, value: T) -> Result<&mut Self, PacketError> {
        value.write_to(self)?;
        Ok(self)
    }
}

impl<const N: usize> Default for Packet<N> {
    fn default() -> Self {
        Self::new()
    }
}

pub trait ToLeBytes {
    fn write_to<const N: usize>(self, packet: &mut Packet<N>) -> Result<(), PacketError>;
}

macro_rules! impl_to_le_bytes {
    ($($ty:ty),+ $(,)?) => {$(
        impl ToLeBytes for $ty {
            fn write_to<const N: usize>(self, packet: &mut Packet<N>) -> Result<(), PacketError> {
                packet.push_bytes(&self.to_le_bytes())?;
                Ok(())
            }
        }
    )+};
}

impl_to_le_bytes!(u8, i8, u16, i16, u32, i32, u64, i64, f32, f64);

impl ToLeBytes for bool {
    fn write_to<const N: usize>(self, packet: &mut Packet<N>) -> Result<(), PacketError> {
        packet.push_bytes(&[u8::from(self)])?;
        Ok(())
    }
}
