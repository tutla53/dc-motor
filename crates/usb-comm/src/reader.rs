/*
* Little-endian Byte Reader
*/

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReadError {
    UnexpectedEnd,
    InvalidValue,
    NonFiniteFloat,
}

pub struct ByteReader<'a> {
    data: &'a [u8],
    cursor: usize,
}

impl<'a> ByteReader<'a> {
    pub const fn new(data: &'a [u8]) -> Self {
        Self { data, cursor: 0 }
    }
    pub fn position(&self) -> usize {
        self.cursor
    }
    pub fn remaining(&self) -> usize {
        self.data.len() - self.cursor
    }
    pub fn is_finished(&self) -> bool {
        self.remaining() == 0
    }

    pub fn read_bytes(&mut self, len: usize) -> Result<&'a [u8], ReadError> {
        if len > self.remaining() {
            return Err(ReadError::UnexpectedEnd);
        }
        let end = self.cursor + len;
        let bytes = &self.data[self.cursor..end];
        self.cursor = end;
        Ok(bytes)
    }

    pub fn read<T: FromLeBytes>(&mut self) -> Result<T, ReadError> {
        if T::SIZE > self.remaining() {
            return Err(ReadError::UnexpectedEnd);
        }
        let end = self.cursor + T::SIZE;
        let value = T::from_le_bytes(&self.data[self.cursor..end])?;
        self.cursor = end;
        Ok(value)
    }

    /// Read a finite f32; failed validation leaves the cursor unchanged.
    pub fn read_f32(&mut self) -> Result<f32, ReadError> {
        let start = self.cursor;
        let value = self.read::<f32>()?;
        if !value.is_finite() {
            self.cursor = start;
            return Err(ReadError::NonFiniteFloat);
        }
        Ok(value)
    }
}

pub trait FromLeBytes: Sized {
    const SIZE: usize;
    fn from_le_bytes(bytes: &[u8]) -> Result<Self, ReadError>;
}

macro_rules! impl_from_le_bytes {
    ($($ty:ty),+ $(,)?) => {$(
        impl FromLeBytes for $ty {
            const SIZE: usize = core::mem::size_of::<Self>();
            fn from_le_bytes(bytes: &[u8]) -> Result<Self, ReadError> {
                let array = bytes.try_into().map_err(|_| ReadError::UnexpectedEnd)?;
                Ok(<$ty>::from_le_bytes(array))
            }
        }
    )+};
}

impl_from_le_bytes!(u8, i8, u16, i16, u32, i32, u64, i64, f32, f64);

impl FromLeBytes for bool {
    const SIZE: usize = 1;
    fn from_le_bytes(bytes: &[u8]) -> Result<Self, ReadError> {
        match bytes {
            [0] => Ok(false),
            [1] => Ok(true),
            [_] => Err(ReadError::InvalidValue),
            _ => Err(ReadError::UnexpectedEnd),
        }
    }
}
