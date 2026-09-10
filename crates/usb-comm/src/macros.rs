/*
* Protocol Macros
*/

/// Declare an opcode enum and its fallible conversion from a wire byte.
///
/// The caller supplies attributes such as `#[repr(u8)]` and derives.
/// Reserved opcode policy remains in the application's `CommandOpcode` impl.
#[macro_export]
macro_rules! create_opcode_enum {
    ($(#[$meta:meta])* $vis:vis enum $name:ident {
        $($variant:ident = $val:expr),* $(,)?
    }) => {
        $(#[$meta])*
        $vis enum $name {
            $($variant = $val,)*
        }

        impl ::core::convert::TryFrom<u8> for $name {
            type Error = ();

            fn try_from(value: u8) -> ::core::result::Result<Self, Self::Error> {
                match value {
                    $($val => ::core::result::Result::Ok(Self::$variant),)*
                    _ => ::core::result::Result::Err(()),
                }
            }
        }
    };
}
