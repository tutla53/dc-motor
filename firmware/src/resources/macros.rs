#[macro_export]
macro_rules! create_motors {
    ($($id:expr),*) => {
        [
            $(MotorHandler::new($id)),*
        ]
    };
}

#[macro_export]
macro_rules! create_opcode_enum {
    ($(#[$meta:meta])* $vis:vis enum $name:ident {
        $($variant:ident = $val:expr,)*
    }) => {
        $(#[$meta])*
        pub enum $name {
            $($variant = $val,)*
        }

        impl ::core::convert::TryFrom<u8> for $name {
            type Error = ();
            fn try_from(v: u8) -> ::core::result::Result<Self, Self::Error> {
                match v {
                    $($val => ::core::result::Result::Ok($name::$variant),)*
                    _ => ::core::result::Result::Err(()),
                }
            }
        }
    }
}