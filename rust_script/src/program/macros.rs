// src/macros.rs

pub trait IntoResult {
    type OkType;
    type ErrType;
    fn into_result(self) -> Result<Self::OkType, Self::ErrType>;
}

impl<T, E> IntoResult for Result<T, E> {
    type OkType = T;
    type ErrType = E;
    fn into_result(self) -> Result<T, E> { self }
}

impl IntoResult for () {
    type OkType = ();
    type ErrType = &'static str;
    fn into_result(self) -> Result<(), &'static str> { Ok(()) }
}

#[macro_export]
macro_rules! with_lock {
    ($resources:expr, $field:ident . $method:ident ( $($args:expr),* $(,)? )) => {
        match $resources.$field.lock() {
            #[allow(unused_mut)]
            Ok(mut target) => {
                use $crate::program::macros::IntoResult;
                match target.$method($($args),*).into_result() {
                    Ok(val) => Ok(val),
                    Err(e) => Err(Box::<dyn std::error::Error>::from(e)),
                }
            },
            Err(_) => Err(Box::<dyn std::error::Error>::from("Mutex poisoned")),
        }
    };
}