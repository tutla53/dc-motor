// src/macros.rs

use super::*;

pub trait MutexExt<T> {
    fn with<F, R>(&self, f: F) -> Result<R, Box<dyn std::error::Error>>
    where
        F: FnOnce(&mut MutexGuard<'_, T>) -> R;
}

impl<T> MutexExt<T> for Mutex<T> {
    fn with<F, R>(&self, f: F) -> Result<R, Box<dyn std::error::Error>>
    where
        F: FnOnce(&mut MutexGuard<'_, T>) -> R,
    {
        let mut guard = self.lock().map_err(|_| "Mutex poisoned")?;
        Ok(f(&mut guard))
    }
}

#[macro_export]
macro_rules! try_lock {
    ($target:expr => $method:ident ( $($args:tt)* )) => {{
        (*$target).with(|guard| guard.$method($($args)*))
    }};
}
