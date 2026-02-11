#[macro_export]
macro_rules! create_motors {
    ($($id:expr),*) => {
        [
            $(MotorHandler::new($id)),*
        ]
    };
}