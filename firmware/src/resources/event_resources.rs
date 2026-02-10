/*
    Event Resources
*/

/* --------------------------- Event List -------------------------- */
#[derive(Clone, Copy)]
pub enum EventList {
    MotorMoveDone(u8),
}