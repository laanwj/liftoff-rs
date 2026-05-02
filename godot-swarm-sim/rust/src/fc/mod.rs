//! Flight controller subsystems. The per-tick path is:
//!
//! ```text
//! sticks → mode.rate_setpoint() → rate-PIDs → mixer → motor commands
//! ```

pub mod mixer;
pub mod mode;
pub mod pid;
pub mod rates;
