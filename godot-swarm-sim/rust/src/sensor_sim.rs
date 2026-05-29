//! Sensor simulation — turns physics-engine truth into simulated
//! sensor readings of the same shape the firmware would get from real
//! hardware. The FC core consumes only this layer's output, never
//! `BodyTruth` directly, so the SITL boundary is hardware-identical.
//!
//! M1 covers gyro. M2 adds an accelerometer for the Mahony estimator.
//! Future extensions live here without touching the FC core: bias,
//! white noise, quantisation, ODR, axis misalignment.
//!
//! Stateless and deterministic for now — adding bias/noise later means
//! making this struct-shaped (per-drone state, RNG seed).
//!
//! The IMU on the target hardware is an ICM-42688-P; matching its
//! noise/quantisation characteristics is what gives genuine
//! firmware-vs-sim parity.

/// Simulate the gyro that the FC core sees this tick.
///
/// `truth_gyro` is the rigid body's true angular velocity, rad/s, in
/// the FC's body frame (FRD). The M1 implementation is an identity
/// pass-through — the deterministic baseline that lets the rest of
/// the SITL be reasoned about before noise/bias enter the picture.
#[inline]
pub fn simulate_gyro(truth_gyro: [f32; 3]) -> [f32; 3] {
    truth_gyro
}
