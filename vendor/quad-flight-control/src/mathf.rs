//! `no_std` float helpers.
//!
//! `f32::clamp` is in `core`, but `abs`, `signum`, `powf`, `exp`,
//! `floor`, and `f32::min`/`max` are `std`-only. Library code uses
//! these wrappers (backed by `libm`) instead so the crate builds for
//! the firmware target without `std`.

#[inline(always)]
pub fn abs(x: f32) -> f32 {
    libm::fabsf(x)
}

/// Sign of `x` as `-1.0` / `0.0` / `+1.0`. Differs from `f32::signum`
/// only at `±0.0` (returns `0.0`); call sites multiply this by a
/// magnitude that is zero whenever `x` is zero, so the difference is
/// inert.
#[inline(always)]
pub fn signum(x: f32) -> f32 {
    if x > 0.0 {
        1.0
    } else if x < 0.0 {
        -1.0
    } else {
        0.0
    }
}

#[inline(always)]
pub fn powf(x: f32, y: f32) -> f32 {
    libm::powf(x, y)
}

#[inline(always)]
pub fn exp(x: f32) -> f32 {
    libm::expf(x)
}

#[inline(always)]
pub fn floor(x: f32) -> f32 {
    libm::floorf(x)
}

#[inline(always)]
pub fn max(a: f32, b: f32) -> f32 {
    libm::fmaxf(a, b)
}
