// This is the math used for solving the lamber problem
// Times and velocities for travelling between bodies
// is calculated here
//
// === See the math at the bottom of the file !===

use bevy::math::DVec3;
use std::f64::consts::TAU;

// Stumpff functions C(z), S(z)
fn stumpff_c(z: f64) -> f64 {
    if z > 0.0 {
        (1.0 - z.sqrt().cos()) / z
    } else if z < 0.0 {
        ((-z).sqrt().cosh() - 1.0) / (-z)
    } else {
        0.5
    }
}

fn stumpff_s(z: f64) -> f64 {
    if z > 0.0 {
        let s = z.sqrt();
        (s - s.sin()) / s.powi(3)
    } else if z < 0.0 {
        let s = (-z).sqrt();
        (s.sinh() - s) / s.powi(3)
    } else {
        1.0 / 6.0
    }
}

// Given the two position vectors and a time of flight, returns the velocity
// vectors (v1, v2) at r1 and r2, or None if it can't converge / is degenerate.
pub(crate) fn lambert(
    r1: DVec3,
    r2: DVec3,
    tof: f64,
    mu: f64,
    prograde: bool,
) -> Option<(DVec3, DVec3)> {
    let r1n = r1.length();
    let r2n = r2.length();

    let cross = r1.cross(r2);
    let cos_dnu = (r1.dot(r2) / (r1n * r2n)).clamp(-1.0, 1.0);

    // transfer angle Δν, picking the branch from the desired direction
    // acos -> [0, pi] so must account for other theta
    let dnu = if prograde == (cross.z >= 0.0) {
        cos_dnu.acos()
    } else {
        TAU - cos_dnu.acos()
    };

    // g = r₁ r₂ sinΔν / √(μp)
    let a = dnu.sin() * (r1n * r2n / (1.0 - dnu.cos())).sqrt();
    if a.abs() < 1e-12 {
        return None; // ~180° / colinear: transfer plane undefined
    }

    let y = |z: f64| r1n + r2n + a * (z * stumpff_s(z) - 1.0) / stumpff_c(z).sqrt();
    let t_of_z = |z: f64| {
        let yz = y(z);
        let chi = (yz / stumpff_c(z)).sqrt();
        (chi.powi(3) * stumpff_s(z) + a * yz.sqrt()) / mu.sqrt()
    };

    // start at z = 0, raise until y(z) >= 0 (so χ is real)
    let mut z = 0.0;
    let mut guard = 0;
    while y(z) < 0.0 {
        z += 0.1;
        guard += 1;
        if guard > 10_000 {
            return None;
        }
    }

    // Newton with a numerical derivative (avoids the fragile closed-form dt/dz)
    for _ in 0..100 {
        let t = t_of_z(z);
        let h = 1e-6 * (1.0 + z.abs());
        let dtdz = (t_of_z(z + h) - t_of_z(z - h)) / (2.0 * h);
        if !t.is_finite() || !dtdz.is_finite() || dtdz.abs() < 1e-30 {
            break;
        }
        let dz = (t - tof) / dtdz;
        z -= dz;
        if y(z) < 0.0 {
            z += dz * 0.5; // damped: stepped into the invalid region
        }
        if dz.abs() < 1e-9 {
            break;
        }
    }

    let yz = y(z);
    if !z.is_finite() || yz < 0.0 {
        return None;
    }

    let f = 1.0 - yz / r1n;
    let g = a * (yz / mu).sqrt();
    let gdot = 1.0 - yz / r2n;
    if g.abs() < 1e-30 {
        return None;
    }

    let v1 = (r2 - f * r1) / g;
    let v2 = (gdot * r2 - r1) / g;
    Some((v1, v2))
}

// ===== MATH =====
/*
 *  The universal anomlay χ is defined by dχ/dt = √μ / r
 *      elipsis: χ = √a · ΔE (change in Eccentric Anomoly
 *      hyperbola: χ = √(−a) · ΔF (change in hyperbolic Anomoly)
 *
 *  Let α = 1/a and define  z = α·χ²
 *  Then the sign of z encodes the conic type,
 *      a > 0 => z > 0
 *      a < 0 => z < 0
 *      a = infinity => z = 0
 *
 *  Stumpff Functions:
 *      C(z) = Σ_{k≥0} (−z)^k/(2k+2)!  = 1/2 − z/24 + z²/720 − …
 *      S(z) = Σ_{k≥0} (−z)^k/(2k+3)!  = 1/6 − z/120 + z²/5040 − …
 *  Closed form:
 *      z>0:  C=(1−cos√z)/z          S=(√z − sin√z)/(√z)³
 *      z<0:  C=(cosh√−z − 1)/(−z)   S=(sinh√−z − √−z)/(√−z)³
 *      z=0:  C=1/2                  S=1/6
 *
 *  For motion over a universal step χ, our state propogates r₂ = f·r₁ + g·v₁ with:
 * f  = 1 − (χ²/r₁)·C(z)
 * g  = Δt − (χ³/√μ)·S(z)
 * ġ  = 1 − (χ²/r₂)·C(z)
 * ḟ  = (√μ/(r₁r₂))·χ·(z·S(z) − 1)
 * ( wronskian law applies, ie: f·ġ − ḟ·g = 1 )
 *
 * The transfer angle is given by:
 * cos Δν = (r₁·r₂)/(r₁ r₂)
 *
 * The seed for our iteration will be derived from:
 * r₁ × r₂ = r₁ × (f·r₁ + g·v₁) = g·(r₁ × v₁) = g·h
 * Since f·r₁ is dependent with r₁, it vanishes. |r₁×r₂| = r₁r₂sinΔν and
 * |h| = √(μp), so g = r₁ r₂ sinΔν / √(μp)
 *
 * The geometry collapses to A = sinΔν · √( r₁ r₂ / (1 − cosΔν) )
 * Appluing half angle identity:
 *
 * A = √(2 r₁ r₂) · cos(Δν/2). This is the seed used in the lambert
 *
 *
 * Next define y = χ²C(z) and
 * y(z) = r₁ + r₂ + A·(z·S(z) − 1)/√C(z) implies χ = √( y / C(z) )
 *
 * Central lamber equation:
 * √μ · Δt = χ³·S(z) + A·√y = (y/C)^{3/2}·S(z) + A·√y and
 * Δt(z) = [ (y/C)^{3/2}·S(z) + A·√y ] / √μ
 *
 * f = 1 − (χ²/r₁)·C = 1 − y/r₁
 * ġ = 1 − (χ²/r₂)·C = 1 − y/r₂
 * g = A·√(y/μ)
 *
 * Then r₂ = f·r₁ + g·v₁ implies:
 * v₁ = (r₂ − f·r₁)
 *
 * The wronskian implies:
 * v₂ = (ġ·r₂ − r₁)/g
 *
*/

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::orbital_elements::OrbitalElements;

    // Curtis, Orbital Mechanics for Engineering Students, Example 5.2
    #[test]
    fn lambert_matches_curtis_example_5_2() {
        let r1 = DVec3::new(5000.0, 10000.0, 2100.0);
        let r2 = DVec3::new(-14600.0, 2500.0, 7000.0);
        let (v1, v2) = lambert(r1, r2, 3600.0, 398_600.0, true).expect("converged");

        let v1_exp = DVec3::new(-5.9925, 1.9254, 3.2456);
        let v2_exp = DVec3::new(-3.3125, -4.1966, -0.38529);
        assert!((v1 - v1_exp).length() < 1e-3, "v1 = {v1:?}");
        assert!((v2 - v2_exp).length() < 1e-3, "v2 = {v2:?}");
    }

    // Propagate a known orbit forward, then Lambert must recover the departure velocity.
    #[test]
    fn lambert_round_trips_through_propagation() {
        let mu = 398_600.0;
        let r1 = DVec3::new(7000.0, 0.0, 1000.0);
        let v1_true = DVec3::new(0.0, 7.5, 0.5);
        let el = OrbitalElements::from_state(r1, v1_true, mu, 0.0);
        let tof = 2000.0;
        let r2 = el.offset_at(tof);

        let (v1, _v2) = lambert(r1, r2, tof, mu, true).expect("converged");
        assert!((v1 - v1_true).length() < 1e-6, "v1 = {v1:?} vs {v1_true:?}");
    }
}
