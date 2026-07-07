use bevy::prelude::*;
use bevy::math::DVec3;
use serde::{Deserialize, Serialize};

use crate::sim::{
    orbit::Orbit,
    integrate::StateVec,
};

pub(crate) const CTRL_W: f64 = 0.1;

// TODO: Allow for greater escape velocity but add perpendicular acceleration from this budget 
// or else add angular velocity to stop escape. 

// Cruise scales with local circular speed and is kept
// BELOW escape velocity (√2 · v_circ ≈ 1.414 · v_circ) so the craft stays gravitationally
// bound even if thrust is briefly starved — otherwise it slingshots away.
const MOVE_CRUISE_FACTOR: f64 = 1.2;
const ARRIVE_EPS: f64 = 1.0;

#[derive(Component, Clone, Copy, Debug, Default, Serialize, Deserialize)]
pub enum Guidance{
    #[default]
    Idle, 
    Hold,
    StationKeep { radius: f64 },
    Seek { target: Entity },
    MoveTo { target: DVec3 }, // RTS move order: glide to a point on the shell (frame-local)
}

// hold a fixed distance from the fame body
pub(crate) fn station_keep(pos: DVec3, vel: DVec3, mu: f64, radius: f64) -> DVec3 {
    let r = pos.length();
    if r < 1.0 { return DVec3::ZERO; }
    let r_hat = pos / r; // direction of position 
    let v_r = vel.dot(r_hat); // directional derivative
    // length^2 of the component of velocity not in the direction of pos
    let v_tan2 = (vel - v_r * r_hat).length_squared(); 
    //r¨ = v_tan²/r - μ/r² + a_thrust
    //since r¨ = 0, we have
    let a_thrust = mu / (r * r) - v_tan2 / r;
    // tuner, radius - r is the error
    let a_pd = CTRL_W * CTRL_W * (radius - r) - 2.0 * CTRL_W * v_r;
    (a_thrust + a_pd) * r_hat
}

// Hover at current pos, possible if max_accel exceeds gravity
pub(crate) fn hold(pos: DVec3, vel: DVec3, mu: f64) -> DVec3 {
    let r = pos.length();
    if r < 1.0 { return DVec3::ZERO; }
    // gravity is -μ/r² r_hat, so canvel this out 
    (mu / (r * r)) * (pos / r) - (2.0 * CTRL_W) * vel
}

// pursuit towards a target position 
#[allow(dead_code)]
pub(crate) fn seek(pos: DVec3, vel: DVec3, target_pos: DVec3, target_vel: DVec3) -> DVec3 {
    let rel = target_pos - pos; 
    let rel_v = target_vel - vel;
    // ω² rel + 2ω rel_v
    CTRL_W * CTRL_W * rel + 2.0 * CTRL_W * rel_v
}

// Glide to a point on the orbital shell at constant altitude (RTS move order). A radial
// term holds the shell radius (|target|) while a tangential arrival term steers along the
// sphere surface toward the target and decelerates to a stop; on arrival it damps to a
// hover. Holding the radius keeps motion on the shell — a straight chord would dive toward
// the planet mid-route. `max_accel` sizes the kinematic braking distance.
pub(crate) fn move_to(pos: DVec3, vel: DVec3, mu: f64, target: DVec3, max_accel: f64) -> DVec3 {
    let r = pos.length();
    if r < 1.0 { return DVec3::ZERO; }
    let r_hat = pos / r;
    let shell = target.length();
    let v_r = vel.dot(r_hat);
    let v_tan = vel - v_r * r_hat;
    let a_ff = mu / (r * r) - v_tan.length_squared() / r; // gravity vs centrifugal feed-forward
    let a_budget = 0.5 * max_accel.max(0.0);
    let v_cruise = MOVE_CRUISE_FACTOR * (mu / shell).sqrt(); // sub-escape

    // radial: VELOCITY-LIMITED approach to the shell. A raw PD on a large altitude error
    // would saturate thrust and build huge radial speed, overshoot the shell, and slingshot
    // away — the arrival profile caps the approach speed (and brakes) instead.
    let r_err = shell - r;
    let desired_v_r = r_err.signum() * (2.0 * a_budget * r_err.abs()).sqrt().min(v_cruise);
    let a_radial = a_ff + (desired_v_r - v_r) * 2.0 * CTRL_W;

    // tangential: velocity-limited arrival along the surface toward the target's projection
    let to_target = target - pos;
    let tan = to_target - to_target.dot(r_hat) * r_hat; // surface direction toward target
    let arc = tan.length();
    let a_tan = if arc < ARRIVE_EPS {
        -2.0 * CTRL_W * v_tan // arrived: damp tangential drift
    } else {
        let dir = tan / arc;
        let desired = dir * (2.0 * a_budget * arc).sqrt().min(v_cruise);
        (desired - v_tan) * (2.0 * CTRL_W)
    };

    // radial-priority clamp: spend the budget on holding altitude first and cruise with the
    // rest, so the centripetal / altitude-hold thrust is never starved by a saturating cruise.
    let rt = a_radial.clamp(-max_accel, max_accel);
    let rem = (max_accel * max_accel - rt * rt).max(0.0).sqrt();
    let a_tan = {
        let m = a_tan.length();
        if m > rem && m > 0.0 { a_tan * (rem / m) } else { a_tan }
    };

    rt * r_hat + a_tan
}

// targets (pos, vel) in frames local coords 
#[allow(dead_code)]
pub(crate) fn target_local_state(
    target: Entity, frame: Entity, t: f64, 
    statevecs: &Query<&StateVec>, orbits: &Query<&Orbit>,
) -> Option<(DVec3, DVec3)> {
    if let Ok(tsv) = statevecs.get(target) && tsv.frame == frame {
        return Some((tsv.pos, tsv.vel));
    }
    if let Ok(to) = orbits.get(target) && to.parent == frame {
        return Some(to.elements.state_vectors_at(t));
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sim::integrate::{verlet_step, GravitySource};

    const MU: f64 = 7.0e12;
    const RAD: f64 = 2.0e6;
    const MAXA: f64 = 200.0;

    fn clamp(a: DVec3, lim: f64) -> DVec3 {
        let m = a.length();
        if m > lim { a * (lim / m) } else { a }
    }
    fn src() -> [GravitySource; 1] { [GravitySource { mu: MU, pos: DVec3::ZERO }] }

    #[test]
    fn station_keep_holds_radius() {
        let vc = (MU / RAD).sqrt();
        let (mut p, mut v) = (DVec3::new(0.92 * RAD, 0.0, 0.0), DVec3::new(0.0, vc * 0.97, 0.0));
        for _ in 0..40_000 {
            let th = clamp(station_keep(p, v, MU, RAD), MAXA);
            verlet_step(&mut p, &mut v, &src(), th, 0.05);
        }
        let err = (p.length() - RAD).abs() / RAD;
        assert!(err < 1e-3, "radius rel err {err:.2e}");
    }

    #[test]
    fn hold_brings_to_rest() {
        let (mut p, mut v) = (DVec3::new(RAD, 0.0, 0.0), DVec3::new(200.0, -150.0, 80.0));
        for _ in 0..20_000 {
            let th = clamp(hold(p, v, MU), MAXA);
            verlet_step(&mut p, &mut v, &src(), th, 0.05);
        }
        assert!(v.length() < 1.0, "final speed {:.3e}", v.length());
    }

    #[test]
    fn seek_closes_on_coorbiting_target() {
        let vc = (MU / RAD).sqrt();
        let (mut p, mut v) = (DVec3::new(RAD, 0.0, 0.0), DVec3::new(0.0, vc, 0.0));
        let tvc = (MU / (1.1 * RAD)).sqrt();
        let (mut tp, mut tv) = (DVec3::new(0.0, 1.1 * RAD, 0.0), DVec3::new(-tvc, 0.0, 0.0));
        let d0 = (tp - p).length();
        let mut dmin = d0;
        for _ in 0..80_000 {
            let th = clamp(seek(p, v, tp, tv), MAXA);
            verlet_step(&mut p, &mut v, &src(), th, 0.05);
            verlet_step(&mut tp, &mut tv, &src(), DVec3::ZERO, 0.05);
            dmin = dmin.min((tp - p).length());
        }
        assert!(dmin < 0.1 * d0, "closest {dmin:.2e} vs start {d0:.2e}");
    }

    #[test]
    fn move_to_glides_to_point_and_holds_shell() {
        let target = DVec3::new(0.0, RAD, 0.0); // quarter-way around the same shell
        let (mut p, mut v) = (DVec3::new(RAD, 0.0, 0.0), DVec3::ZERO);
        let mut r_min = RAD;
        for _ in 0..400_000 {
            let th = clamp(move_to(p, v, MU, target, MAXA), MAXA);
            verlet_step(&mut p, &mut v, &src(), th, 0.05);
            r_min = r_min.min(p.length());
        }
        assert!((p - target).length() < 0.02 * RAD, "didn't arrive: {p:?}");
        assert!(v.length() < 50.0, "didn't stop, v = {}", v.length());
        assert!(r_min > 0.9 * RAD, "left the shell (dipped to {:.3}R)", r_min / RAD);
    }
}
