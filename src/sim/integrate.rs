use bevy::prelude::*;
use bevy::math::DVec3;
use serde::{Serialize, Deserialize};

use crate::sim::clock::SimClock;
use crate::sim::orbit::Body;
use crate::sim::guidance::{Guidance, hold, move_to, station_keep};

// Fixed physics step, in *sim seconds*.
pub(crate) const PHYS_DT: f64 = 1.0 / 30.0;
// Guidance is re-evaluated every substep, so substeps must stay small, but also bounded
pub(crate) const MAX_SUBSTEPS: u32 = 512;

#[derive(Component, Clone, Copy, Debug)]
pub struct StateVec {
    pub pos: DVec3,
    pub vel: DVec3,     
    pub frame: Entity,  // frame of reference body 
}

impl StateVec {
    pub fn from_orbit(orbit: &crate::sim::orbit::Orbit, t: f64) -> Self {
        let (pos, vel) = orbit.elements.state_vectors_at(t);
        Self { pos, vel, frame: orbit.parent }
    }
}

#[derive(Component, Clone, Copy, Debug, Default, Serialize, Deserialize)]
pub struct Propulsion {
    pub max_accel: f64,
    pub throttle:  f64,  // ie how much gas is on the pedal
}

#[derive(Component, Clone, Copy, Debug, Default)]
pub struct ThrustCommand {
    pub accel: DVec3,
}

// here for eventual N-body, integration will (hopefully) take multiple of these 
#[derive(Clone, Copy, Debug)]
pub struct GravitySource {
    pub mu: f64,
    pub pos: DVec3,
}

// a = Σ −μ_i (r − r_i) / |r − r_i|³
pub(crate) fn gravity(sources: &[GravitySource], pos: DVec3) -> DVec3 {
    let mut a = DVec3::ZERO;
    for s in sources {
        let d = pos - s.pos; // vector from us to gravity source
        let r2 = d.length_squared();
        if r2 > 0.0 {
            let inv_r3 = 1.0 / (r2 * r2.sqrt());
            a -= s.mu * d * inv_r3;
        }
    }
    a
}

// verlet integration 
/*
From wikipedia: 
        Vec3d new_pos = pos + vel*dt + acc*(dt*dt*0.5);
        Vec3d new_acc = apply_forces();
        Vec3d new_vel = vel + (acc+new_acc)*(dt*0.5);
        pos = new_pos;
        vel = new_vel;
        acc = new_acc;
*/
pub(crate) fn verlet_step( pos: &mut DVec3, vel: &mut DVec3,
    sources: &[GravitySource], thrust: DVec3, dt: f64 ) 
{
    let a0 = gravity(sources, *pos) + thrust;
    *pos += *vel * dt + 0.5 * a0 * dt * dt;
    let a1 = gravity(sources, *pos) + thrust;
    *vel += 0.5 * (a0 + a1) * dt;
}

// clamp a desired acceleration to the craft's thrust limit
fn clamp_accel(a: DVec3, limit: f64) -> DVec3 {
    let mag = a.length();
    if mag > limit && mag > 0.0 { a * (limit / mag) } else { a }
}

#[derive(Resource, Default)]
pub struct PhysAccumulator {
    last_t: f64,
    pending: f64,
    initialized: bool,
}

pub(crate) fn max_powered_frame_budget() -> f64 {
    PHYS_DT * MAX_SUBSTEPS as f64
}

// integrate currently powered ships 
pub fn integrate_powered(
    clock: Res<SimClock>,
    mut acc: ResMut<PhysAccumulator>,
    bodies: Query<&Body>,
    mut ships: Query<(&mut StateVec, &Propulsion, &Guidance, &mut ThrustCommand)>,
) {
    if ships.is_empty() {
        acc.last_t = clock.t;
        acc.pending = 0.0;
        acc.initialized = true;
        return;
    }

    if !acc.initialized {
        acc.last_t = clock.t;
        acc.pending = 0.0;
        acc.initialized = true;
        return;
    }

    let dt_total = clock.t - acc.last_t;
    acc.last_t = clock.t;
    if dt_total <= 0.0 {
        return;
    }

    acc.pending = (acc.pending + dt_total).min(max_powered_frame_budget());
    while acc.pending >= PHYS_DT {
        for (mut sv, prop, guidance, mut cmd) in &mut ships {
            let Ok(body) = bodies.get(sv.frame) else { continue };
            let mu = body.mu;
            let sources = [GravitySource {
                mu,
                pos: DVec3::ZERO // sits at the center of the local frame
            }];

            // Guidance is sampled once per fixed step so arrival behavior stays stable
            // even when frame-time or sim warp changes.
            let limit = prop.max_accel * prop.throttle.clamp(0.0, 1.0);
            let desired = match *guidance {
                Guidance::Idle => DVec3::ZERO,
                Guidance::Hold => hold(sv.pos, sv.vel, mu),
                Guidance::StationKeep { radius } => station_keep(sv.pos, sv.vel, mu, radius),
                Guidance::MoveTo { target } => move_to(sv.pos, sv.vel, mu, target, limit),
                Guidance::Seek { .. } => cmd.accel,
            };
            let thrust = clamp_accel(desired, limit);
            cmd.accel = thrust;

            let (mut pos, mut vel) = (sv.pos, sv.vel);
            verlet_step(&mut pos, &mut vel, &sources, thrust, PHYS_DT);
            sv.pos = pos;
            sv.vel = vel;
        }
        acc.pending -= PHYS_DT;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::orbital_elements::OrbitalElements;

    const MU: f64 = 1.267e17; // ~Jupiter G·M (m³/s²)

    fn one_source(mu: f64) -> [GravitySource; 1] {
        [GravitySource { mu, pos: DVec3::ZERO }]
    }

    #[test]
    fn gravity_points_inward_with_inverse_square_magnitude() {
        let r = 1.0e7;
        let a = gravity(&one_source(MU), DVec3::new(r, 0.0, 0.0));
        assert!(a.x < 0.0 && a.y.abs() < 1e-6 && a.z.abs() < 1e-6);
        assert!((a.length() - MU / (r * r)).abs() / (MU / (r * r)) < 1e-12);
    }

    #[test]
    fn gravity_sums_over_sources() {
        let s = [
            GravitySource { mu: MU, pos: DVec3::new(-1.0e7, 1.0e7, 0.0) },
            GravitySource { mu: MU, pos: DVec3::new( 1.0e7, 1.0e7, 0.0) },
        ];
        let a = gravity(&s, DVec3::ZERO);
        assert!(a.x.abs() < 1e-3, "x should cancel, got {}", a.x);
        assert!(a.y > 0.0, "net pull toward the masses (+y)");
    }

    #[test]
    fn verlet_is_exact_for_constant_acceleration() {
        let accel = DVec3::new(0.3, -0.2, 0.05);
        let (mut pos, mut vel) = (DVec3::ZERO, DVec3::ZERO);
        let (dt, n) = (0.5, 400);
        for _ in 0..n {
            verlet_step(&mut pos, &mut vel, &[], accel, dt);
        }
        let t = n as f64 * dt;
        assert!((pos - 0.5 * accel * t * t).length() < 1e-6);
        assert!((vel - accel * t).length() < 1e-9);
    }

    #[test]
    fn integrator_tracks_analytic_circular_orbit() {
        let r = 1.0e7;
        let v = (MU / r).sqrt();
        let pos0 = DVec3::new(r, 0.0, 0.0);
        let vel0 = DVec3::new(0.0, v, 0.0);
        let el = OrbitalElements::from_state(pos0, vel0, MU, 0.0);

        let sources = one_source(MU);
        let (dt, steps_per_check) = (0.25, 400); // exact integer t (=100 s) per check
        let (mut pos, mut vel) = (pos0, vel0);
        for check in 1..=5 {
            for _ in 0..steps_per_check {
                verlet_step(&mut pos, &mut vel, &sources, DVec3::ZERO, dt);
            }
            let t = (check * steps_per_check) as f64 * dt;
            let (ar, _) = el.state_vectors_at(t);
            let err = (pos - ar).length();
            assert!(err / r < 1e-3, "{:.2e} rel drift at t={t}", err / r);
        }
    }

    #[test]
    fn coasting_orbit_conserves_energy() {
        let r = 1.0e7;
        let v = (MU / r).sqrt() * 1.1; // slightly elliptical
        let pos0 = DVec3::new(r, 0.0, 0.0);
        let vel0 = DVec3::new(0.0, v, 0.0);
        let energy = |p: DVec3, vv: DVec3| vv.length_squared() / 2.0 - MU / p.length();
        let e0 = energy(pos0, vel0);

        let sources = one_source(MU);
        let (mut pos, mut vel) = (pos0, vel0);
        for _ in 0..20_000 {
            verlet_step(&mut pos, &mut vel, &sources, DVec3::ZERO, 0.25);
        }
        let drift = (energy(pos, vel) - e0).abs() / e0.abs();
        assert!(drift < 1e-4, "energy drifted {drift:.2e}");
    }

    #[test]
    fn coast_powered_coast_round_trip() {
        // eccentric, inclined orbit (unambiguous e > 0)
        let r0 = DVec3::new(1.0e7, 0.0, 0.0);
        let vc = (MU / 1.0e7).sqrt();
        let v0 = DVec3::new(0.0, vc * 0.9, vc * 0.2);
        let el0 = OrbitalElements::from_state(r0, v0, MU, 0.0);

        // Orbit -> StateVec at t0, integrate forward with zero thrust to t1
        let (mut pos, mut vel) = el0.state_vectors_at(0.0);
        let sources = one_source(MU);
        let (dt, n) = (0.25, 2000);
        for _ in 0..n {
            verlet_step(&mut pos, &mut vel, &sources, DVec3::ZERO, dt);
        }
        let t1 = n as f64 * dt;

        // StateVec -> Orbit at t1; the rebuilt conic must agree with the original later
        let el1 = OrbitalElements::from_state(pos, vel, MU, t1);
        let t2 = t1 + 5000.0;
        let p_ref = el0.state_vectors_at(t2).0;
        let p_new = el1.state_vectors_at(t2).0;
        assert!(
            (p_ref - p_new).length() / r0.length() < 1e-3,
            "round-trip drift {:.2e}",
            (p_ref - p_new).length() / r0.length()
        );
    }
}
