use bevy::prelude::*;
use bevy::math::DVec3;

use crate::sim::clock::SimClock;
use crate::sim::orbit::Body;

// Fixed physics step, in *sim seconds*.
pub(crate) const PHYS_DT: f64 = 1.0 / 30.0;
const MAX_SUBSTEPS: u32 = 64;

#[derive(Component, Clone, Copy, Debug)]
pub struct StateVec {
    pub pos: DVec3,
    pub vel: DVec3,     
    pub frame: Entity,  // frame of reference body 
}

#[derive(Component, Clone, Copy, Debug, Default)]
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

// rseolve guidance vs crafts thrust limit 
fn commanded_thrust(cmd: Option<&ThrustCommand>, prop: &Propulsion) -> DVec3 {
    let want = cmd.map_or(DVec3::ZERO, |c| c.accel);
    let limit = prop.max_accel * prop.throttle.clamp(0.0, 1.0);
    let mag = want.length();
    // if thrust needed more than limit then return want with maxed according to limit 
    if mag > limit && mag > 0.0 {
        want * (limit / mag)
    } else { 
        want 
    }
}

#[derive(Resource, Default)]
pub struct PhysAccumulator {
    last_t: f64, 
    accum: f64, 
    initialized: bool,
}

// integrate currently powered ships 
pub fn integrate_powered(
    clock: Res<SimClock>,
    mut acc: ResMut<PhysAccumulator>,
    bodies: Query<&Body>,
    mut ships: Query<(&mut StateVec, &Propulsion, Option<&ThrustCommand>)>,
) {
    // on first tick 
    if !acc.initialized {
        acc.last_t = clock.t;
        acc.initialized = true;
        return;
    }

    let dt_total = clock.t - acc.last_t;
    acc.last_t = clock.t; // update last time 
    if dt_total <= 0.0 { return; } // exit early if paused 
    acc.accum += dt_total; 

    // compute number of steps in between a whole step and max substeps 
    let mut steps = 0;
    while acc.accum >= PHYS_DT && steps < MAX_SUBSTEPS {
        for (mut sv, prop, cmd) in &mut ships {
            let Ok(body) = bodies.get(sv.frame) else { continue };
            let sources = [GravitySource {
                mu: body.mu,
                pos: DVec3::ZERO // sits at the center of the local frame 
            }];
            let thrust = commanded_thrust(cmd, prop); // thrust to be applied 
            let (mut pos, mut vel) = (sv.pos, sv.vel);

            verlet_step(&mut pos, &mut vel, &sources, thrust, PHYS_DT); // numerically integrate 
            sv.pos = pos;
            sv.vel = vel;
        }
        acc.accum -= PHYS_DT;
        steps += 1;
    }

    // reset accumulation 
    if steps == MAX_SUBSTEPS { acc.accum = 0.0; }
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
}

