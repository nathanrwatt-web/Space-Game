use bevy::prelude::*;
use crate::sim::clock::SimClock;
use crate::sim::orbit::{Burn, Maneuvers, Orbit};

// marker for a ship that should enter a circular orbit once
// in a body's sphere of influence 
#[derive(Component)]
pub struct CaptureIntent {
    pub target: Entity,
}

pub fn auto_capture( 
    clock: Res<SimClock>,
    mut commands: Commands, 
    mut ships: Query<(Entity, &Orbit, &mut Maneuvers, &CaptureIntent)>,
) {
    for (entity, orbit, mut maneuver, intent) in &mut ships {
        // capture once we are orbiting the target 

        if orbit.parent != intent.target || orbit.elements.e < 1.0 {
            continue;
        }

        let el = orbit.elements;
        let t_peri = el.time_of_periapsis();   // single periapsis of the hyperbola
        let r_p = el.a * (1.0 - el.e);         // a<0, (1-e)<0  ->  r_p > 0
        let v_hyp = el.velocity_at(t_peri);    // planet-relative, global axes
        let v_circ = (el.mu / r_p).sqrt();

        // pure retrograde: scale velocity down from v_hyp to v_circ at periapsis
        let dv = -v_hyp.normalize() * (v_hyp.length() - v_circ);

        maneuver.queue.push_back(Burn {
            execute_at: t_peri.max(clock.t), // if we entered past periapsis, fire now
            dv,
        });

        commands.entity(entity).remove::<CaptureIntent>();
    }
}


#[cfg(test)]
mod tests {
    use crate::math::orbital_elements::OrbitalElements;
    use bevy::math::DVec3;

    #[test]
    fn capture_burn_circularizes_a_hyperbola() {
        let mu = 1.0e12;
        // an inbound hyperbola: position + super-escape velocity
        let r = DVec3::new(1.0e6, 0.0, 0.0);
        let v = DVec3::new(0.5e3, 3.0e3, 0.0); // |v| > escape -> e > 1
        let hyp = OrbitalElements::from_state(r, v, mu, 0.0);
        assert!(hyp.e > 1.0, "not hyperbolic: e = {}", hyp.e);

        // replicate exactly what auto_capture computes
        let t_peri = hyp.time_of_periapsis();
        let r_p = hyp.a * (1.0 - hyp.e);
        let v_hyp = hyp.velocity_at(t_peri);
        let v_circ = (mu / r_p).sqrt();
        let dv = -v_hyp.normalize() * (v_hyp.length() - v_circ);

        let captured = hyp.with_burn(t_peri, dv);
        assert!(captured.e < 1e-6, "e after capture = {}", captured.e);
        assert!((captured.a - r_p).abs() / r_p < 1e-6, "a = {} vs r_p = {}", captured.a, r_p);
    }
}
