// Debug-only sim input helpers: keyboard shortcuts for queuing test burns, toggling powered
// flight, and switching guidance laws on the selected ship. All gated to AppMode::Run by the
// DebugPlugin registration.

use bevy::prelude::*;

use crate::sim::clock::SimClock;
use crate::sim::orbit::{Body, Burn, Maneuvers, Orbit, OrbitPropagationCache};
use crate::sim::integrate::{Propulsion, StateVec, ThrustCommand};
use crate::sim::guidance::Guidance;
use super::DebugUi;

// B: queue a small prograde test burn on every ship at the current time.
pub fn debug_burn_key(
    keys: Res<ButtonInput<KeyCode>>,
    clock: Res<SimClock>,
    mut ships: Query<(&Orbit, &mut Maneuvers)>,
) {
    if !keys.just_pressed(KeyCode::KeyB) { return; }
    for (orbit, mut maneuvers) in &mut ships {
        let t = clock.t;
        let v = orbit.elements.velocity_at(t);
        let dv = v * 0.1;
        maneuvers.queue.push_back(Burn { execute_at: t, dv });
    }
}

// P: flip the selected ship between on-rails coast and powered integration.
// Power-up grants 4× local-gravity authority so Hold/StationKeep are feasible.
#[allow(clippy::too_many_arguments)]
pub fn debug_toggle_powered(
    keys: Res<ButtonInput<KeyCode>>,
    debug: Res<DebugUi>,
    clock: Res<SimClock>,
    mut orbit_cache: ResMut<OrbitPropagationCache>,
    mut commands: Commands,
    coasting: Query<&Orbit, With<Maneuvers>>,
    flying: Query<&StateVec, With<Maneuvers>>,
    bodies: Query<&Body>,
) {
    if !keys.just_pressed(KeyCode::KeyP) { return; }
    let Some(e) = debug.selected else { return };

    if let Ok(orbit) = coasting.get(e) {
        let (pos, _) = orbit.elements.state_vectors_at(clock.t);
        let mu = bodies.get(orbit.parent).map_or(0.0, |b| b.mu);
        let r = pos.length();
        let max_accel = (4.0 * mu / (r * r)).max(1.0); // authority over local gravity
        commands.entity(e).remove::<Orbit>().insert((
            StateVec::from_orbit(orbit, clock.t),
            Propulsion { max_accel, throttle: 1.0 },
            ThrustCommand::default(),
            Guidance::Idle,
        ));
        orbit_cache.dirty = true;
        info!("ship {e:?} -> POWERED (max_accel {max_accel:.1})");
    } else if let Ok(sv) = flying.get(e) {
        let Ok(body) = bodies.get(sv.frame) else { return };
        commands.entity(e).remove::<(StateVec, ThrustCommand, Guidance)>()
            .insert(Orbit::park_from_statevec(sv, body.mu, body.radius, clock.t));
        orbit_cache.dirty = true;
        info!("ship {e:?} -> COAST");
    }
}

// TODO: Remove these for a sinular Hold
// K = StationKeep (current radius),
// H = Hold,
// I = Idle.
pub fn debug_guidance_keys(
    keys: Res<ButtonInput<KeyCode>>,
    debug: Res<DebugUi>,
    mut q: Query<(&StateVec, &mut Guidance)>,
) {
    let Some(e) = debug.selected else { return };
    let Ok((sv, mut g)) = q.get_mut(e) else { return };
    if keys.just_pressed(KeyCode::KeyK) { *g = Guidance::StationKeep { radius: sv.pos.length() }; info!("StationKeep"); }
    if keys.just_pressed(KeyCode::KeyH) { *g = Guidance::Hold; info!("Hold"); }
    if keys.just_pressed(KeyCode::KeyI) { *g = Guidance::Idle; info!("Idle"); }
}
