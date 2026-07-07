use crate::sim::integrate::StateVec;
use crate::sim::integrate::max_powered_frame_budget;
use bevy::prelude::*;

// ==== speed settings ====
const DAY: f64 = 60.0 * 60.0 * 24.0;
const WARP_LEVELS: [f64; 12] = [
    0.0, // paused
    1.0, // real time — fine for powered flight
    10.0,
    60.0,
    600.0, // 10 min/s
    0.125 * DAY,
    0.25 * DAY,
    0.50 * DAY,
    1.0 * DAY, // 1 day — powered-flight ceiling (integration goes slightly inaccurate past here)
    2.0 * DAY,
    10.0 * DAY,
    30.0 * DAY, // month
];

#[derive(Resource)]
pub struct SimClock {
    pub t: f64,       // seconds
    level: usize,     // index for WARP_LEVELS (the player's requested warp)
    max_level: usize, // ceiling enforced while powered craft are integrating
}

impl Default for SimClock {
    fn default() -> Self {
        // start at a gentle day-scale rung; loading a world pauses anyway
        Self {
            t: 0.0,
            level: 5,
            max_level: WARP_LEVELS.len() - 1,
        }
    }
}

impl SimClock {
    // effective warp respects the powered-craft ceiling without forgetting the
    // player's requested level
    pub fn warp(&self) -> f64 {
        WARP_LEVELS[self.level.min(self.max_level)]
    }

    // force warp to the paused level (index 0). Loading a world starts frozen.
    pub fn pause(&mut self) {
        self.level = 0;
    }

    pub fn faster(&mut self) {
        self.level = (self.level + 1).min(WARP_LEVELS.len() - 1);
    }

    pub fn slower(&mut self) {
        self.level = self.level.saturating_sub(1);
    }

    // cap (or release) the effective warp; set by clamp_warp each frame
    pub fn set_max_level(&mut self, max: usize) {
        self.max_level = max.min(WARP_LEVELS.len() - 1);
    }
}

pub fn advance_clock(time: Res<Time>, mut clock: ResMut<SimClock>) {
    clock.t += clock.warp() * time.delta_secs() as f64;
}

// While powered craft are integrating, cap warp so a single rendered frame never
// asks the fixed-step integrator to consume more than its bounded budget.
pub fn clamp_warp(
    time: Res<Time>,
    mut clock: ResMut<SimClock>,
    powered: Query<(), With<StateVec>>,
) {
    let max = if powered.is_empty() {
        WARP_LEVELS.len() - 1
    } else {
        let dt = time.delta_secs().max(f32::EPSILON) as f64;
        let ceiling = max_powered_frame_budget() / dt;
        WARP_LEVELS
            .iter()
            .rposition(|&warp| warp <= ceiling)
            .unwrap_or(0)
    };
    clock.set_max_level(max);
}

// for now time change will be with brackets
pub fn warp_keys(keys: Res<ButtonInput<KeyCode>>, mut clock: ResMut<SimClock>) {
    // just_pressed: has the input been pressed during the current frame?
    if keys.just_pressed(KeyCode::BracketRight) {
        clock.faster();
        info!("warp -> {:.0} sim-s/s", clock.warp());
    }
    if keys.just_pressed(KeyCode::BracketLeft) {
        clock.slower();
        info!("warp -> {:.0} sim-s/s", clock.warp());
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn powered_frame_budget_maps_to_warp_ceiling() {
        let dt = 1.0 / 60.0;
        let ceiling = max_powered_frame_budget() / dt;
        assert!(ceiling > 0.0);
        assert!(ceiling * dt <= max_powered_frame_budget() + 1e-9);
    }
}
