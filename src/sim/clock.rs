use bevy::prelude::*;

// ==== speed settings ====
const DAY: f64 = 60.0 * 60.0 * 24.0;
const WARP_LEVELS: [f64; 9] = [
    0.0,          // paused
    0.125 * DAY,
    0.25 * DAY,
    0.50 * DAY,
    1.0  * DAY,   // 1 day 
    2.0  * DAY, 
    5.0  * DAY, 
    10.0 * DAY,
    30.0 * DAY,   // month 
];

#[derive(Resource)]
pub struct SimClock {
    pub t: f64,   // seconds 
    level: usize, // index for WARP_LEVELS
}

impl Default for SimClock {
    fn default() -> Self {
        Self { t: 0.0, level: 1 }
    }
}

impl SimClock {
    pub fn warp(&self) -> f64 { WARP_LEVELS[self.level] }

    // force warp to the paused level (index 0). Loading a world starts frozen.
    pub fn pause(&mut self) { self.level = 0; }

    fn faster(&mut self) { 
        self.level = (self.level + 1).min(WARP_LEVELS.len() - 1);
    }

    fn slower(&mut self) {
        self.level = self.level.saturating_sub(1);
    }
}

pub fn advance_clock(time: Res<Time>, mut clock: ResMut<SimClock>) {
    clock.t += clock.warp() * time.delta_secs() as f64;
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
