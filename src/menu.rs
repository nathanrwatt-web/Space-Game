// egui UI for the start screen (MainMenu) and the pause menu (Paused). UI only — world
// creation/loading is just a CurrentWorld write + a state transition; the work happens in
// game_state::load_scene / save_scene / despawn_world.

use bevy::prelude::*;
use bevy::app::AppExit;
use bevy_egui::{egui, EguiContexts};

use crate::game_state::GameState;
use crate::worlds::{self, CurrentWorld};

// run_if(MainMenu): list worlds to load, start a new one, or quit to desktop.
pub fn start_screen(
    mut contexts: EguiContexts,
    mut current: ResMut<CurrentWorld>,
    mut next: ResMut<NextState<GameState>>,
    mut exit: MessageWriter<AppExit>,
) -> Result {
    let ctx = contexts.ctx_mut()?;
    egui::Window::new("start")
        .title_bar(false)
        .collapsible(false)
        .resizable(false)
        .anchor(egui::Align2::CENTER_CENTER, egui::Vec2::ZERO)
        .show(ctx, |ui| {
            ui.vertical_centered(|ui| {
                ui.heading("Orbital");
                ui.add_space(12.0);

                if ui.button("New World").clicked() {
                    current.0 = Some(worlds::next_world_name()); // load_scene seeds the default
                    next.set(GameState::Loading);
                }

                ui.add_space(10.0);
                ui.label("Load world:");
                let worlds = worlds::list_worlds();
                if worlds.is_empty() {
                    ui.weak("(none yet)");
                }
                for slot in worlds {
                    let label = format!("{}   (t = {:.0})", slot.name, slot.meta.sim_time);
                    if ui.button(label).clicked() {
                        current.0 = Some(slot.name);
                        next.set(GameState::Loading);
                    }
                }

                ui.add_space(12.0);
                if ui.button("Quit").clicked() {
                    exit.write(AppExit::Success);
                }
            });
        });
    Ok(())
}

// run_if(Paused): resume, save the current world, or quit back to the start screen.
pub fn pause_menu(
    mut contexts: EguiContexts,
    mut next: ResMut<NextState<GameState>>,
) -> Result {
    let ctx = contexts.ctx_mut()?;
    egui::Window::new("paused")
        .title_bar(false)
        .collapsible(false)
        .resizable(false)
        .anchor(egui::Align2::CENTER_CENTER, egui::Vec2::ZERO)
        .show(ctx, |ui| {
            ui.vertical_centered(|ui| {
                ui.heading("Paused");
                ui.add_space(8.0);
                if ui.button("Resume").clicked()    { next.set(GameState::Running); }
                if ui.button("Save").clicked()      { next.set(GameState::Saving); }
                if ui.button("Main Menu").clicked() { next.set(GameState::MainMenu); }
            });
        });
    Ok(())
}
