// egui UI for the start screen  and the pause menu 
// only writes current level and world or changes game state  / app mode 

use bevy::prelude::*;
use bevy::app::AppExit;
use bevy_egui::{egui, EguiContexts};

use crate::game_state::{AppMode, GameState};
use crate::worlds::{self, CurrentWorld};

// run_if(AppMode::Menu): play a world (Run) or quit.
pub fn start_screen(
    mut contexts: EguiContexts,
    mut world: ResMut<CurrentWorld>,
    mut app_next: ResMut<NextState<AppMode>>,
    mut exit: MessageWriter<AppExit>,
) -> Result {
    let ctx = contexts.ctx_mut()?;
    egui::Window::new("Start Menu")
        // turn the next three to true for small window use 
        .title_bar(false) 
        .collapsible(false)
        .resizable(false)
        .anchor(egui::Align2::CENTER_CENTER, egui::Vec2::ZERO) // makes window immovable, centers it 
        .show(ctx, |ui| {
            ui.vertical_centered(|ui| {
                ui.heading("Orbital");
                ui.add_space(12.0);

                // --- play ---
                if ui.button("New World").clicked() {
                    world.0 = Some(worlds::next_world_name()); // load_scene seeds the default
                    app_next.set(AppMode::Run);
                }
                ui.add_space(6.0);
                ui.label("Load world:");
                let world_slots = worlds::list_worlds();
                if world_slots.is_empty() {
                    ui.weak("(none yet)");
                }
                for slot in world_slots {
                    let label = format!("{}   (t = {:.0})", slot.name, slot.meta.sim_time);
                    if ui.button(label).clicked() {
                        world.0 = Some(slot.name);
                        app_next.set(AppMode::Run);
                    }
                }

                ui.add_space(14.0);
                if ui.button("Quit").clicked() {
                    exit.write(AppExit::Success);
                }
            });
        });
    Ok(())
}

// menu for once systems are running.
pub fn pause_menu(
    mut contexts: EguiContexts,
    mut next: ResMut<NextState<GameState>>,
    mut app_next: ResMut<NextState<AppMode>>,
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
                if ui.button("Resume").clicked() { next.set(GameState::Running); }
                if ui.button("Save").clicked() { next.set(GameState::Saving); }
                if ui.button("Main Menu").clicked() { app_next.set(AppMode::Menu); }
            });
        });
    Ok(())
}
