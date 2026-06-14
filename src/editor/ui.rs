// Dedicated editor GUI: spawn spheres, edit the selected orbit numerically, time controls,
// save / exit. (The free camera lives in camera.rs; persistence + time stepping in mod.rs.)

use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts};

use crate::game_state::{AppMode, Appearance};
use crate::math::orbital_elements::OrbitalElements;
use crate::sim::clock::SimClock;
use crate::sim::orbit::Orbit;
use crate::world_pos::WorldPos;
use super::{fmt_time, EditorBody, EditorSaveRequest};

#[allow(clippy::too_many_arguments)]
pub fn editor_panel(
    mut contexts: EguiContexts,
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut clock: ResMut<SimClock>,
    mut save_req: ResMut<EditorSaveRequest>,
    mut app_next: ResMut<NextState<AppMode>>,
    roots: Query<Entity, (With<EditorBody>, Without<Orbit>)>, // candidate parents (no orbit)
    mut bodies: Query<(Entity, &Name, &mut Orbit), With<EditorBody>>,
    mut selected: Local<Option<Entity>>,
    mut spawn_count: Local<u32>,
) -> Result {
    let ctx = contexts.ctx_mut()?;
    egui::Window::new("Level Editor").default_width(280.0).show(ctx, |ui| {
        // --- time ---
        ui.label(format!("t = {}", fmt_time(clock.t)));
        ui.horizontal(|ui| {
            if ui.button("−").clicked() { clock.slower(); }
            ui.label(format!("{:.0} s/s", clock.warp()));
            if ui.button("+").clicked() { clock.faster(); }
        });
        ui.weak("[ / ] single-step");
        ui.separator();

        // --- spawn ---
        if ui.button("Spawn Sphere").clicked() {
            if let Some(root) = roots.iter().next() {
                *spawn_count += 1;
                let r = 50.0_f32;
                let color = [0.6, 0.8, 0.6];
                let mesh = meshes.add(Sphere::new(r));
                let material = materials.add(Color::srgb(color[0], color[1], color[2]));
                commands.spawn((
                    Name::new(format!("body_{}", *spawn_count)),
                    Mesh3d(mesh),
                    MeshMaterial3d(material),
                    Transform::default(),
                    WorldPos::ORIGIN,
                    Appearance::Sphere { radius: r, color },
                    Orbit {
                        elements: OrbitalElements {
                            a: 2000.0, e: 0.0, i: 0.0, lan: 0.0,
                            arg_pe: 0.0, m0: 0.0, epoch: clock.t, mu: 100.0,
                        },
                        parent: root,
                    },
                    EditorBody,
                ));
            } else {
                ui.label("(spawn a Center first)");
            }
        }
        ui.separator();

        // --- body list ---
        ui.label("bodies");
        for (e, name, _) in &bodies {
            if ui.selectable_label(*selected == Some(e), name.as_str()).clicked() {
                *selected = Some(e);
            }
        }

        // --- numeric orbit edit for the selection ---
        if let Some(sel) = *selected && let Ok((_, _, mut orbit)) = bodies.get_mut(sel) {
                ui.separator();
                ui.label("orbit");
                let el = &mut orbit.elements;
                ui.add(egui::DragValue::new(&mut el.a).speed(10.0).prefix("a "));
                ui.add(egui::DragValue::new(&mut el.e).speed(0.001).prefix("e "));
                ui.add(egui::DragValue::new(&mut el.i).speed(0.01).prefix("i "));
                ui.add(egui::DragValue::new(&mut el.m0).speed(0.01).prefix("m0 "));
        }

        ui.separator();
        if ui.button("Save").clicked() { save_req.0 = true; }
        if ui.button("Exit to Menu").clicked() { app_next.set(AppMode::Menu); }
    });
    Ok(())
}
