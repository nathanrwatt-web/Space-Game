// Dedicated editor GUI: a top bar (time / save / exit / window toggles), a right-side
// Hierarchy window (tree + double-click zoom + inline inspector), and a Spawn window. UI only —
// the camera lives in camera.rs, persistence + time stepping in mod.rs.

use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts};
use std::collections::HashMap;

use crate::game_state::{AppMode, Appearance};
use crate::math::orbital_elements::OrbitalElements;
use crate::sim::clock::SimClock;
use crate::sim::orbit::{Body, Orbit};
use crate::world_pos::WorldPos;
use super::{fmt_time, EditorBody, EditorFocus, EditorSaveRequest, EditorSelection, EditorSpawnForm, EditorWindows};

// one tree node + recurse into its children
fn show_node(
    ui: &mut egui::Ui,
    e: Entity,
    name: &str,
    children: &HashMap<Entity, Vec<(Entity, String)>>,
    selection: Option<Entity>,
    clicked: &mut Option<Entity>,
    dbl: &mut Option<Entity>,
) {
    let resp = ui.selectable_label(selection == Some(e), name);
    if resp.clicked() { *clicked = Some(e); }
    if resp.double_clicked() { *dbl = Some(e); }
    if let Some(kids) = children.get(&e) {
        ui.indent(e, |ui| {
            for (ce, cn) in kids {
                show_node(ui, *ce, cn, children, selection, clicked, dbl);
            }
        });
    }
}

#[allow(clippy::too_many_arguments, clippy::type_complexity)]
pub fn editor_ui(
    mut contexts: EguiContexts,
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut clock: ResMut<SimClock>,
    mut save_req: ResMut<EditorSaveRequest>,
    mut app_next: ResMut<NextState<AppMode>>,
    mut windows: ResMut<EditorWindows>,
    mut selection: ResMut<EditorSelection>,
    mut focus: ResMut<EditorFocus>,
    mut form: ResMut<EditorSpawnForm>,
    mut bodies: Query<(Entity, &Name, Option<&mut Orbit>, Option<&Body>, &mut Appearance), With<EditorBody>>,
) -> Result {
    let ctx = contexts.ctx_mut()?;
    let t = clock.t;
    let warp = clock.warp();
    let sel_now = selection.0;

    // ---- collect, before the egui closures ----
    let mut roots: Vec<(Entity, String)> = Vec::new();
    let mut children: HashMap<Entity, Vec<(Entity, String)>> = HashMap::new();
    let mut parents: Vec<(Entity, String)> = Vec::new(); // candidate parents (have mass)
    let mut sel_has_orbit = false;
    let mut edit_el: Option<OrbitalElements> = None;
    let mut edit_radius = 0.0_f32;
    let mut edit_color = [0.0_f32; 3];

    for (e, name, orbit, body, appearance) in &bodies {
        let nm = name.as_str().to_string();
        match orbit.map(|o| o.parent) {
            Some(p) => children.entry(p).or_default().push((e, nm.clone())),
            None => roots.push((e, nm.clone())),
        }
        if body.is_some() {
            parents.push((e, nm));
        }
        if Some(e) == sel_now {
            sel_has_orbit = orbit.is_some();
            if let Some(o) = orbit { edit_el = Some(o.elements); }
            if let Appearance::Sphere { radius, color } = appearance {
                edit_radius = *radius;
                edit_color = *color;
            }
        }
    }
    if form.parent.is_none() {
        form.parent = parents.first().map(|(e, _)| *e);
    }

    // ---- draw ----
    let mut sf = form.clone();
    let mut hierarchy_open = windows.hierarchy;
    let mut spawn_open = windows.spawn;
    let (mut clicked, mut dbl): (Option<Entity>, Option<Entity>) = (None, None);
    let (mut do_spawn, mut do_save, mut do_exit) = (false, false, false);
    let (mut faster, mut slower) = (false, false);
    let (mut el_changed, mut appearance_changed) = (false, false);

    egui::TopBottomPanel::top("editor_bar").show(ctx, |ui| {
        ui.horizontal(|ui| {
            ui.label(format!("t = {}", fmt_time(t)));
            if ui.button("−").clicked() { slower = true; }
            ui.label(format!("{warp:.0} s/s"));
            if ui.button("+").clicked() { faster = true; }
            ui.separator();
            if ui.button("Save").clicked() { do_save = true; }
            if ui.button("Exit").clicked() { do_exit = true; }
            ui.separator();
            ui.checkbox(&mut hierarchy_open, "Hierarchy");
            ui.checkbox(&mut spawn_open, "Spawn");
            ui.weak("[ / ] step");
        });
    });

    egui::Window::new("Hierarchy")
        .open(&mut hierarchy_open)
        .anchor(egui::Align2::RIGHT_TOP, egui::vec2(-8.0, 36.0))
        .default_width(220.0)
        .show(ctx, |ui| {
            for (e, name) in &roots {
                show_node(ui, *e, name, &children, sel_now, &mut clicked, &mut dbl);
            }

            if let Some(sel) = sel_now {
                if sel_has_orbit && let Some(ee) = edit_el.as_mut() {
                    ui.separator();
                    ui.label("orbit");
                    el_changed |= ui.add(egui::DragValue::new(&mut ee.a).speed(10.0).prefix("a ")).changed();
                    el_changed |= ui.add(egui::DragValue::new(&mut ee.e).speed(0.001).prefix("e ")).changed();
                    el_changed |= ui.add(egui::DragValue::new(&mut ee.i).speed(0.01).prefix("i ")).changed();
                    el_changed |= ui.add(egui::DragValue::new(&mut ee.lan).speed(0.01).prefix("lan ")).changed();
                    el_changed |= ui.add(egui::DragValue::new(&mut ee.arg_pe).speed(0.01).prefix("arg ")).changed();
                    el_changed |= ui.add(egui::DragValue::new(&mut ee.m0).speed(0.01).prefix("m0 ")).changed();
                }
                ui.separator();
                ui.label("appearance");
                ui.horizontal(|ui| {
                    appearance_changed |= ui.add(egui::DragValue::new(&mut edit_radius).speed(0.5).prefix("r ")).changed();
                    appearance_changed |= ui.color_edit_button_rgb(&mut edit_color).changed();
                });
                let _ = sel;
            }
        });

    egui::Window::new("Spawn")
        .open(&mut spawn_open)
        .default_width(220.0)
        .show(ctx, |ui| {
            ui.horizontal(|ui| { ui.label("name"); ui.text_edit_singleline(&mut sf.name); });

            let current = sf.parent
                .and_then(|p| parents.iter().find(|(e, _)| *e == p).map(|(_, l)| l.clone()))
                .unwrap_or_else(|| "<choose>".to_string());
            egui::ComboBox::from_label("parent").selected_text(current).show_ui(ui, |ui| {
                for (e, label) in &parents {
                    ui.selectable_value(&mut sf.parent, Some(*e), label);
                }
            });

            ui.horizontal(|ui| {
                ui.add(egui::DragValue::new(&mut sf.a).speed(10.0).prefix("a "));
                ui.add(egui::DragValue::new(&mut sf.e).speed(0.001).prefix("e "));
                ui.add(egui::DragValue::new(&mut sf.i).speed(0.01).prefix("i "));
            });
            ui.horizontal(|ui| {
                ui.add(egui::DragValue::new(&mut sf.lan).speed(0.01).prefix("lan "));
                ui.add(egui::DragValue::new(&mut sf.arg_pe).speed(0.01).prefix("arg "));
                ui.add(egui::DragValue::new(&mut sf.m0).speed(0.01).prefix("m0 "));
            });
            ui.horizontal(|ui| {
                ui.add(egui::DragValue::new(&mut sf.radius).speed(0.5).prefix("r "));
                ui.color_edit_button_rgb(&mut sf.color);
                ui.add(egui::DragValue::new(&mut sf.mu).speed(0.1).prefix("mu "));
            });

            let enabled = sf.parent.is_some();
            if ui.add_enabled(enabled, egui::Button::new("Spawn")).clicked() { do_spawn = true; }
        });

    // ---- writeback ----
    windows.hierarchy = hierarchy_open;
    windows.spawn = spawn_open;
    if faster { clock.faster(); }
    if slower { clock.slower(); }
    if do_save { save_req.0 = true; }
    if do_exit { app_next.set(AppMode::Menu); }

    // inspector edits apply to the body that was rendered (sel_now), before any new click
    if (el_changed || appearance_changed) && let Some(sel) = sel_now
        && let Ok((_, _, orbit, _, mut appearance)) = bodies.get_mut(sel) {
        if el_changed && let (Some(mut o), Some(new_el)) = (orbit, edit_el) {
            o.elements = new_el;
        }
        if appearance_changed {
            *appearance = Appearance::Sphere { radius: edit_radius, color: edit_color };
            let mesh = meshes.add(Sphere::new(edit_radius));
            let material = materials.add(Color::srgb(edit_color[0], edit_color[1], edit_color[2]));
            commands.entity(sel).insert((Mesh3d(mesh), MeshMaterial3d(material)));
        }
    }

    if do_spawn && let Some(parent) = sf.parent
        && let Ok((_, _, _, Some(pbody), _)) = bodies.get(parent) {
        let parent_mu = pbody.mu;
        let elements = OrbitalElements {
            a: sf.a, e: sf.e, i: sf.i, lan: sf.lan, arg_pe: sf.arg_pe, m0: sf.m0,
            epoch: t, mu: parent_mu,
        };
        let mesh = meshes.add(Sphere::new(sf.radius));
        let material = materials.add(Color::srgb(sf.color[0], sf.color[1], sf.color[2]));
        commands.spawn((
            Name::new(sf.name.clone()),
            Mesh3d(mesh),
            MeshMaterial3d(material),
            Transform::default(),
            WorldPos::ORIGIN,
            Appearance::Sphere { radius: sf.radius, color: sf.color },
            Orbit { elements, parent },
            Body { mu: sf.mu, radius: sf.radius as f64 },
            EditorBody,
        ));
    }

    *form = sf;
    if let Some(e) = clicked { selection.0 = Some(e); }
    if let Some(e) = dbl { focus.0 = Some(e); selection.0 = Some(e); }
    Ok(())
}
