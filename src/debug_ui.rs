use bevy::prelude::*;
use bevy::math::DVec3;
use bevy_egui::{egui, EguiContexts};

use crate::body_traits::Focusable;
use crate::camera::OrbitCam;
use crate::edit::AppMode;
use crate::sim::clock::SimClock;
use crate::sim::orbit::{Body, Burn, Maneuvers, Orbit};
use crate::math::orbital_elements::OrbitalElements;
use crate::sim::soi::soi_radius;
use crate::world_pos::WorldPos;

#[derive(Clone, Copy, PartialEq)]
pub enum BurnFrame {
    Prograde, Retrograde, RadialOut, RadialIn, Normal, AntiNormal,
}

#[derive(Clone)]
pub struct BurnForm {
    pub frame: BurnFrame,
    pub magnitude: f64,
    pub raw: DVec3,
}
impl Default for BurnForm {
    fn default() -> Self {
        Self { frame: BurnFrame::Prograde, magnitude: 0.0002, raw: DVec3::ZERO }
    }
}

#[derive(Clone, Copy, PartialEq)]
pub enum SpawnKind { Body, Ship }

#[derive(Clone)]
pub struct SpawnForm {
    pub kind: SpawnKind,
    pub name: String,
    pub parent: Option<Entity>,
    pub a: f64,
    pub mu: f64,
    pub mesh_radius: f32,
    pub color: [f32; 3],
    pub advanced: bool,
    pub e: f64,
    pub i: f64,
    pub lan: f64,
    pub arg_pe: f64,
    pub m0: f64,
}

impl Default for SpawnForm {
    fn default() -> Self {
        Self {
            kind: SpawnKind::Ship,
            name: "new".to_string(),
            parent: None,
            a: 200.0,
            mu: 1.0e-5,
            mesh_radius: 5.0,
            color: [0.8, 0.4, 0.4],
            advanced: false,
            e: 0.0, i: 0.0, lan: 0.0, arg_pe: 0.0, m0: 0.0,
        }
    }
}

#[derive(Resource, Default)]
pub struct DebugUi {
    pub open: bool,
    pub selected: Option<Entity>,
    pub burn_form: BurnForm,
    pub spawn_form: SpawnForm,
    pub capture_rp: Option<f64>,            // r_p override; None ⇒ planner default
    pub last_mission: Option<MissionReadout>,
    pub focus_request: Option<Entity>,      // if double click list item in debug
}

pub fn debug_is_open(ui:  Res<DebugUi>) -> bool {
    ui.open
}

pub fn toggle_debug_ui(keys: Res<ButtonInput<KeyCode>>, mut ui: ResMut<DebugUi>) {
    if keys.just_pressed(KeyCode::F1) {
        ui.open = !ui.open;
    }
}

#[derive(Clone, Copy)]
pub struct MissionReadout {
    pub t_dep: f64,
    pub wait: f64,
    pub t_peri: f64,
    pub v_inf: f64,
    pub dep_dv: f64,
    pub capture_dv: f64,
    pub total_dv: f64,
    pub r_p: f64,
    pub captured_a: f64,
    pub captured_e: f64,
}

pub fn debug_panel(
    mut contexts: EguiContexts,
    mut state: ResMut<DebugUi>,
    clock: Res<SimClock>,
    mode: Res<State<AppMode>>,
    mut next_mode: ResMut<NextState<AppMode>>,
    cam: Single<&OrbitCam, With<Camera>>,
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut entities: Query<
        (Entity, Option<&Name>, &WorldPos, Option<&mut Orbit>, Option<&Body>, Option<&mut Maneuvers>),
        Or<(With<Orbit>, With<Body>)>,
    >,
) -> Result {
    if !state.open {
        return Ok(());
    }
    let ctx = contexts.ctx_mut()?;

    let t = clock.t;
    let warp = clock.warp();
    let mut selected = state.selected.or(Some(cam.focus));

    // ---- list labels + candidate parents (read-only, before the closure) ----
    let mut items: Vec<(Entity, String)> = Vec::new();
    let mut parents: Vec<(Entity, String)> = Vec::new();
    for (e, name, _wp, orbit, body, maneuvers) in &entities {
        let base = name.map(|n| n.as_str().to_string()).unwrap_or_else(|| format!("{e:?}"));
        let tag = if maneuvers.is_some() { "ship" } else if orbit.is_some() { "body" } else { "root" };
        items.push((e, format!("{base}  [{tag}]")));
        if body.is_some() {
            parents.push((e, base)); // a parent must have mass (Body) to give the child its mu
        }
    }

    // ---- detail + cached state for the selected entity ----
    let mut detail: Vec<String> = Vec::new();
    let mut sel_state: Option<(DVec3, DVec3)> = None;
    let mut sel_el: Option<OrbitalElements> = None;
    let mut sel_is_ship = false;
    let mut sel_queue: Vec<(f64, DVec3)> = Vec::new();

    if let Some(sel) = selected {
        if let Ok((e, name, wp, orbit, body, maneuvers)) = entities.get(sel) {
            sel_is_ship = maneuvers.is_some();
            let label = name.map(|n| n.as_str().to_string()).unwrap_or_else(|| format!("{e:?}"));
            detail.push(format!("=== {label} ==="));
            detail.push(format!("pos = ({:.2}, {:.2}, {:.2})", wp.0.x, wp.0.y, wp.0.z));

            if let Some(b) = body {
                detail.push(format!("mu  = {:.4e}", b.mu));
            }
            if let Some(o) = orbit {
                let el = o.elements;
                sel_el = Some(el);
                let (r_vec, v_vec) = el.state_vectors_at(t);
                sel_state = Some((r_vec, v_vec));
                let r = r_vec.length();
                let v = v_vec.length();
                let visviva = (el.mu * (2.0 / r - 1.0 / el.a)).sqrt();
                detail.push(format!("a = {:.3}    e = {:.4}    i = {:.4} rad", el.a, el.e, el.i));
                detail.push(format!("lan = {:.4}  arg_pe = {:.4}  m0 = {:.4}", el.lan, el.arg_pe, el.m0));
                detail.push(format!("epoch = {:.1}   mu = {:.4e}", el.epoch, el.mu));
                detail.push(format!("|r| = {:.3}    |v| = {:.6}", r, v));
                detail.push(format!("vis-viva |v| = {:.6}    Δ = {:.2e}", visviva, (v - visviva).abs()));

                let parent_label = entities.get(o.parent).ok()
                    .map(|(pe, pn, _, _, _, _)| pn.map(|n| n.as_str().to_string()).unwrap_or_else(|| format!("{pe:?}")))
                    .unwrap_or_else(|| "<root>".to_string());
                detail.push(format!("parent = {parent_label}"));

                if let Some(b) = body {
                    detail.push(format!("own SOI radius = {:.3}", soi_radius(el.a, b.mu, el.mu)));
                }
                if let Ok((_, _, _, Some(po), Some(pb), _)) = entities.get(o.parent) {
                    let p_soi = soi_radius(po.elements.a, pb.mu, po.elements.mu);
                    detail.push(format!("parent SOI = {:.3}    edge in {:.3}", p_soi, p_soi - r));
                }
            }
            if let Some(m) = maneuvers {
                sel_queue = m.queue.iter().map(|b| (b.execute_at, b.dv)).collect();
            }
        } else {
            detail.push("selected entity is not inspectable".to_string());
        }
    }

    // ---- draw ----
    let mut open = state.open;
    let mut bf = state.burn_form.clone();
    let mut capture_rp = state.capture_rp;     // Option<f64>, Copy
    let last_mission = state.last_mission;      // Option<MissionReadout>, Copy
    let mut sf = state.spawn_form.clone();
    if sf.parent.is_none() {
        sf.parent = parents.first().map(|(e, _)| *e); // preselect so the spawn button isn't dead
    }
    let mut clicked: Option<Entity> = None;
    let mut focus_double_click: Option<Entity> = None;
    let mut pending: Option<DVec3> = None;
    let mut do_spawn = false;
    let mut clear_queue = false;
    let mut remove_index: Option<usize> = None;

    // edit-mode controls
    let in_edit = *mode.get() == AppMode::Edit;
    let mut toggle_mode_clicked = false;
    // editable copy of the selected orbit, re-anchored to "now" so resizing/reshaping
    // holds the body's current angular position (and resume is seamless)
    let mut edit_el = sel_el.map(|el| {
        let mut x = el;
        x.m0 = el.mean_anomaly_at(t);
        x.epoch = t;
        x
    });
    let mut el_changed = false;

    egui::Window::new("Debug")
        .open(&mut open)
        .default_width(340.0)
        .show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.label(if in_edit { "MODE: EDIT (paused)" } else { "MODE: RUN" });
                if ui.button(if in_edit { "▶ Run" } else { "⏸ Edit" }).clicked() {
                    toggle_mode_clicked = true;
                }
            });
            ui.label(format!("t = {t:.1} s     warp = {warp:.0} sim-s/s"));
            ui.separator();

            ui.label("entities");
            for (e, label) in &items {
                let resp = ui.selectable_label(Some(*e) == selected, label);
                if resp.clicked() { clicked = Some(*e); } // single click selects 
                if resp.double_clicked() { focus_double_click = Some(*e); } // double click changes focus 
            }
            ui.separator();
            for line in &detail {
                ui.label(line);
            }

            // --- edit orbital elements (Edit mode, on-rails entity) ---
            if in_edit && let Some(ee) = edit_el.as_mut() {
                ui.separator();
                ui.label("edit orbit (epoch = now)");
                ui.horizontal(|ui| {
                    ui.label("a");
                    el_changed |= ui.add(egui::DragValue::new(&mut ee.a).speed(1.0)).changed();
                    ui.label("e");
                    el_changed |= ui.add(egui::DragValue::new(&mut ee.e).speed(0.001)).changed();
                });
                ui.horizontal(|ui| {
                    ui.label("i");
                    el_changed |= ui.add(egui::DragValue::new(&mut ee.i).speed(0.01)).changed();
                    ui.label("lan");
                    el_changed |= ui.add(egui::DragValue::new(&mut ee.lan).speed(0.01)).changed();
                });
                ui.horizontal(|ui| {
                    ui.label("arg_pe");
                    el_changed |= ui.add(egui::DragValue::new(&mut ee.arg_pe).speed(0.01)).changed();
                    ui.label("m0");
                    el_changed |= ui.add(egui::DragValue::new(&mut ee.m0).speed(0.01)).changed();
                });
            }

            if sel_is_ship {
                ui.separator();
                ui.label(format!("maneuver queue ({})", sel_queue.len()));
                for (idx, (exec, dv)) in sel_queue.iter().enumerate() {
                    ui.horizontal(|ui| {
                        ui.label(format!(
                            "[{idx}] in {:+.1}s  |dv|={:.5}  ({:.4}, {:.4}, {:.4})",
                            exec - t, dv.length(), dv.x, dv.y, dv.z
                        ));
                        if ui.small_button("x").clicked() {
                            remove_index = Some(idx);
                        }
                    });
                }
                if !sel_queue.is_empty() && ui.button("clear queue").clicked() {
                    clear_queue = true;
                }
            }

            // --- manual burn (ships only) ---
            if sel_is_ship && let Some((r_vec, v_vec)) = sel_state {
                ui.separator();
                ui.label("manual burn");
                ui.horizontal_wrapped(|ui| {
                    ui.selectable_value(&mut bf.frame, BurnFrame::Prograde, "Pro");
                    ui.selectable_value(&mut bf.frame, BurnFrame::Retrograde, "Retro");
                    ui.selectable_value(&mut bf.frame, BurnFrame::RadialOut, "Rad+");
                    ui.selectable_value(&mut bf.frame, BurnFrame::RadialIn, "Rad−");
                    ui.selectable_value(&mut bf.frame, BurnFrame::Normal, "Nor+");
                    ui.selectable_value(&mut bf.frame, BurnFrame::AntiNormal, "Nor−");
                });
                ui.horizontal(|ui| {
                    ui.label("mag");
                    ui.add(egui::DragValue::new(&mut bf.magnitude).speed(0.0001));
                    if ui.button("queue framed burn").clicked() {
                        let dir = match bf.frame {
                            BurnFrame::Prograde => v_vec.normalize(),
                            BurnFrame::Retrograde => -v_vec.normalize(),
                            BurnFrame::RadialOut => r_vec.normalize(),
                            BurnFrame::RadialIn => -r_vec.normalize(),
                            BurnFrame::Normal => r_vec.cross(v_vec).normalize(),
                            BurnFrame::AntiNormal => -r_vec.cross(v_vec).normalize(),
                        };
                        pending = Some(dir * bf.magnitude);
                    }
                });
                ui.horizontal(|ui| {
                    ui.label("raw dv");
                    ui.add(egui::DragValue::new(&mut bf.raw.x).speed(0.0001));
                    ui.add(egui::DragValue::new(&mut bf.raw.y).speed(0.0001));
                    ui.add(egui::DragValue::new(&mut bf.raw.z).speed(0.0001));
                    if ui.button("queue raw burn").clicked() {
                        pending = Some(bf.raw);
                    }
                });
            }

            // --- spawn ---
            ui.separator();
            ui.collapsing("spawn", |ui| {
                ui.horizontal(|ui| {
                    ui.selectable_value(&mut sf.kind, SpawnKind::Ship, "Ship");
                    ui.selectable_value(&mut sf.kind, SpawnKind::Body, "Body");
                });
                ui.horizontal(|ui| { ui.label("name"); ui.text_edit_singleline(&mut sf.name); });

                let current = sf.parent
                    .and_then(|p| parents.iter().find(|(e, _)| *e == p).map(|(_, l)| l.clone()))
                    .unwrap_or_else(|| "<choose>".to_string());
                egui::ComboBox::from_label("parent").selected_text(current).show_ui(ui, |ui| {
                    for (e, label) in &parents {
                        ui.selectable_value(&mut sf.parent, Some(*e), label);
                    }
                });

                ui.horizontal(|ui| { ui.label("a"); ui.add(egui::DragValue::new(&mut sf.a).speed(1.0)); });
                if sf.kind == SpawnKind::Body {
                    ui.horizontal(|ui| { ui.label("mu"); ui.add(egui::DragValue::new(&mut sf.mu).speed(1e-6)); });
                }
                ui.horizontal(|ui| {
                    ui.label("mesh r");
                    ui.add(egui::DragValue::new(&mut sf.mesh_radius).speed(0.5));
                    ui.color_edit_button_rgb(&mut sf.color);
                });

                ui.checkbox(&mut sf.advanced, "advanced elements");
                if sf.advanced {
                    ui.horizontal(|ui| {
                        ui.label("e"); ui.add(egui::DragValue::new(&mut sf.e).speed(0.001));
                        ui.label("i"); ui.add(egui::DragValue::new(&mut sf.i).speed(0.01));
                    });
                    ui.horizontal(|ui| {
                        ui.label("lan"); ui.add(egui::DragValue::new(&mut sf.lan).speed(0.01));
                        ui.label("arg_pe"); ui.add(egui::DragValue::new(&mut sf.arg_pe).speed(0.01));
                        ui.label("m0"); ui.add(egui::DragValue::new(&mut sf.m0).speed(0.01));
                    });
                }

                let enabled = sf.parent.is_some();
                if ui.add_enabled(enabled, egui::Button::new("spawn")).clicked() {
                    do_spawn = true;
                }
            });

            // missions 
            ui.separator();
            ui.collapsing("mission", |ui| {
                let mut overriding = capture_rp.is_some();
                ui.checkbox(&mut overriding, "override capture r_p");
                if overriding {
                    let mut v = capture_rp.unwrap_or(5.0);
                    ui.add(egui::DragValue::new(&mut v).speed(0.1).prefix("r_p "));
                    capture_rp = Some(v);
                } else {
                    capture_rp = None;
                }

                if let Some(m) = last_mission {
                    ui.label(format!("depart t={:.0}   (in {:.0}s)", m.t_dep, m.wait));
                    ui.label(format!("v_inf = {:.5}", m.v_inf));
                    ui.label(format!("Δv = {:.5} + {:.5} = {:.5}", m.dep_dv, m.capture_dv, m.total_dv));
                    ui.label(format!("capture t={:.0}", m.t_peri));
                    ui.label(format!("orbit a={:.3}  e={:.4}  (r_p {:.3})", m.captured_a, m.captured_e, m.r_p));
                } else {
                    ui.label("no mission planned yet");
                }
            });
        });

    // ---- writes burn the closure ---
    if let Some(target) = selected && (pending.is_some() || clear_queue || remove_index.is_some()) 
        && let Ok((.., Some(mut man))) = entities.get_mut(target) {
            if let Some(dv) = pending {
                man.queue.push_back(Burn { execute_at: t, dv });
            }
            if let Some(idx) = remove_index && idx < man.queue.len() {
                man.queue.remove(idx);
        }
            if clear_queue {
                man.queue.clear();
            }
        }

    if do_spawn && let Some(parent) = sf.parent &&
        let Ok((_, _, _, _, Some(pbody), _)) = entities.get(parent) {
            let parent_mu = pbody.mu;
            let elements = OrbitalElements {
                a: sf.a,
                e: if sf.advanced { sf.e } else { 0.0 },
                i: if sf.advanced { sf.i } else { 0.0 },
                lan: if sf.advanced { sf.lan } else { 0.0 },
                arg_pe: if sf.advanced { sf.arg_pe } else { 0.0 },
                m0: if sf.advanced { sf.m0 } else { 0.0 },
                epoch: t,                 // warp-correct, exactly like a burn
                mu: parent_mu,            // child's mu = parent's G·M (the invariant)
            };
            let mesh = meshes.add(Sphere::new(sf.mesh_radius));
            let material = materials.add(Color::srgb(sf.color[0], sf.color[1], sf.color[2]));
            let mut ec = commands.spawn((
                Mesh3d(mesh),
                MeshMaterial3d(material),
                Transform::default(),
                WorldPos::ORIGIN,
                Name::new(sf.name.clone()),
                Focusable::default(),
                Orbit { elements, parent },
            ));
            match sf.kind {
                SpawnKind::Body => { ec.insert(Body { mu: sf.mu, radius: sf.mesh_radius as f64 }); }
                SpawnKind::Ship => { ec.insert(Maneuvers::default()); }
            }
    }

    // write edited elements back to the selected entity (uses the displayed selection,
    // before any click below reassigns it)
    if el_changed && let Some(target) = selected && let Some(new_el) = edit_el
        && let Ok((.., Some(mut orbit), _, _)) = entities.get_mut(target) {
            orbit.elements = new_el;
        }

    if toggle_mode_clicked {
        next_mode.set(if in_edit { AppMode::Run } else { AppMode::Edit });
    }

    if let Some(e) = clicked {
        selected = Some(e);
    }
    if let Some(e) = focus_double_click {
        state.focus_request = Some(e);
    }
    state.selected = selected;
    state.burn_form = bf;
    state.spawn_form = sf;
    state.open = open;
    state.capture_rp = capture_rp;
    Ok(())
}
