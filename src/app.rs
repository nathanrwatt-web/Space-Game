// Application assembly: the App is split into cohesive Bevy Plugins, and the
// gameplay pipeline ordering lives in one ordered GameSet chain.

use bevy::{math::DQuat, prelude::*};
use bevy_egui::{EguiPlugin, EguiPrimaryContextPass, input::EguiWantsInput};

use crate::camera::{OrbitCam, orbit_camera, focus_on_click};
use crate::edit::{spawn_handles, position_handles, drag_handle, run_handle_target, HandleTarget};
use crate::log_capture::{LogWindow, log_panel};
use crate::sim::orbit::{Orbit, Maneuvers, Body, shell_radius, propagate_orbits, draw_orbits, execute_maneuvers};
use crate::sim::{
    soi::{draw_soi, update_soi, soi_radius},
    clock::{SimClock, warp_keys, advance_clock, clamp_warp},
    mission::{plan_mission, plan_escape, plan_root_capture},
    capture::{ScheduledCapture, execute_capture},
    integrate::{PhysAccumulator, integrate_powered},
    guidance::apply_guidance,
    broadphase::{Broadphase, build_broadphase},
};
use crate::math::orbital_elements::OrbitalElements;
use crate::world_pos::WorldPos;
use crate::body_traits::Focusable;
use crate::debug::{DebugUi, MissionReadout, debug_is_open, debug_camera, debug_cam_inactive};
use crate::game_state::{AppMode, GameState, not_menu, load_scene, save_scene, toggle_mode, despawn_world};
use crate::menu::{start_screen, pause_menu};
use crate::worlds::CurrentWorld;
use crate::ship_control::move_order;

// Ordered gameplay pipeline. Systems keep their own run conditions
#[derive(SystemSet, Clone, Copy, Debug, PartialEq, Eq, Hash)]
enum GameSet {
    Time,        // warp input + clock advance
    Maneuver,    // impulsive burns
    Broadphase,  // rebuild the spatial grid
    Soi,         // sphere-of-influence transitions
    Capture,     // scheduled orbital insertions
    Guidance,    // thrust commands from guidance laws
    Integrate,   // numerical integration of powered craft
    Propagate,   // analytic positions from orbits (Run AND Edit)
    Camera,      // camera follow + focus
    Draw,        // debug gizmos (orbits, SOI)
}

// ===== World / state lifecycle =====
pub struct WorldPlugin;
impl Plugin for WorldPlugin {
    fn build(&self, app: &mut App) {
        app
            // resources for current file being used
            .init_resource::<CurrentWorld>()
            // while running states
            .init_state::<AppMode>()
            // Menu / Running / Editting
            .add_sub_state::<GameState>()
            // run (world) lifecycle
            .add_systems(OnExit(AppMode::Run), despawn_world)
            .add_systems(OnEnter(GameState::Loading), load_scene)
            .add_systems(OnEnter(GameState::Saving), save_scene);
    }
}

// ===== Simulation core (orbits, thrust, SOI, broadphase, the gameplay pipeline) =====
pub struct SimPlugin;
impl Plugin for SimPlugin {
    fn build(&self, app: &mut App) {
        app
        // time keeping resource
        .init_resource::<SimClock>()
        // for numerical integation (Any where N-body or constant thrust)
        .init_resource::<PhysAccumulator>()
        // grid for body calculation
        .init_resource::<Broadphase>()
        // the ordered gameplay pipeline; members carry their own run conditions
        .configure_sets(Update, (
                GameSet::Time, GameSet::Maneuver, GameSet::Broadphase, GameSet::Soi,
                GameSet::Capture, GameSet::Guidance, GameSet::Integrate, GameSet::Propagate,
                GameSet::Camera, GameSet::Draw,
            ).chain())
        // gameplay: runs only inside a loaded world
        .add_systems(Update, (
                warp_keys.in_set(GameSet::Time),                                          // time change settings
                clamp_warp.in_set(GameSet::Time).before(advance_clock),                   // cap warp while powered craft fly
                advance_clock.in_set(GameSet::Time).run_if(in_state(GameState::Running)), // change the time
                execute_maneuvers.in_set(GameSet::Maneuver),                              // regular burns
                build_broadphase.in_set(GameSet::Broadphase),                             // internal grid map
                update_soi.in_set(GameSet::Soi),                                          // update spheres of influence
                execute_capture.in_set(GameSet::Capture),                                 // capture bodies in soi
                apply_guidance.in_set(GameSet::Guidance),                                 // generate thrust command for numerical inegration
                integrate_powered.in_set(GameSet::Integrate),                             // numerical integration for thrust
            ).run_if(in_state(AppMode::Run)))
        // positions come from elements + clock (only Run is non-menu now)
        .add_systems(Update, propagate_orbits.in_set(GameSet::Propagate).run_if(not_menu))
        // draw orbits and soi helper gizmos (when the debug inspector is open)
        .add_systems(Update, (draw_orbits, draw_soi).chain().in_set(GameSet::Draw).run_if(debug_is_open).run_if(not_menu))
        // right-click to plan transfer/capture burns
        .add_observer(intercept_transfer);
    }
}

// ===== Camera, focus, and f64 -> f32 render sync =====
pub struct CameraPlugin;
impl Plugin for CameraPlugin {
    fn build(&self, app: &mut App) {
        app
        .add_systems(Startup, (setup, configure_gizmos))
        // focus requests are consumed just before the camera follows them
        // skipped while the free-flight debug camera has control of the entity
        .add_systems(Update, (
                apply_focus_request.before(orbit_camera),
                orbit_camera,
            ).in_set(GameSet::Camera).run_if(in_state(AppMode::Run)).run_if(debug_cam_inactive))
        // after all the position udpates, render it to the screen (any non-menu mode).
        // MUST run before transform propagation, or the GlobalTransform used for rendering
        // is one frame stale — which looks like the world lagging/skipping when the camera moves.
        .add_systems(PostUpdate, sync_render_space
            .before(TransformSystems::Propagate)
            .run_if(not_menu))
        // primary click on Focusable bodies
        .add_observer(focus_on_click);
    }
}

// ===== Player ship commands (select + move-to-point) =====
pub struct ShipControlPlugin;
impl Plugin for ShipControlPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Update, move_order.run_if(in_state(AppMode::Run)));
    }
}

// ===== Orbit-edit gizmo handles (both Run and Edit, self-gated via HandleTarget) =====
pub struct EditHandlePlugin;
impl Plugin for EditHandlePlugin {
    fn build(&self, app: &mut App) {
        app
        .init_resource::<HandleTarget>()
        .add_systems(Startup, spawn_handles)
        // Run path: drive the orbit-gizmo target from the debug selection
        .add_systems(Update, run_handle_target.run_if(in_state(AppMode::Run)))
        // orbit-edit handles: self-gated via HandleTarget, after whichever camera updates
        .add_systems(Update, position_handles.after(orbit_camera).after(debug_camera))
        // drag edit handles to modify orbit
        .add_observer(drag_handle);
    }
}

// ===== egui: debug panel, log window, menus =====
pub struct UiPlugin;
impl Plugin for UiPlugin {
    fn build(&self, app: &mut App) {
        app
        .add_plugins(EguiPlugin::default())
        // info!() mirror
        .init_resource::<LogWindow>()
        // input + GUI systems
        .add_systems(Update, toggle_mode.run_if(in_state(AppMode::Run)))
        .add_systems(EguiPrimaryContextPass, (
                // the log is a debug tool: only while the F1 overlay is up, toggled from its toolbar
                log_panel.run_if(in_state(AppMode::Run)).run_if(debug_is_open),
                start_screen.run_if(in_state(AppMode::Menu)),
                pause_menu.run_if(in_state(GameState::Paused)),
            ));
    }
}

// summons light + camera only, the rest of loading is handed to
// the world load_scene system
fn setup(
    mut commands: Commands,
) {
    // sun shines parallel from far away
    commands.spawn((DirectionalLight {
        illuminance: 8000.0,
        shadows_enabled: false,
        ..default() },
        Transform::from_rotation(Quat::from_euler(EulerRot::XYZ, -0.7, 0.5, 0.0)),
    ));

    commands.spawn((
        Camera3d::default(),
        Transform::default(),
        WorldPos::new(0.0, 10000.0, 25000.0),
        OrbitCam {
            focus: Entity::PLACEHOLDER,
            focus_point: WorldPos::ORIGIN.0,
            orientation: DQuat::from_rotation_x(-0.6),
            distance: 25000.0,
            last_focus: Entity::PLACEHOLDER,
            last_focus_pos: WorldPos::ORIGIN.0,
        },
    ));
}

// crisper gizmo lines across the whole app (orbits, SOI, debug grid/axes/handles)
fn configure_gizmos(mut store: ResMut<GizmoConfigStore>) {
    let (config, _) = store.config_mut::<DefaultGizmoConfigGroup>();
    config.line.width = 2.0;
}

// translate f64 math to f32 for rendering
fn sync_render_space( camera: Single<&WorldPos, With<Camera>>, mut bodies: Query<(&WorldPos, &mut Transform)>, ) {
    let origin = **camera;                              // the camera's f64 position
    for (pos, mut transform) in &mut bodies {
        transform.translation = pos.to_render_space(origin); // subtract in f64, then cast
    }
}

// adds a mission which intercepts another body with a hohmann transfer
fn intercept_transfer(
    click: On<Pointer<Click>>,
    mut debug: ResMut<DebugUi>,
    egui_wants: Res<EguiWantsInput>,
    clock: Res<SimClock>,
    mut commands: Commands,
    mut ships: Query<(&Orbit, &mut Maneuvers)>,
    bodies: Query<(Entity, &Orbit, &Body, &Focusable), Without<Maneuvers>>,
    roots: Query<(&Body, &Focusable), (Without<Orbit>, Without<Maneuvers>)>,
) {
    if !debug.open || egui_wants.wants_any_pointer_input() {
        info!("No egui or right clicked on egui panel");
        return;
    }
    if click.event.button != PointerButton::Secondary { return; }

    let Some(ship_e) = debug.selected else {
        info!("No ship selected");
        return;
    };

    // make sure the selected ship is indeed a ship
    let Ok((ship_orbit, _)) = ships.get(ship_e) else {
        info!("intercept: focused entity {ship_e:?} is not a ship");
        return;
    };

    // copy out so the ship/body query borrows can end before we push burns
    let ship_parent = ship_orbit.parent;
    let ship_el = ship_orbit.elements;

    // Resolve (escape, plan, r_p). The target is either an orbiting body (rendezvous + capture
    // into its SOI) or a root body (Hohmann-style transfer into a circular orbit about it).
    let (escape, plan, r_p) = if let Ok((_, target_orbit, target_body, _)) = bodies.get(click.entity) {
        let target_el = target_orbit.elements;
        let mu_target = target_body.mu;
        // target orbits its parent (the shared frame G) with mu = target_el.mu
        let r_soi = soi_radius(target_el.a, mu_target, target_el.mu);
        // arrive on the target's operational shell (clamped to stay inside its SOI)
        let r_p = debug.capture_rp.unwrap_or(shell_radius(target_body).min(0.9 * r_soi));

        // Same parent means plan directly. Otherwise, if the target sits in the ship's
        // grandparent frame escape the current parent first.
        let (escape, plan) = if ship_parent == target_orbit.parent {
            let Some(plan) = plan_mission(&ship_el, &target_el, mu_target, clock.t, r_p, &[], 0.0) else {
                info!("no mission found");
                return;
            };
            (None, plan)
        } else {
            let Ok((_, p_orbit, p_body, _)) = bodies.get(ship_parent) else {
                info!("intercept: ship's parent {ship_parent:?} is not an orbiting body");
                return;
            };
            if target_orbit.parent != p_orbit.parent {
                info!("intercept: target is in neither the ship's parent nor grandparent frame (multi-leg unsupported)");
                return;
            }
            let r_soi_p = soi_radius(p_orbit.elements.a, p_body.mu, p_orbit.elements.mu);
            let Some((escape, ship_g, t_exit)) = plan_escape(&ship_el, &p_orbit.elements, r_soi_p, clock.t) else {
                info!("intercept: could not plan an escape from the parent SOI");
                return;
            };
            let Some(plan) = plan_mission(&ship_g, &target_el, mu_target, t_exit, r_p, &[], 0.0) else {
                info!("intercept: no transfer found after escape");
                return;
            };
            (Some(escape), plan)
        };
        (escape, plan, r_p)
    } else if let Ok((root_body, _)) = roots.get(click.entity) {
        // root target: transfer into a circular orbit about the central body
        let mu_root = root_body.mu;
        // arrive on the central body's operational shell (same shell move-to uses)
        let r_p = debug.capture_rp.unwrap_or(shell_radius(root_body));
        let primary_floor = root_body.radius;

        // every other body orbiting this root is an obstacle to screen against (skip the moon
        // the ship is leaving, if any — it starts inside that one)
        let siblings: Vec<(OrbitalElements, f64)> = bodies
            .iter()
            .filter(|(e, o, _, _)| o.parent == click.entity && *e != ship_parent)
            .map(|(_, o, b, _)| (o.elements, soi_radius(o.elements.a, b.mu, o.elements.mu)))
            .collect();

        if ship_parent == click.entity {
            // scenario A: ship already orbits the root → reshape into the circular orbit
            let Some(plan) = plan_root_capture(&ship_el, mu_root, r_p, clock.t, &siblings, primary_floor) else {
                info!("no mission found");
                return;
            };
            (None, plan, r_p)
        } else {
            // scenario B: ship orbits a moon of the root → escape that moon, then circularize
            let Ok((_, p_orbit, p_body, _)) = bodies.get(ship_parent) else {
                info!("intercept: ship's parent {ship_parent:?} is not an orbiting body");
                return;
            };
            if p_orbit.parent != click.entity {
                info!("intercept: root is neither the ship's parent nor grandparent (multi-leg unsupported)");
                return;
            }
            let r_soi_p = soi_radius(p_orbit.elements.a, p_body.mu, p_orbit.elements.mu);
            let Some((escape, ship_g, t_exit)) = plan_escape(&ship_el, &p_orbit.elements, r_soi_p, clock.t) else {
                info!("intercept: could not plan an escape from the parent SOI");
                return;
            };
            let Some(plan) = plan_root_capture(&ship_g, mu_root, r_p, t_exit, &siblings, primary_floor) else {
                info!("intercept: no transfer found after escape");
                return;
            };
            (Some(escape), plan, r_p)
        }
    } else {
        info!("intercept: clicked {:?} is not a targetable body or root", click.entity);
        return;
    };

    let t_dep = plan.departure.execute_at;
    {
        let mut man = ships.get_mut(ship_e).unwrap().1;
        if let Some(escape) = escape {
            man.queue.push_back(escape); // leave the parent SOI now; update_soi re-parents at the crossing
        }
        man.queue.push_back(plan.departure);
    }

    commands.entity(ship_e).insert(ScheduledCapture {
        execute_at: plan.t_peri,
        parent: click.entity,
        elements: plan.circular,
    });
    info!("mission planned: capture at t = {:.0}", plan.t_peri);

    debug.last_mission = Some(MissionReadout {
        t_dep,
        wait: t_dep - clock.t,
        t_peri: plan.t_peri,
        v_inf: plan.v_inf,
        dep_dv: plan.dep_dv,
        capture_dv: plan.capture_cost,
        total_dv: plan.dep_dv + plan.capture_cost,
        r_p,
        captured_a: plan.circular.a,
        captured_e: plan.circular.e,
    });
}

fn apply_focus_request(mut ui: ResMut<DebugUi>, mut cam: Single<&mut OrbitCam, With<Camera>>,) {
    let Some(e) = ui.focus_request.take() else { return; };  // consume once
    cam.focus = e;
}
