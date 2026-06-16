mod world_pos;
mod math;
mod sim;
mod camera;
mod body_traits;
mod debug_ui;
mod edit;
mod log_capture;
mod game_state;
mod worlds;
mod menu;
mod editor;

use camera::{OrbitCam, orbit_camera, focus_on_click};
use edit::{spawn_handles, position_handles, drag_handle, run_handle_target, HandleTarget};
use log_capture::{capture_layer, LogWindow, toggle_log_window, log_panel};
use sim::orbit::{Orbit, Maneuvers, Burn, Body, propagate_orbits, draw_orbits, execute_maneuvers};
use sim::{
    soi::{draw_soi, update_soi, soi_radius},
    clock::{SimClock, warp_keys, advance_clock},
    mission::{plan_mission, plan_escape, plan_root_capture},
    capture::{ScheduledCapture, execute_capture},
};
use math::orbital_elements::OrbitalElements;
use world_pos::WorldPos;
use body_traits::Focusable;
use bevy::{log::LogPlugin, math::DQuat, prelude::*};
use debug_ui::{DebugUi, MissionReadout, toggle_debug_ui, debug_panel, debug_is_open};
use bevy_egui::{EguiPlugin, EguiPrimaryContextPass, input::EguiWantsInput};
use game_state::{AppMode, GameState, not_menu, load_scene, save_scene, toggle_mode, despawn_world};
use menu::{start_screen, pause_menu};
use worlds::CurrentWorld;
use editor::{
    CurrentLevel, EditorCamera, EditorSaveRequest, EditorSelection, EditorFocus,
    EditorWindows, EditorSpawnForm,
    editor_setup, editor_teardown, editor_time, save_level, fly_camera, editor_ui,
    editor_handle_target, draw_reference_axes, apply_editor_focus,
};


fn main() {
   App::new()
       .add_plugins(DefaultPlugins
           .set(WindowPlugin { primary_window: Some(Window { title: "Orbital".into(), ..default() }),..default() })
           .set(LogPlugin { custom_layer: capture_layer, ..default() }))
       .add_plugins(MeshPickingPlugin)
       .add_plugins(EguiPlugin::default())
       // F1 in the game 
       .init_resource::<DebugUi>()
       // info!() mirror 
       .init_resource::<LogWindow>()
       // time keeping resource 
       .init_resource::<SimClock>()
       // resources for current file being used 
       .init_resource::<CurrentWorld>()
       .init_resource::<CurrentLevel>()
       // non azimuthal camera: free from
       .init_resource::<EditorCamera>()
       .init_resource::<EditorSaveRequest>()
       // editor ui state + the shared orbit-gizmo target
       .init_resource::<EditorSelection>()
       .init_resource::<EditorFocus>()
       .init_resource::<EditorWindows>()
       .init_resource::<EditorSpawnForm>()
       .init_resource::<HandleTarget>()
       // while running states
       .init_state::<AppMode>()
       // Menu / Running / Editting 
       .add_sub_state::<GameState>()
       // run (world) lifecycle
       .add_systems(OnExit(AppMode::Run), despawn_world)
       .add_systems(OnEnter(GameState::Loading), load_scene)
       .add_systems(OnEnter(GameState::Saving), save_scene)
       // edit (level editor) lifecycle
       .add_systems(OnEnter(AppMode::Edit), editor_setup)
       .add_systems(OnExit(AppMode::Edit), editor_teardown)
       .add_systems(Startup, (setup, spawn_handles))
       // gameplay: runs only inside a loaded world
       .add_systems(Update, (
               warp_keys,                                             // time change settings
               advance_clock.run_if(in_state(GameState::Running)),    // change the time
               debug_burn_key,                                        // Custom burns
               execute_maneuvers,                                     // regular burns
               update_soi,                                            // update spheres of influence
               execute_capture,                                       // capture bodies in soi
               orbit_camera,                                          // update camera
            ).chain().run_if(in_state(AppMode::Run)))
       // positions come from elements + clock; needed in Run AND Edit (editor bodies move too)
       .add_systems(Update, propagate_orbits
            .after(execute_capture)
            .before(orbit_camera)
            .run_if(not_menu))
       // editor: free camera + time stepping + save + axes/focus/handle-target (Edit only)
       .add_systems(Update, (
               fly_camera, editor_time, save_level,
               editor_handle_target, draw_reference_axes, apply_editor_focus,
            ).run_if(in_state(AppMode::Edit)))
       // Run path: drive the orbit-gizmo target from the debug selection
       .add_systems(Update, run_handle_target.run_if(in_state(AppMode::Run)))
       // orbit-edit handles: both modes (self-gated via HandleTarget), after either camera updates
       .add_systems(Update, position_handles.after(orbit_camera).after(fly_camera))
       // draw orbits and soi helper gizmos (Run or Edit, when debug is open)
       .add_systems(Update, (draw_orbits, draw_soi).chain().after(orbit_camera).run_if(debug_is_open).run_if(not_menu))
       .add_systems(Update, apply_focus_request.before(orbit_camera).run_if(in_state(AppMode::Run)))
       // after all the position udpates, render it to the screen (any non-menu mode)
       .add_systems(PostUpdate, sync_render_space.run_if(not_menu))
       // mouse click observers
       .add_observer(focus_on_click)
       .add_observer(intercept_transfer)
       .add_observer(drag_handle)
       // input + GUI systems
       .add_systems(Update, (
               toggle_debug_ui,
               toggle_log_window,
               toggle_mode.run_if(in_state(AppMode::Run)),
            ))
       .add_systems(EguiPrimaryContextPass, (
               debug_panel.run_if(in_state(AppMode::Run)),
               log_panel,
               start_screen.run_if(in_state(AppMode::Menu)),
               pause_menu.run_if(in_state(GameState::Paused)),
               editor_ui.run_if(in_state(AppMode::Edit)),
            ))
       .run();
}

// summons light + camera only, the rest of loading is handed to 
// editor start / load screne systems 
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

// translate f64 math to f32 for rendering 
fn sync_render_space( camera: Single<&WorldPos, With<Camera>>, mut bodies: Query<(&WorldPos, &mut Transform)>, ) {
    let origin = **camera;                              // the camera's f64 position
    for (pos, mut transform) in &mut bodies {
        transform.translation = pos.to_render_space(origin); // subtract in f64, then cast
    }
}

// for testing 
fn debug_burn_key(
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
        let r_p = debug.capture_rp.unwrap_or((target_body.radius * 1.2).min(0.9 * r_soi));

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
        let r_p = debug.capture_rp.unwrap_or(root_body.radius * 2.0);
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
