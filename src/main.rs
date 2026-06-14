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
use edit::{spawn_handles, position_handles, drag_handle};
use log_capture::{capture_layer, LogWindow, toggle_log_window, log_panel};
use sim::orbit::{Orbit, Maneuvers, Burn, Body, propagate_orbits, draw_orbits, execute_maneuvers};
use sim::{
    soi::{draw_soi, update_soi, soi_radius},
    clock::{SimClock, warp_keys, advance_clock},
    mission::plan_mission,
    capture::{ScheduledCapture, execute_capture},
};
use world_pos::WorldPos;
use body_traits::Focusable;
use bevy::{log::LogPlugin, math::DQuat, prelude::*};
use debug_ui::{DebugUi, MissionReadout, toggle_debug_ui, debug_panel, debug_is_open};
use bevy_egui::{EguiPlugin, EguiPrimaryContextPass, input::EguiWantsInput};
use game_state::{AppMode, GameState, not_menu, load_scene, save_scene, toggle_mode, despawn_world};
use menu::{start_screen, pause_menu};
use worlds::CurrentWorld;
use editor::{
    CurrentLevel, EditorCamera, EditorSaveRequest,
    editor_setup, editor_teardown, editor_time, save_level, fly_camera, editor_panel,
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
               propagate_orbits,                                      // update orbit positions
               orbit_camera,                                          // update camera
            ).chain().run_if(in_state(AppMode::Run)))
       // editor: free camera + time stepping + save (Edit only)
       .add_systems(Update, (fly_camera, editor_time, save_level).run_if(in_state(AppMode::Edit)))
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
               position_handles.after(orbit_camera).run_if(in_state(AppMode::Run)),
            ))
       .add_systems(EguiPrimaryContextPass, (
               debug_panel.run_if(in_state(AppMode::Run)),
               log_panel,
               start_screen.run_if(in_state(AppMode::Menu)),
               pause_menu.run_if(in_state(GameState::Paused)),
               editor_panel.run_if(in_state(AppMode::Edit)),
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
    bodies: Query<(&Orbit, &Body, &Focusable), Without<Maneuvers>>,
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

    // TODO, the ship needs to be able to travel back to the root

    // make sure the selected ship is indeed a ship
    let Ok((ship_orbit, _)) = ships.get(ship_e) else { 
        info!("intercept: focused entity {ship_e:?} is not a ship");
        return; 
    };

    // make sure what is clicked can be targeted 
    let Ok((target_orbit, target_body, _)) = bodies.get(click.entity) else {
        info!("intercept: clicked {:?} is not a targetable body (root/ship/occluder?)", click.entity);
        return;
    };
    
    // check if ship is returning to home body (in reference) or moving in reference frame 
    if (target_orbit.parent != ship_orbit.parent) && (click.entity != ship_orbit.parent)  { 
        info!("The parent of the target ({:?}) is not the target of the ship ({:?})",
            target_orbit.parent, ship_orbit.parent);
        return; 
    }

    let ship_el = ship_orbit.elements;
    let target_el = target_orbit.elements;
    let mu_target = target_body.mu;
    let r_soi = soi_radius(target_el.a, mu_target, ship_el.mu);
    let r_p = debug.capture_rp.unwrap_or((target_body.radius * 1.2).min(0.9 * r_soi));

    let Some(plan) = plan_mission(&ship_el, &target_el, mu_target, clock.t, r_p, &[], 0.0) else {
        info!("no mission found");
        return;
    };

    let t_dep = plan.departure.execute_at;
    ships.get_mut(ship_e).unwrap().1.queue.push_back(plan.departure);
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
