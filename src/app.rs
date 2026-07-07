// Application assembly: the App is split into cohesive Bevy Plugins, and the
// gameplay pipeline ordering lives in one ordered GameSet chain.

use bevy::{math::DQuat, prelude::*};
use bevy_egui::{EguiPlugin, EguiPrimaryContextPass};

use crate::camera::{OrbitCam, focus_on_click, orbit_camera};
use crate::debug::{DebugUi, debug_cam_inactive, debug_camera, debug_is_open};
use crate::edit::{HandleTarget, drag_handle, position_handles, run_handle_target, spawn_handles};
use crate::game_state::{
    AppMode, GameState, despawn_world, load_scene, not_menu, save_scene, toggle_mode,
};
use crate::log_capture::{LogWindow, log_panel};
use crate::menu::{pause_menu, start_screen};
use crate::ship_control::{
    MoveOrderEvent, TransferOrderEvent, apply_move_orders, apply_transfer_orders, queue_ship_orders,
};
use crate::sim::orbit::{OrbitPropagationCache, draw_orbits, execute_maneuvers, propagate_orbits};
use crate::sim::{
    broadphase::{FrameSpaceCache, build_frame_cache},
    capture::execute_capture,
    clock::{SimClock, advance_clock, clamp_warp, warp_keys},
    integrate::{PhysAccumulator, integrate_powered},
    soi::{draw_soi, update_soi},
};
use crate::world_pos::WorldPos;
use crate::worlds::CurrentWorld;

// Ordered gameplay pipeline. Systems keep their own run conditions
#[derive(SystemSet, Clone, Copy, Debug, PartialEq, Eq, Hash)]
enum GameSet {
    Time,       // warp input + clock advance
    Orders,     // input orders translated into sim actions
    Maneuver,   // impulsive burns
    FrameCache, // rebuild the sibling cache for frame-local lookups
    Soi,        // sphere-of-influence transitions
    Capture,    // scheduled orbital insertions
    Integrate,  // numerical integration of powered craft
    Propagate,  // analytic positions from orbits (Run AND Edit)
    Camera,     // camera follow + focus
    Draw,       // debug gizmos (orbits, SOI)
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
            .init_resource::<OrbitPropagationCache>()
            .init_resource::<FrameSpaceCache>()
            // the ordered gameplay pipeline; members carry their own run conditions
            .configure_sets(
                Update,
                (
                    GameSet::Time,
                    GameSet::Orders,
                    GameSet::Maneuver,
                    GameSet::FrameCache,
                    GameSet::Soi,
                    GameSet::Capture,
                    GameSet::Integrate,
                    GameSet::Propagate,
                    GameSet::Camera,
                    GameSet::Draw,
                )
                    .chain(),
            )
            // gameplay mutation only happens while the simulation is actively running
            .add_systems(
                Update,
                (
                    warp_keys.in_set(GameSet::Time),
                    clamp_warp.in_set(GameSet::Time).before(advance_clock),
                    advance_clock.in_set(GameSet::Time),
                    execute_maneuvers.in_set(GameSet::Maneuver),
                    build_frame_cache.in_set(GameSet::FrameCache),
                    update_soi.in_set(GameSet::Soi),
                    execute_capture.in_set(GameSet::Capture),
                    integrate_powered.in_set(GameSet::Integrate),
                )
                    .run_if(in_state(GameState::Running)),
            )
            // positions are still propagated while paused/editing so inspection stays correct
            .add_systems(
                Update,
                propagate_orbits.in_set(GameSet::Propagate).run_if(not_menu),
            )
            // draw orbits and soi helper gizmos (when the debug inspector is open)
            .add_systems(
                Update,
                (draw_orbits, draw_soi)
                    .chain()
                    .in_set(GameSet::Draw)
                    .run_if(debug_is_open)
                    .run_if(not_menu),
            );
    }
}

// ===== Camera, focus, and f64 -> f32 render sync =====
pub struct CameraPlugin;
impl Plugin for CameraPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, (setup, configure_gizmos))
            // focus requests are consumed just before the camera follows them
            // skipped while the free-flight debug camera has control of the entity
            .add_systems(
                Update,
                (apply_focus_request.before(orbit_camera), orbit_camera)
                    .in_set(GameSet::Camera)
                    .run_if(in_state(AppMode::Run))
                    .run_if(debug_cam_inactive),
            )
            // after all the position udpates, render it to the screen (any non-menu mode).
            // MUST run before transform propagation, or the GlobalTransform used for rendering
            // is one frame stale — which looks like the world lagging/skipping when the camera moves.
            .add_systems(
                PostUpdate,
                sync_render_space
                    .before(TransformSystems::Propagate)
                    .run_if(not_menu),
            )
            // primary click on Focusable bodies
            .add_observer(focus_on_click);
    }
}

// ===== Player ship commands (select + move-to-point) =====
pub struct ShipControlPlugin;
impl Plugin for ShipControlPlugin {
    fn build(&self, app: &mut App) {
        app.add_message::<MoveOrderEvent>()
            .add_message::<TransferOrderEvent>()
            // only run ship orders if game is running
            .add_systems(
                Update,
                queue_ship_orders
                    .in_set(GameSet::Orders)
                    .run_if(in_state(GameState::Running)),
            )
            // if game running apply move orders -> apply body transfer
            .add_systems(
                Update,
                (apply_move_orders, apply_transfer_orders)
                    .chain()
                    .in_set(GameSet::Orders)
                    .run_if(in_state(GameState::Running)),
            );
    }
}

// ===== Orbit-edit gizmo handles (both Run and Edit, self-gated via HandleTarget) =====
pub struct EditHandlePlugin;
impl Plugin for EditHandlePlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<HandleTarget>()
            .add_systems(Startup, spawn_handles)
            // Run path: drive the orbit-gizmo target from the debug selection
            .add_systems(Update, run_handle_target.run_if(in_state(AppMode::Run)))
            // orbit-edit handles: self-gated via HandleTarget, after whichever camera updates
            .add_systems(
                Update,
                position_handles.after(orbit_camera).after(debug_camera),
            )
            // drag edit handles to modify orbit
            .add_observer(drag_handle);
    }
}

// ===== egui: debug panel, log window, menus =====
pub struct UiPlugin;
impl Plugin for UiPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(EguiPlugin::default())
            // info!() mirror
            .init_resource::<LogWindow>()
            // input + GUI systems
            .add_systems(Update, toggle_mode.run_if(in_state(AppMode::Run)))
            .add_systems(
                EguiPrimaryContextPass,
                (
                    // the log is a debug tool: only while the F1 overlay is up, toggled from its toolbar
                    log_panel
                        .run_if(in_state(AppMode::Run))
                        .run_if(debug_is_open),
                    start_screen.run_if(in_state(AppMode::Menu)),
                    pause_menu.run_if(in_state(GameState::Paused)),
                ),
            );
    }
}

// summons light + camera only, the rest of loading is handed to
// the world load_scene system
fn setup(mut commands: Commands) {
    // sun shines parallel from far away
    commands.spawn((
        DirectionalLight {
            illuminance: 8000.0,
            shadows_enabled: false,
            ..default()
        },
        Transform::from_rotation(Quat::from_euler(EulerRot::XYZ, -0.7, 0.5, 0.0)),
    ));

    commands.spawn((
        Camera3d::default(),
        // lift the shadowed side so bodies aren't pure black away from the sun
        AmbientLight {
            brightness: 400.0,
            ..default()
        },
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
fn sync_render_space(
    camera: Single<&WorldPos, With<Camera>>,
    mut bodies: Query<(&WorldPos, &mut Transform)>,
) {
    let origin = **camera; // the camera's f64 position
    for (pos, mut transform) in &mut bodies {
        transform.translation = pos.to_render_space(origin); // subtract in f64, then cast
    }
}

fn apply_focus_request(mut ui: ResMut<DebugUi>, mut cam: Single<&mut OrbitCam, With<Camera>>) {
    let Some(e) = ui.focus_request.take() else {
        return;
    }; // consume once
    cam.focus = e;
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Instant;

    use bevy::ecs::system::RunSystemOnce;
    use bevy::math::DVec3;

    use crate::math::orbital_elements::OrbitalElements;
    use crate::sim::entity::{SimEntity, SimulationTier};
    use crate::sim::guidance::{Guidance, hold, move_to, station_keep};
    use crate::sim::integrate::{Propulsion, StateVec, ThrustCommand};
    use crate::sim::orbit::{Body, Maneuvers, Orbit};

    const ROOT_MU: f64 = 1.267e17;

    #[test]
    #[ignore]
    fn perf_report_mixed_entity_counts() {
        for &count in &[100_usize, 1_000, 5_000] {
            let mut world = perf_world(count);

            world.run_system_once(integrate_powered).unwrap();
            world.resource_mut::<SimClock>().t = 2.0;

            let frame_cache = timed(|| world.run_system_once(build_frame_cache).unwrap());
            let soi = timed(|| world.run_system_once(update_soi).unwrap());
            let guidance = timed(|| world.run_system_once(sample_guidance_only).unwrap());
            let integrate = timed(|| world.run_system_once(integrate_powered).unwrap());
            let propagate = timed(|| world.run_system_once(propagate_orbits).unwrap());
            let render = timed(|| world.run_system_once(sync_render_space).unwrap());

            println!(
                "perf n={count}: frame_cache={frame_cache:.2?} soi={soi:.2?} guidance={guidance:.2?} integrate={integrate:.2?} propagate={propagate:.2?} render_sync={render:.2?}"
            );
        }
    }

    fn timed(run: impl FnOnce()) -> std::time::Duration {
        let start = Instant::now();
        run();
        start.elapsed()
    }

    fn perf_world(ship_count: usize) -> World {
        let mut world = World::new();
        world.insert_resource(SimClock::default());
        world.insert_resource(PhysAccumulator::default());
        world.insert_resource(OrbitPropagationCache::default());
        world.insert_resource(FrameSpaceCache::default());

        world.spawn((Camera::default(), Transform::default(), WorldPos::ORIGIN));

        let root = world
            .spawn((
                SimEntity,
                SimulationTier::Rendered,
                WorldPos::ORIGIN,
                Transform::default(),
                Body {
                    mu: ROOT_MU,
                    radius: 7.0e6,
                },
            ))
            .id();

        let mut parents = vec![root];
        for i in 0..8 {
            let orbit = Orbit {
                elements: OrbitalElements {
                    a: 4.0e7 + i as f64 * 8.0e6,
                    e: 0.0,
                    i: 0.0,
                    lan: 0.0,
                    arg_pe: 0.0,
                    m0: i as f64 * 0.2,
                    epoch: 0.0,
                    mu: ROOT_MU,
                },
                parent: root,
            };
            let body = world
                .spawn((
                    SimEntity,
                    SimulationTier::Rendered,
                    WorldPos::ORIGIN,
                    Transform::default(),
                    orbit,
                    Body {
                        mu: 5.0e13 + i as f64 * 1.0e12,
                        radius: 1.5e6,
                    },
                ))
                .id();
            parents.push(body);
        }

        for i in 0..ship_count {
            let parent = parents[i % parents.len()];
            let tier = match i % 5 {
                0 => SimulationTier::Background,
                1 => SimulationTier::Local,
                _ => SimulationTier::Rendered,
            };
            let mut entity = world.spawn((
                SimEntity,
                tier,
                WorldPos::ORIGIN,
                Maneuvers::default(),
                Propulsion {
                    max_accel: 25.0,
                    throttle: 1.0,
                },
            ));
            if tier != SimulationTier::Background {
                entity.insert(Transform::default());
            }
            if i % 3 == 0 {
                entity.insert((
                    StateVec {
                        pos: DVec3::new(1.8e7 + i as f64 * 10.0, 0.0, 0.0),
                        vel: DVec3::new(0.0, (ROOT_MU / 1.8e7).sqrt() * 0.98, 0.0),
                        frame: parent,
                    },
                    Guidance::MoveTo {
                        target: DVec3::new(0.0, 1.8e7, 0.0),
                    },
                    ThrustCommand::default(),
                ));
            } else {
                entity.insert(Orbit {
                    elements: OrbitalElements {
                        a: 2.0e7 + (i % 64) as f64 * 1.2e5,
                        e: 0.0,
                        i: 0.0,
                        lan: 0.0,
                        arg_pe: 0.0,
                        m0: i as f64 * 0.01,
                        epoch: 0.0,
                        mu: ROOT_MU,
                    },
                    parent,
                });
            }
        }

        world
    }

    fn sample_guidance_only(
        bodies: Query<&Body>,
        mut ships: Query<(&StateVec, &Guidance, &Propulsion, &mut ThrustCommand)>,
    ) {
        for (sv, guidance, propulsion, mut cmd) in &mut ships {
            let Ok(body) = bodies.get(sv.frame) else {
                cmd.accel = DVec3::ZERO;
                continue;
            };
            let mu = body.mu;
            let limit = propulsion.max_accel * propulsion.throttle.clamp(0.0, 1.0);
            cmd.accel = match *guidance {
                Guidance::Idle => DVec3::ZERO,
                Guidance::Hold => hold(sv.pos, sv.vel, mu),
                Guidance::StationKeep { radius } => station_keep(sv.pos, sv.vel, mu, radius),
                Guidance::MoveTo { target } => move_to(sv.pos, sv.vel, mu, target, limit),
                Guidance::Seek { .. } => cmd.accel,
            };
        }
    }
}
