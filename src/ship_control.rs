// Player ship commands. Right-clicking issues either a move order on the current shell
// or a transfer-planning order against another body under the cursor.

use bevy::math::DVec3;
use bevy::prelude::*;
use bevy::ecs::message::{MessageReader, MessageWriter};
use bevy_egui::input::EguiWantsInput;

use crate::debug::{DebugUi, MissionReadout};
use crate::math::orbital_elements::OrbitalElements;
use crate::sim::capture::ScheduledCapture;
use crate::sim::clock::SimClock;
use crate::sim::integrate::{StateVec, ThrustCommand};
use crate::sim::mission::{plan_escape, plan_mission, plan_root_capture};
use crate::sim::orbit::{shell_radius, Body, Maneuvers, Orbit, OrbitPropagationCache};
use crate::sim::soi::soi_radius;
use crate::sim::guidance::Guidance;
use crate::world_pos::WorldPos;

// event for moving to position within shell 
#[derive(Message, Clone, Copy, Debug)]
pub struct MoveOrderEvent {
    pub ship: Entity,
    pub target: DVec3,
}

// event for changing orbits 
#[derive(Message, Clone, Copy, Debug)]
pub struct TransferOrderEvent {
    pub ship: Entity,
    pub target: Entity,
    pub capture_rp: Option<f64>,
}

#[allow(clippy::too_many_arguments)]
pub fn queue_ship_orders(
    mouse: Res<ButtonInput<MouseButton>>,
    egui_wants: Res<EguiWantsInput>,
    debug: Res<DebugUi>,
    window: Single<&Window>,
    camera: Single<(&Camera, &GlobalTransform, &WorldPos), With<Camera>>,
    coast: Query<&Orbit, With<Maneuvers>>,
    powered: Query<&StateVec, With<Maneuvers>>,
    bodies: Query<(Entity, &WorldPos, &Body)>,
    mut move_orders: MessageWriter<MoveOrderEvent>,
    mut transfer_orders: MessageWriter<TransferOrderEvent>,
) {
    if !mouse.just_pressed(MouseButton::Right) || egui_wants.wants_any_pointer_input() {
        return;
    }

    let Some(ship) = debug.selected else { return };

    let frame = if let Ok(orbit) = coast.get(ship) {
        orbit.parent
    } else if let Ok(state) = powered.get(ship) {
        state.frame
    } else {
        return;
    };

    let Ok((_, frame_wp, frame_body)) = bodies.get(frame) else { return };
    let shell = shell_radius(frame_body);

    let (cam, cam_tf, cam_wp) = *camera;
    let Some(cursor) = window.cursor_position() else { return };
    let Ok(ray) = cam.viewport_to_world(cam_tf, cursor) else { return };

    let mut nearest: Option<(f32, Entity)> = None;
    for (entity, wp, body) in &bodies {
        if let Some(t) = ray_sphere_enter(ray, wp.to_render_space(*cam_wp), body.radius as f32)
            && nearest.is_none_or(|(best_t, _)| t < best_t)
        {
            nearest = Some((t, entity));
        }
    }

    if let Some((_, hit)) = nearest && hit != frame {
        if coast.get(ship).is_ok() {
            transfer_orders.write(TransferOrderEvent {
                ship,
                target: hit,
                capture_rp: debug.capture_rp,
            });
        }
        return;
    }

    let center = frame_wp.to_render_space(*cam_wp);
    let Some(hit) = ray_sphere(ray, center, shell as f32) else { return };
    let target = (hit - center).as_dvec3();
    move_orders.write(MoveOrderEvent { ship, target });
}

// move to specific place 
pub fn apply_move_orders(
    mut orders: MessageReader<MoveOrderEvent>,
    clock: Res<SimClock>,
    mut orbit_cache: ResMut<OrbitPropagationCache>,
    coast: Query<&Orbit, With<Maneuvers>>,
    mut guidance_q: Query<&mut Guidance>,
    mut commands: Commands,
) {
    for order in orders.read() {
        if let Ok(orbit) = coast.get(order.ship) {
            commands.entity(order.ship).remove::<Orbit>().insert((
                StateVec::from_orbit(orbit, clock.t), // transition to velocity 
                ThrustCommand::default(),
                Guidance::MoveTo { target: order.target }, // give orders 
            ));
            orbit_cache.dirty = true; // mark as seen 
        } else if let Ok(mut guidance) = guidance_q.get_mut(order.ship) {
            *guidance = Guidance::MoveTo {
                target: order.target,
            };
        }
    }
}

// move from one body to another 
pub fn apply_transfer_orders(
    mut orders: MessageReader<TransferOrderEvent>,
    mut debug: ResMut<DebugUi>,
    clock: Res<SimClock>,
    mut commands: Commands,
    mut ships: Query<(&Orbit, &mut Maneuvers)>,
    bodies: Query<(Entity, &Orbit, &Body), Without<Maneuvers>>,
    roots: Query<&Body, (Without<Orbit>, Without<Maneuvers>)>,
) {
    for order in orders.read() {
        let Ok((ship_orbit, _)) = ships.get(order.ship) else {
            info!("transfer: selected ship {:?} is not currently coasting", order.ship);
            continue;
        };

        let ship_parent = ship_orbit.parent;
        let ship_el = ship_orbit.elements;

        let (escape, plan, r_p) = if let Ok((_, target_orbit, target_body)) = bodies.get(order.target) {
            let target_el = target_orbit.elements;
            let mu_target = target_body.mu;
            let r_soi = soi_radius(target_el.a, mu_target, target_el.mu);
            // goal distanct to move from body 
            let r_p = order.capture_rp.unwrap_or(shell_radius(target_body).min(0.9 * r_soi));

            // ship and target share a parent
            let (escape, plan) = if ship_parent == target_orbit.parent {
                let Some(plan) = plan_mission(&ship_el, &target_el, mu_target, clock.t, r_p, &[], 0.0) else {
                    info!("transfer: no mission found");
                    continue;
                };
                (None, plan)
            } else {
                let Ok((_, parent_orbit, parent_body)) = bodies.get(ship_parent) else {
                    info!("transfer: ship parent {:?} is not an orbiting body", ship_parent);
                    continue;
                };
                // parent is not in shared parent or grandparent relation ie IO -> mercury 
                if target_orbit.parent != parent_orbit.parent {
                    info!("transfer: multi-leg planning is not supported yet");
                    continue;
                }
                let r_soi_parent = soi_radius(parent_orbit.elements.a, parent_body.mu, parent_orbit.elements.mu);
                let Some((escape, ship_grandparent, t_exit)) =
                    plan_escape(&ship_el, &parent_orbit.elements, r_soi_parent, clock.t)
                else {
                    info!("transfer: could not plan escape from parent SOI");
                    continue;
                };
                let Some(plan) =
                    plan_mission(&ship_grandparent, &target_el, mu_target, t_exit, r_p, &[], 0.0)
                else {
                    info!("transfer: no transfer found after escape");
                    continue;
                };
                (Some(escape), plan)
            };
            (escape, plan, r_p)
        // what if we are movung to the root? 
        } else if let Ok(root_body) = roots.get(order.target) {
            let mu_root = root_body.mu;
            let r_p = order.capture_rp.unwrap_or(shell_radius(root_body));
            let primary_floor = root_body.radius;

            let siblings: Vec<(OrbitalElements, f64)> = bodies
                .iter()
                .filter(|(entity, orbit, _)| orbit.parent == order.target && *entity != ship_parent)
                .map(|(_, orbit, body)| {
                    (orbit.elements, soi_radius(orbit.elements.a, body.mu, orbit.elements.mu))
                })
                .collect();

            if ship_parent == order.target {
                let Some(plan) = plan_root_capture(&ship_el, mu_root, r_p, clock.t, &siblings, primary_floor) else {
                    info!("transfer: no root-capture mission found");
                    continue;
                };
                (None, plan, r_p)
            } else {
                let Ok((_, parent_orbit, parent_body)) = bodies.get(ship_parent) else {
                    info!("transfer: ship parent {:?} is not an orbiting body", ship_parent);
                    continue;
                };
                if parent_orbit.parent != order.target {
                    info!("transfer: multi-leg root capture is not supported yet");
                    continue;
                }
                let r_soi_parent = soi_radius(parent_orbit.elements.a, parent_body.mu, parent_orbit.elements.mu);
                let Some((escape, ship_grandparent, t_exit)) =
                    plan_escape(&ship_el, &parent_orbit.elements, r_soi_parent, clock.t)
                else {
                    info!("transfer: could not plan escape from the parent SOI");
                    continue;
                };
                let Some(plan) =
                    plan_root_capture(&ship_grandparent, mu_root, r_p, t_exit, &siblings, primary_floor)
                else {
                    info!("transfer: no root capture found after escape");
                    continue;
                };
                (Some(escape), plan, r_p)
            }
        } else {
            info!("transfer: target {:?} is not a body", order.target);
            continue;
        };

        let t_dep = plan.departure.execute_at;
        {
            let mut maneuvers = ships.get_mut(order.ship).unwrap().1;
            if let Some(escape) = escape {
                maneuvers.queue.push_back(escape);
            }
            maneuvers.queue.push_back(plan.departure);
        }

        commands.entity(order.ship).insert(ScheduledCapture {
            execute_at: plan.t_peri,
            parent: order.target,
            elements: plan.circular,
        });
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
        info!("mission planned: capture at t = {:.0}", plan.t_peri);
    }
}

// Nearest forward hit of the ray with the sphere; on a miss, the silhouette point closest
// to the ray (so a click in empty space still yields a point on the shell).
fn ray_sphere(ray: Ray3d, center: Vec3, radius: f32) -> Option<Vec3> {
    let oc = ray.origin - center;
    let b = oc.dot(*ray.direction);
    let c = oc.length_squared() - radius * radius;
    let disc = b * b - c;
    if disc >= 0.0 {
        let s = disc.sqrt();
        let t = if -b - s > 0.0 { -b - s } else { -b + s };
        if t > 0.0 {
            return Some(ray.origin + *ray.direction * t);
        }
    }

    let closest = ray.origin + *ray.direction * b.max(0.0);
    let dir = (closest - center).normalize_or_zero();
    (dir != Vec3::ZERO).then(|| center + dir * radius)
}

// Distance to the nearest forward intersection of the ray with the sphere, or None if the
// ray misses or the sphere lies entirely behind the camera.
fn ray_sphere_enter(ray: Ray3d, center: Vec3, radius: f32) -> Option<f32> {
    let oc = ray.origin - center;
    let b = oc.dot(*ray.direction);
    let c = oc.length_squared() - radius * radius;
    let disc = b * b - c;
    if disc < 0.0 {
        return None;
    }
    let s = disc.sqrt();
    let (t_near, t_far) = (-b - s, -b + s);
    if t_near > 0.0 {
        Some(t_near)
    } else if t_far > 0.0 {
        Some(t_far)
    } else {
        None
    }
}
