// WorldPos it the basic type system
// For absolute coordinates f64 is used as well as for translation
// WorldPos is converted to a vector of f32 for camera rendering
// The accuracy of math is kept high and only for the camera do we loose accuracy
//
// Since f64 has 11 bits for exponent and 52 for sigfig.,
// if we want to be precise down to .01, this leaves
// 45 bits of sig-fig for distance or else  2^45 ~ 1.75e13.
// this is 17.5 billion km
//
// The planetary size of our solar system is only about 9 billion km
// so this is well enough accuracy for planetary shenanigans.

use bevy::math::DVec3;
use bevy::prelude::*;
use serde::{Deserialize, Serialize};

// include component for ECS (enttiy, component, system)
#[derive(Component, Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct WorldPos(pub DVec3);

#[allow(dead_code)] // helpers for test 
impl WorldPos {
    pub const ORIGIN: Self = WorldPos(DVec3::ZERO);

    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self(DVec3::new(x, y, z))
    }

    fn translate(self, delta: DVec3) -> Self {
        Self(self.0 + delta)
    }

    // displacement from self to other
    fn delta_to(self, other: Self) -> DVec3 {
        other.0 - self.0
    }

    // where to draw this relative to the camera
    // Needs f32 for rendering
    pub fn to_render_space(self, camera: WorldPos) -> Vec3 {
        (self.0 - camera.0).as_vec3()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const FAR: f64 = 4.5e12;
    const MAX_DIST: f64 = 1.05e13; // farthest gameplay point (~Pluto–Uranus span)
    const RESOLVE: f64 = 1.0; // smallest move we still want to see
    const TOLERANCE: f32 = 0.01; // how far off the rendered offset may be (1 cm)

    #[test]
    fn translate_and_delta_are_exact_near_origin() {
        let a = WorldPos::ORIGIN;
        let b = a.translate(DVec3::new(10.0, -5.0, 2.0));

        assert_eq!(b.0, DVec3::new(10.0, -5.0, 2.0));
        assert_eq!(a.delta_to(b), DVec3::new(10.0, -5.0, 2.0));
        assert_eq!(b.delta_to(a), DVec3::new(-10.0, 5.0, -2.0));
        assert_eq!(b.delta_to(b), DVec3::ZERO);
    }

    #[test]
    fn camera_at_origin_is_identity_downcast() {
        let p = WorldPos::new(123.0, 456.0, 789.0);
        let r = p.to_render_space(WorldPos::ORIGIN);
        assert_eq!(r, Vec3::new(123.0, 456.0, 789.0));
    }

    #[test]
    fn render_space_preserves_meters_at_solar_scale() {
        let body_a = WorldPos::new(FAR, FAR, FAR);
        let body_b = body_a.translate(DVec3::new(100.0, 0.0, 0.0));
        let r = body_b.to_render_space(body_a);

        let err = (r - Vec3::new(100.0, 0.0, 0.0)).length();

        assert!(
            err < 0.01,
            "expected ~100m offset preserved, got {r:?} (err {err} m)"
        );
    }

    #[test]
    fn naive_absolute_f32_loses_the_seperation() {
        let body_a = DVec3::new(FAR, FAR, FAR);
        let body_b = body_a + DVec3::new(100.0, 0.0, 0.0);

        let naive = body_b.as_vec3() - body_a.as_vec3();

        let err = (naive - Vec3::new(100.0, 0.0, 0.0)).length();
        assert!(err > 1.0, "naive f32 was unexpectedly accurate ({naive:?})");
    }

    #[test]
    fn determinism_same_input_as_output() {
        let cam = WorldPos::new(FAR, -FAR, FAR);
        let p = WorldPos::new(FAR + 250.0, -FAR + 10.0, FAR - 5.0);
        assert_eq!(p.to_render_space(cam), p.to_render_space(cam));
    }

    #[test]
    fn translate_is_commutative_and_associative_in_f64() {
        let p = WorldPos::new(FAR, 0.0, 0.0);
        let d1 = DVec3::new(3.0, 4.0, 0.0);
        let d2 = DVec3::new(-1.0, 2.0, 7.0);

        assert_eq!(p.translate(d1).translate(d2), p.translate(d2).translate(d1));
        assert_eq!(p.translate(d1).translate(d2), p.translate(d1 + d2));
    }

    #[test]
    fn delta_round_trips_through_translate() {
        let a = WorldPos::new(FAR, FAR * 0.5, -FAR);
        let b = WorldPos::new(FAR + 42.0, FAR * 0.5 - 13.0, -FAR + 9.0);
        assert_eq!(a.translate(a.delta_to(b)), b);
    }

    #[test]
    fn a_meter_scale_move_survives_at_world_edge() {
        let cam = WorldPos::new(MAX_DIST, MAX_DIST, MAX_DIST);
        let target = cam.translate(DVec3::new(RESOLVE, 0.0, 0.0));
        let r = target.to_render_space(cam);
        let err = (r - Vec3::new(RESOLVE as f32, 0.0, 0.0)).length();

        assert!(
            err < TOLERANCE,
            "a {RESOLVE} m move at {MAX_DIST:e} m came back off by {err} m (budget {TOLERANCE} m) — \
             MAX_DIST has outgrown f64"
        );
    }

    #[test]
    fn precision_floor_at_world_edge() {
        let cam = WorldPos::new(MAX_DIST, MAX_DIST, MAX_DIST);
        // f64 grid spacing here is 2^-9 m ≈ 1.95 mm. Below it, a move literally cannot exist.
        let sub_grid = cam.translate(DVec3::new(0.0005, 0.0, 0.0)); // 0.5 mm < spacing
        assert_eq!(
            sub_grid.to_render_space(cam),
            Vec3::ZERO,
            "a sub-grid move does not vanish entirely, unexpected precision"
        );
        // Comfortably above the spacing, the move is preserved
        let above = cam.translate(DVec3::new(0.05, 0.0, 0.0)); // 50 mm move 
        let off = (above.to_render_space(cam).x - 0.05).abs();
        assert!(
            off < 0.002,
            "above-grid move drifted {off} m (spacing ≈ 0.002 m)"
        );
    }
}
