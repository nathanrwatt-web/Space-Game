use std::f64::consts::PI;
use crate::sim::orbit::Burn;
use crate::math::orbital_elements::OrbitalElements;
use crate::math::lambert::lambert;

// constucts two burns for the hohmann orbit 
pub fn plan_hohmann(ship: &OrbitalElements, r2: f64, t_now: f64) -> (Burn, Burn) {
    let r1 = ship.a; // semi-major axis of the first orbit 
    let mu = ship.mu;// gravitational effet on the first orbit 

    let (dv1_magnitude, dv2_magnitude, t_transfer) = hohmann(r1, r2, mu);
    
    let departure_direction = ship.velocity_at(t_now).normalize();
    let dv1 = departure_direction * dv1_magnitude; 
    let transfer = ship.with_burn(t_now, dv1); // new orbit with the burn of dv1
 
    let t_arrival = t_now + t_transfer;
    let arrival_direction = transfer.velocity_at(t_arrival).normalize();
    let dv2 = arrival_direction * dv2_magnitude;

    (Burn { execute_at: t_now,      dv: dv1, }, 
     Burn { execute_at: t_arrival,  dv: dv2, },)
}

// Departure burn for intercept which meets the target body 
pub fn plan_lambert_intercept(
    ship: &OrbitalElements,
    target: &OrbitalElements,
    t_dep: f64,
    tof: f64,
) -> Option<Burn> {
    let mu = ship.mu;
    // start pos is now, end pos is later 
    let r1 = ship.offset_at(t_dep);
    let r2 = target.offset_at(t_dep + tof);

    let (v1, _v2) = lambert(r1, r2, tof, mu, true)?;
    let dv1 = v1 - ship.velocity_at(t_dep);
    Some(Burn { execute_at: t_dep, dv: dv1 })
}

pub fn hohmann_tof(r1: f64, r2: f64, mu: f64) -> f64 {
    hohmann(r1, r2, mu).2
}


// Results in (v1, v2, t) where v's are velocity of burns and t is the time of the orbit 
// Homann math: 
// Δv₁ = √(μ/r1)·(√(2r2/(r1+r2)) − 1)
// Δv₂ = √(μ/r2)·(1 − √(2r1/(r1+r2)))
// t_transfer = π·√(a_t³/μ)
fn hohmann(r1: f64, r2: f64, mu: f64) -> (f64, f64, f64) {
    
    let v1 = (mu / r1).sqrt() * ((2.0 * r2 / (r1 + r2)).sqrt() - 1.0);
    let v2 = (mu / r2).sqrt() * ( 1.0 - (2.0 * r1 / (r1 + r2)).sqrt());
    let t = PI * ( (r1 + r2).powi(3) / (8.0 * mu)).sqrt();

    (v1, v2, t)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hohmann_matches_textbook_leo_to_geo() {
        let mu = 398_600.0;      // km^3/s^2, Earth
        let r1 = 6_678.0;        // km, ~300 km altitude LEO
        let r2 = 42_164.0;       // km, GEO
        let (dv1, dv2, t) = hohmann(r1, r2, mu);

        assert!((dv1 - 2.4258).abs() < 1e-3, "dv1 = {dv1}");
        assert!((dv2 - 1.4669).abs() < 1e-3, "dv2 = {dv2}");
        assert!((t - 18_989.0).abs() < 2.0, "t = {t}");
    }

    #[test]
    fn lambert_intercept_reaches_moving_target() {
        let mu = 1.0e14;
        // two coplanar circular orbits, target given a phase offset (m0 = 1.0 rad)
        let ship   = OrbitalElements { a: 1.0e8, e: 0.0, i: 0.0, lan: 0.0, arg_pe: 0.0, m0: 0.0, epoch: 0.0, mu };
        let target = OrbitalElements { a: 1.6e8, e: 0.0, i: 0.0, lan: 0.0, arg_pe: 0.0, m0: 1.0, epoch: 0.0, mu };

        let tof = hohmann_tof(ship.a, target.a, mu);
        let burn = plan_lambert_intercept(&ship, &target, 0.0, tof).expect("planned");

        // fly the planned transfer and confirm it lands on the planet, not just its radius
        let transfer = ship.with_burn(0.0, burn.dv);
        let arrival = transfer.offset_at(tof);
        let planet_pos = target.offset_at(tof);
        let miss = (arrival - planet_pos).length() / planet_pos.length();
        assert!(miss < 1e-6, "missed by {miss}: {arrival:?} vs {planet_pos:?}");
    }
}
