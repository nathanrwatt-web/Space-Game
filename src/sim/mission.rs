use bevy::prelude::*;
use bevy::math::DVec3;
use crate::math::{
    orbital_elements::OrbitalElements,
    lambert::lambert,
    hyperbola::{impact_parameter,capture_dv}
};
use crate::sim::{
    orbit::Burn,
    soi::soi_radius,
    transfer::hohmann_tof,
};

use std::f64::consts::TAU;

pub struct MissionPlan {
    pub departure: Burn,
    pub t_peri: f64,
    pub circular: OrbitalElements,
    pub v_inf: f64,        // arrival excess speed
    pub dep_dv: f64,       // |departure burn|
    pub capture_cost: f64, // estimated insertion Δv
}

// Solves the planet relative hyperbola at SOI entry and returns 
// (t_peri, circular_elemts) for the new orbit 
pub fn plan_capture(
    transfer: &OrbitalElements,
    target: &OrbitalElements,
    mu_target: f64,
    t_dep: f64,
    tof: f64,
) -> Option<(f64, OrbitalElements)> {
    let r_soi = soi_radius(target.a, mu_target, transfer.mu);
    let Some(t_soi) = find_soi_entry(transfer, target, r_soi, t_dep, t_dep + tof * 1.2) else {
        info!("capture: transfer never enters {:.1}-radius SOI (aim too shallow / r_p too deep)", r_soi);
        return None;
    };

    let r_rel = transfer.offset_at(t_soi) - target.offset_at(t_soi);
    let v_rel = transfer.velocity_at(t_soi) - target.velocity_at(t_soi);
    let hyp = OrbitalElements::from_state(r_rel, v_rel, mu_target, t_soi);
    let mut t_peri = hyp.time_of_periapsis();
    if hyp.e < 1.0 {
        // Sub-escape: the ship entered already bound — an ellipse, not a hyperbola.
        // An ellipse passes periapsis every period, and time_of_periapsis() returns a
        // single one that can fall before SOI entry. Advance to the first periapsis at
        // or after t_soi so the capture is scheduled in the future, not the past.
        info!("capture: entered SOI sub-escape (e={:.3}); already bound", hyp.e);
        let period = hyp.period();
        if t_peri < t_soi {
            let n = ((t_soi - t_peri) / period).ceil();
            t_peri += n * period;
        }
    }
    let r_peri = hyp.offset_at(t_peri);
    let v_peri = hyp.velocity_at(t_peri);

    let v_circ_vec = v_peri.normalize() * (mu_target / r_peri.length()).sqrt();
    let circular = OrbitalElements::from_state(r_peri, v_circ_vec, mu_target, t_peri);
    Some((t_peri, circular))
}


pub fn plan_mission(
    ship: &OrbitalElements,
    target: &OrbitalElements,
    mu_target: f64,
    t_now: f64,
    r_p: f64,
    siblings: &[(OrbitalElements, f64)],
    primary_floor: f64,
) -> Option<MissionPlan> {
    let tof = hohmann_tof(ship.a, target.a, ship.mu);
    let t_dep = find_window(ship, target, mu_target, t_now, r_p, tof, siblings, primary_floor)?;
    let departure = bplane_target(ship, target, t_dep, tof, mu_target, r_p)?;
    let transfer = ship.with_burn(t_dep, departure.dv);
    let (t_peri, circular) = plan_capture(&transfer, target, mu_target, t_dep, tof)?;
        let v_inf = (transfer.velocity_at(t_dep + tof) - target.velocity_at(t_dep + tof)).length();
    let dep_dv = departure.dv.length();
    let capture_cost = capture_dv(v_inf, mu_target, r_p);
    Some(MissionPlan { departure, t_peri, circular, v_inf, dep_dv, capture_cost })
}

// Departure burn whose transfer, via a B-plane offset, makes the planet-relative
// approach hyperbola's periapsis ~ r_p (instead of aiming at the body center).
// `ship` and `target` must share a parent (same frame & mu).
pub fn bplane_target(
    ship: &OrbitalElements,
    target: &OrbitalElements,
    t_dep: f64,
    tof: f64,
    mu_target: f64,
    r_p: f64,
) -> Option<Burn> {
    let mu = ship.mu;                      // shared-parent GM
    let t_arr = t_dep + tof;               // arrival time 
    let r1 = ship.offset_at(t_dep);
    let v_ship = ship.velocity_at(t_dep);
    let target_center = target.offset_at(t_arr);
    let v_target = target.velocity_at(t_arr);
    let r_soi = soi_radius(target.a, mu_target, mu);

    // v_inf depends on the aim, the aim depends on v_inf → iterate a few times.
    let mut aim = target_center;
    for _ in 0..3 {
        let (_v1, v2) = lambert(r1, aim, tof, mu, true)?;
        let v_inf = v2 - v_target;
        let speed = v_inf.length();
        if speed < 1e-9 { break; }
        let r_p_eff = r_p.min(0.9 * max_reachable_rp(speed, mu_target, r_soi)); // ← clamp
        let b = impact_parameter(speed, mu_target, r_p_eff);
        let w = v_inf.cross(DVec3::Z).normalize();
        aim = target_center + w * b;
    }

    // final solve with the converged aim
    let (v1, _v2) = lambert(r1, aim, tof, mu, true)?;
    Some(Burn { execute_at: t_dep, dv: v1 - v_ship })
}

// Estimated total Δv (departure + capture) for a center-aimed transfer at t_dep.
fn window_cost(
    ship: &OrbitalElements,
    target: &OrbitalElements,
    mu_target: f64,
    t_dep: f64,
    tof: f64,
    r_p: f64,
) -> Option<f64> {
    let mu = ship.mu;
    let r1 = ship.offset_at(t_dep);
    let r2 = target.offset_at(t_dep + tof); // center is fine for a cost estimate
    let (v1, v2) = lambert(r1, r2, tof, mu, true)?;
    let dv_dep = (v1 - ship.velocity_at(t_dep)).length();
    let v_inf = (v2 - target.velocity_at(t_dep + tof)).length();
    Some(dv_dep + capture_dv(v_inf, mu_target, r_p))
}

// first time in the window t_dep to t_max which the transfer crosses into SOI of target
fn find_soi_entry(
    transfer: &OrbitalElements,
    target: &OrbitalElements,
    r_soi: f64,
    t_dep: f64,
    t_max: f64,
) -> Option<f64> {

    let g = |t: f64| (transfer.offset_at(t) - target.offset_at(t)).length() - r_soi;
    let n = 512;
    let mut prev_t = t_dep;
    let mut prev = g(prev_t);

    for k in 1..=n {
        let t = t_dep + (t_max - t_dep) * k as f64 / n as f64;
        let cur = g(t);
        if prev > 0.0 && cur <= 0.0 {
            let (mut lo, mut hi) = (prev_t, t);   // bisect the +→− crossing
            for _ in 0..60 {
                let mid = 0.5 * (lo + hi);
                if g(mid) > 0.0 { lo = mid; } else { hi = mid; }
            }
            return Some(0.5 * (lo + hi));
        }
        prev_t = t;
        prev = cur;
    }
    None
}

// True if the transfer stays clear of every sibling's SOI and above the primary
// floor across [t_dep, t_dep+tof] (all positions in the shared-parent frame).
fn path_is_clear(
    transfer: &OrbitalElements,
    t_dep: f64,
    tof: f64,
    siblings: &[(OrbitalElements, f64)], // (elements, r_soi)
    primary_floor: f64,
) -> bool {
    let steps = 200;
    for k in 0..=steps {
        let t = t_dep + tof * k as f64 / steps as f64;
        let pos = transfer.offset_at(t);
        if pos.length() < primary_floor {
            return false; // dips into the central body
        }
        for (sib, r_soi) in siblings {
            if (pos - sib.offset_at(t)).length() < *r_soi {
                return false; // threads another body's SOI
            }
        }
    }
    true
}

// Departure time with minimum estimated total Δv over one synodic period.
fn find_window(
    ship: &OrbitalElements,
    target: &OrbitalElements,
    mu_target: f64,
    t_now: f64,
    r_p: f64,
    tof: f64,
    siblings: &[(OrbitalElements, f64)],
    primary_floor: f64,
) -> Option<f64> {
    let mu = ship.mu;
    let n1 = TAU / ship.period();
    let n2 = TAU / target.period();
    let dn = (n1 - n2).abs();
    let t_syn = if dn > 1e-12 { TAU / dn } else { ship.period() };

    // 1. cheap cost scan (no screening)
    let samples = 360;
    let mut candidates: Vec<(f64, f64)> = Vec::new(); // (t_dep, cost)
    for k in 0..samples {
        let t_dep = t_now + t_syn * k as f64 / samples as f64;
        if let Some(cost) = window_cost(ship, target, mu_target, t_dep, tof, r_p) {
            candidates.push((t_dep, cost));
        }
    }

    const DV_BUDGET: f64 = 800.0;
    let mut candidates: Vec<(f64, f64)> = candidates.into_iter().filter(|&(_, c)| c <= DV_BUDGET).collect();
    // 2. cheapest first
    candidates.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));
    // 3. cheapest window whose path is actually clear
    for (t_dep, _) in candidates {
        let r1 = ship.offset_at(t_dep);
        let Some((v1, _)) = lambert(r1, target.offset_at(t_dep + tof), tof, mu, true) else { continue };
        let transfer = OrbitalElements::from_state(r1, v1, mu, t_dep);
        if path_is_clear(&transfer, t_dep, tof, siblings, primary_floor) {
            return Some(t_dep);
        }
    }
    None
}

fn max_reachable_rp(v_inf: f64, mu: f64, r_soi: f64) -> f64 {
    let c = 2.0 * mu / (v_inf * v_inf);
    0.5 * (-c + (c * c + 4.0 * r_soi * r_soi).sqrt())   // the positive root
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sim::soi::soi_radius;
    use crate::sim::transfer::hohmann_tof;

    #[test]
    fn bplane_aim_yields_target_periapsis() {
        let mu = 1.0e16;        // star GM
        let mu_target = 1.0e12; // planet GM
        let ship   = OrbitalElements { a: 1.0e9, e: 0.0, i: 0.0, lan: 0.0, arg_pe: 0.0, m0: 0.0, epoch: 0.0, mu };
        let target = OrbitalElements { a: 1.6e9, e: 0.0, i: 0.0, lan: 0.0, arg_pe: 0.0, m0: 0.7, epoch: 0.0, mu };

        let r_soi = soi_radius(target.a, mu_target, mu);
        let r_p = 0.1 * r_soi;
        let tof = hohmann_tof(ship.a, target.a, mu);

        let burn = bplane_target(&ship, &target, 0.0, tof, mu_target, r_p).expect("planned");

        // fly the transfer, find the FIRST SOI crossing, rebuild the planet-relative conic there
        let transfer = ship.with_burn(0.0, burn.dv);
        let mut t_entry = None;
        let n = 8000;
        for k in 0..=n {
            let t = tof * 1.02 * k as f64 / n as f64;
            if (transfer.offset_at(t) - target.offset_at(t)).length() < r_soi {
                t_entry = Some(t);
                break;
            }
        }
        let t = t_entry.expect("ship entered the SOI");
        let r_rel = transfer.offset_at(t) - target.offset_at(t);
        let v_rel = transfer.velocity_at(t) - target.velocity_at(t);
        let hyp = OrbitalElements::from_state(r_rel, v_rel, mu_target, t);

        assert!(hyp.e > 1.0, "approach not hyperbolic: e = {}", hyp.e);
        let peri = hyp.a * (1.0 - hyp.e);             // a<0,(1-e)<0 → r_p>0
        assert!((peri - r_p).abs() / r_p < 0.2, "periapsis {peri} vs target {r_p}");
    }


    #[test]
    fn plan_capture_produces_circular_orbit() {
        let mu = 1.0e16;
        let mu_target = 1.0e12;
        let ship   = OrbitalElements { a: 1.0e9, e: 0.0, i: 0.0, lan: 0.0, arg_pe: 0.0, m0: 0.0, epoch: 0.0, mu };
        let target = OrbitalElements { a: 1.6e9, e: 0.0, i: 0.0, lan: 0.0, arg_pe: 0.0, m0: 0.7, epoch: 0.0, mu };

        let r_soi = soi_radius(target.a, mu_target, mu);
        let r_p = 0.1 * r_soi;
        let plan = plan_mission(&ship, &target, mu_target, 0.0, r_p, &[], 0.0).expect("mission");

        assert!(plan.circular.e < 1e-3, "e = {}", plan.circular.e);
        assert!((plan.circular.a - r_p).abs() / r_p < 0.25, "a = {} vs r_p {}", plan.circular.a, r_p);
        assert!(plan.t_peri > 0.0);
    }

    #[test]
    fn window_finds_low_vinf_departure() {
        let mu = 1.0e16;
        let mu_target = 1.0e12;
        let ship   = OrbitalElements { a: 1.0e9, e: 0.0, i: 0.0, lan: 0.0, arg_pe: 0.0, m0: 0.0, epoch: 0.0, mu };
        let target = OrbitalElements { a: 1.6e9, e: 0.0, i: 0.0, lan: 0.0, arg_pe: 0.0, m0: 2.0, epoch: 0.0, mu };

        let r_p = 0.1 * soi_radius(target.a, mu_target, mu);
        let tof = hohmann_tof(ship.a, target.a, mu);
        let t_best = find_window(&ship, &target, mu_target, 0.0, r_p, tof, &[], 0.0).expect("window");

        let vinf = |t_dep: f64| {
            let (_v1, v2) = lambert(ship.offset_at(t_dep), target.offset_at(t_dep + tof), tof, mu, true).unwrap();
            (v2 - target.velocity_at(t_dep + tof)).length()
        };
        let t_syn = TAU / (TAU / ship.period() - TAU / target.period()).abs();

        // the chosen window beats a deliberately mis-phased departure...
        assert!(vinf(t_best) < vinf(t_best + 0.5 * t_syn), "window did not minimize v_inf");
        // ...and the approach is gentle: v_inf well below the target's orbital speed
        let v_orbit = (mu / target.a).sqrt();
        assert!(vinf(t_best) < 0.2 * v_orbit, "v_inf {} too large vs {v_orbit}", vinf(t_best));
    }

    #[test]
    fn screening_blocks_obstructed_path() {
        let mu = 1.0e16;
        let ship   = OrbitalElements { a: 1.0e9, e: 0.0, i: 0.0, lan: 0.0, arg_pe: 0.0, m0: 0.0, epoch: 0.0, mu };
        let target = OrbitalElements { a: 1.6e9, e: 0.0, i: 0.0, lan: 0.0, arg_pe: 0.0, m0: 2.0, epoch: 0.0, mu };
        let tof = hohmann_tof(ship.a, target.a, mu);

        let r1 = ship.offset_at(0.0);
        let (v1, _) = lambert(r1, target.offset_at(tof), tof, mu, true).unwrap();
        let transfer = OrbitalElements::from_state(r1, v1, mu, 0.0);

        let sib = OrbitalElements { a: 1.3e9, e: 0.0, i: 0.0, lan: 0.0, arg_pe: 0.0, m0: 0.0, epoch: 0.0, mu };
        assert!( path_is_clear(&transfer, 0.0, tof, &[(sib, 1.0)],     0.0)); // tiny SOI → clear
        assert!(!path_is_clear(&transfer, 0.0, tof, &[(sib, 1.0e12)],  0.0)); // huge SOI → blocked
        assert!(!path_is_clear(&transfer, 0.0, tof, &[],               2.0e9)); // floor above path → blocked
    }
}
