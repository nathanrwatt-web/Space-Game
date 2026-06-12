// Hyperbolic-approach geometry for planetary capture.
// === see the math note at the bottom ===

// Impact parameter b: the perpendicular offset of the incoming asymptote from
// the body center that makes the approach hyperbola's periapsis exactly r_p,
// given hyperbolic excess speed v_inf about a body with gravitational param mu.
//   b = √( r_p² + 2·mu·r_p / v_inf² )
pub(crate) fn impact_parameter(v_inf: f64, mu: f64, r_p: f64) -> f64 {
    (r_p * r_p + 2.0 * mu * r_p / (v_inf * v_inf)).sqrt()
}

// Retrograde Δv to circularize at periapsis r_p from a hyperbolic approach of
// excess speed v_inf:   v_peri − v_circ
//   v_peri = √( v_inf² + 2·mu/r_p ),   v_circ = √( mu/r_p )
pub(crate) fn capture_dv(v_inf: f64, mu: f64, r_p: f64) -> f64 {
    (v_inf * v_inf + 2.0 * mu / r_p).sqrt() - (mu / r_p).sqrt()
}

/* ===== MATH =====
 * A hyperbola about mu with excess speed v_inf (the speed "at infinity",
 * ε = v_inf²/2 > 0) and periapsis r_p satisfies:
 *   a = −mu / v_inf²              (a < 0 for a hyperbola)
 *   e = 1 + r_p·v_inf² / mu       (> 1)
 *   b = |a|·√(e²−1)
 * Expanding b with the above:
 *   b² = (mu/v_inf²)²·(r_p v_inf²/mu)(2 + r_p v_inf²/mu)
 *      = r_p² + 2·mu·r_p / v_inf²
 * so aiming the asymptote a distance b off-center yields periapsis exactly r_p.
 *
 * At periapsis vis-viva gives v_peri = √(mu(2/r_p − 1/a)) = √(v_inf² + 2mu/r_p);
 * a pure retrograde burn down to the circular speed √(mu/r_p) is the capture cost.
 */

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn impact_parameter_hand_value() {
        // v_inf=1000, mu=1e12, r_p=1e6:  b = √(1e12 + 2e12) = √3·1e6
        let b = impact_parameter(1000.0, 1.0e12, 1.0e6);
        assert!((b - 3.0_f64.sqrt() * 1.0e6).abs() < 1.0, "b = {b}");
    }

    #[test]
    fn impact_parameter_matches_hyperbola_geometry() {
        let (v_inf, mu, r_p) = (1500.0, 4.0e12, 2.0e6);
        let b = impact_parameter(v_inf, mu, r_p);
        // independent route: e, |a|, then b = |a|·√(e²−1)
        let e = 1.0 + r_p * v_inf * v_inf / mu;
        let a = mu / (v_inf * v_inf); // |a|
        let b_geo = a * (e * e - 1.0).sqrt();
        assert!((b - b_geo).abs() / b_geo < 1e-12, "{b} vs {b_geo}");
    }

    #[test]
    fn capture_dv_hand_value() {
        // v_inf=1000, mu=1e12, r_p=1e6:  √3e6 − 1000 ≈ 732.05
        let dv = capture_dv(1000.0, 1.0e12, 1.0e6);
        assert!((dv - 732.0508).abs() < 1e-2, "dv = {dv}");
    }
}
