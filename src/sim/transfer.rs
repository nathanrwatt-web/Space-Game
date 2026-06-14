use std::f64::consts::PI;

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
}
