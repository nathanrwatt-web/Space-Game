use bevy::math::{DVec3, DQuat};
use std::f64::consts::TAU;

#[derive(Clone, Copy, Debug)]
pub(crate) struct OrbitalElements {
    // ==== SHAPE AND SIZE ====
    // semi major axis of the elpise 
    // half the length of the longest axis 
    pub(crate) a: f64,  
    // eccentricity, 0 => circle, (0, 1) -> elipse 
    pub(crate) e: f64,  

    // ==== ORIENTATION IN SPACE ==== 
    // inclination, title of orbital plain relative to reference plain 
    pub(crate) i: f64, 
    // Longitude of the ascending node: Ω
    // in regards to a plane of reference, it represents the horizontal angle 
    // to the Ascending Node (where the orbit intersects the plane) relative to 
    // a reference direction which is on that plane 
    pub(crate) lan: f64, 
    // Argument of the Periapsis: ω
    // Angle from the ascending node to the periapsis measured in the direction 
    // of motionl. The periapsis is the point in an eliptical orbit where an object 
    // is closest to the center of the mass it is orbitting
    pub(crate) arg_pe: f64, 
    // ==== EXTRA MISCL. DATA ====
    // mean anomaly at epoch 
    // mean anomaly  M = (2pi / time to complete orbit) * (t - time at which body is at periapsis)
    // epoch = t_0 
    pub(crate) m0: f64,
    // epoch is some t_0 in seconds 
    pub(crate) epoch: f64,
    // this is the parents G·M, which is (m^3 / s^2)
    // see Proposition 1 
    pub(crate) mu: f64,
}

impl OrbitalElements {

    // for checking values 
    pub(crate) fn new(a: f64, e: f64, i: f64, lan: f64, arg_pe: f64, m0: f64, epoch: f64, mu: f64) -> Self {
        debug_assert!(e < 1.0, "Orbit is not bound!");
        debug_assert!(a >= 0.0, "Semi major axis is negative!");
        Self { a, e, i, lan, arg_pe, m0, epoch, mu }
    }
    // see Proposition 5 
    fn mean_motion(&self) -> f64 {
        // mean motion is given by G·M / a^{3/2 }
        (self.mu / self.a.powi(3)).sqrt()
    }

    // 2pi / n = 2pi / (2pi / T) = T = period 
    pub(crate) fn period(&self) -> f64 {
        TAU / self.mean_motion() 
    }

    // given a time t, calculate the offset in position 
    // returns the new position of an orbiting body after t seconds 
    // from the frame of reference of the parent 
    pub(crate) fn offset_at(&self, t: f64) -> DVec3 {
        // mean anomaly at time t: M = M_0 + n · (t - t_0)
        let m = self.m0 + self.mean_motion() * (t - self.epoch);
        let ea = solve_kepler(m, self.e); // Eccentric anomaly 

        // see Proposition 4 
        let x = self.a * (ea.cos() - self.e);
        let y = self.a * (1.0 - self.e * self.e).sqrt() * ea.sin(); // semi minor b = a * sqrt(1 - e^2) 
        // rightmost applied first means spins by ω -> tilted by i -> swung by Ω
        let q = DQuat::from_rotation_z(self.lan)
              * DQuat::from_rotation_x(self.i)
              * DQuat::from_rotation_z(self.arg_pe);
        q * DVec3::new(x, y, 0.0)
    }
}

// see Proposition 2 
fn solve_kepler(m: f64, e: f64) -> f64 { // mean Anomaly + eccentricity
    // convert to [0, 2pi)
    let m = m.rem_euclid(TAU);
    
    // seed guess, some math stuff i'm not totally sure about the derivation  
    let mut ea = m + e * m.sin();

    // at most 8 iterations of newtons method, see Proposition 3
    for _ in 0..8 {
        let dx = (ea - e * ea.sin() - m) / (1.0 - e * ea.cos()); 
        ea -= dx;
        if dx.abs() < 1e-12 { break; }
    }
    ea // outputs Eccentric Anomaly
}


/* ===== MATH =====
 *
 * Proposition 1 
 *  Masses 
 *  Two bodies with masses M and m and positions R_M and R_m. 
 *  Acceleration due to G: R_m'' = -GM · r/r^3 and R_M = Gm · r/r^3 
 *  r'' = R_m'' - R_M'' = -G(M+m) · r/r^3 = −μ·r/r^3
 *  since m << M, we have μ ≈ GM = mu
 *
 * Proposition 2 
 *  Kepler's equation 
 *  Mean Anomaly = Eccentric Anomaly - eccentricity · sin(Eccentric Anomaly)
 *  M = E - esin(E)
 *
 * Proposition 3 
 *  dx = f(E)/f'(E) 
 *  f(E) = E - e sin (E) - M
 *  E_0 = (meanAnomoly + eccentricity * sin(meanAnomoly))  = Eccentric anomoly seed
 *  f'(E) = 1 - e cos(E)
 *
 * Proposition 4
 *  Given Eccentric Anomaly and Kepler's equation, 
 *  x = a(cos(E) - e)
 *  y = b(sin(E))
 *  where a and b are the major and minor semi axis
 *
 * Proposition 5: Keplers 3rd Law 
 *  "the square of a planet's orbital period is directly proportional
 *  to the cube of the semi-major axis of its orbit"
 *  aka (some constant) * T^2 = a^3 
 *  mean motion is the average angular speed required to complete a full
 *  revolution 
 *  Derivation comes from T = 2pi * sqrt (a^3 / GM )
 *  nT = 2pi so n = GM / sqrt(a^3)
 */ 

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    const MU_SUN: f64 = 1.327e20;

    // Sun-centred orbit, epoch 0
    fn elements(a: f64, e: f64, i: f64, lan: f64, arg_pe: f64, m0: f64) -> OrbitalElements {
        OrbitalElements { a, e, i, lan, arg_pe, m0, epoch: 0.0, mu: MU_SUN }
    }

    // ---- Tier 1: core correctness ----
    #[test]
    fn solver_inverts_keplers_equation() {
        for &e in &[0.0, 0.1, 0.5, 0.9] {
            for k in 0..200 {
                let m = TAU * k as f64 / 200.0;
                let ea = solve_kepler(m, e);
                let residual = ea - e * ea.sin() - m; // 0 = E - e sin(E) - M
                assert!(residual.abs() < 1e-9, "e={e}, M={m}: residual {residual}");
            }
        }
    }

    #[test]
    // solve_kepler with e = 0 should just return m 
    fn circular_solver_returns_mean_anomaly() {
        for k in 0..50 {
            let m = TAU * k as f64 / 50.0;
            assert!((solve_kepler(m, 0.0) - m).abs() < 1e-12);
        }
    }

    #[test]
    // edge case tests for mean anomlay = 0 and pi 
    fn apsis_anchors() {
        for &e in &[0.0, 0.2, 0.6, 0.9] {
            assert!(solve_kepler(0.0, e).abs() < 1e-12);        // periapsis: E=0
            assert!((solve_kepler(PI, e) - PI).abs() < 1e-12);  // apoapsis: E=π
        }
    }

    #[test]
    fn solver_handles_negative_and_multirev_anomaly() {
        let e = 0.3;
        for &m in &[-2.0_f64, 20.0, -100.0, 1000.0] {
            let ea = solve_kepler(m, e);
            let wrapped = m.rem_euclid(TAU);
            assert!((ea - e * ea.sin() - wrapped).abs() < 1e-9, "M={m}");
        }
    }

    #[test]
    fn period_round_trip_generic() {
        // all six elements nonzero
        let el = elements(1.5e11, 0.3, 0.4, 1.1, 0.7, 0.5);
        let drift = (el.offset_at(0.0) - el.offset_at(el.period())).length();
        assert!(drift < 0.1, "drifted {drift}00 mm after one period");
    }

    #[test]
    fn apsis_distances() {
        let (a, e) = (1.5e11, 0.4);
        let el = elements(a, e, 0.3, 0.9, 0.6, 0.0); // m0=0 ⇒ starts at periapsis
        let r_peri = el.offset_at(0.0).length();
        let r_apo  = el.offset_at(el.period() / 2.0).length();
        assert!((r_peri - a * (1.0 - e)).abs() < 1.0, "peri {r_peri}");
        assert!((r_apo  - a * (1.0 + e)).abs() < 1.0, "apo  {r_apo}");
    }

    #[test]
    fn one_au_orbit_is_one_year() {
        let el = elements(1.496e11, 0.0167, 0.0, 0.0, 0.0, 0.0);
        let years = el.period() / (365.25 * 86400.0);
        assert!((years - 1.0).abs() < 0.01, "got {years} yr"); // catches unit bugs
    }

    #[test]
    fn offset_is_deterministic() {
        let el = elements(1.5e11, 0.3, 0.4, 1.1, 0.7, 0.5);
        let t = 1.234e7;
        assert_eq!(el.offset_at(t), el.offset_at(t)); // exact eq is correct here
    }

    #[test]
    fn zero_inclination_stays_in_plane() {
        let el = elements(1.5e11, 0.3, 0.0, 1.1, 0.7, 0.0); // i=0, lan/arg_pe nonzero
        for k in 0..50 {
            let t = el.period() * k as f64 / 50.0;
            assert!(el.offset_at(t).z.abs() < 1e-3, "z leaked at t={t}");
        }
    }

    #[test]
    fn nonzero_inclination_leaves_plane() {
        let el = elements(1.5e11, 0.3, 0.5, 1.1, 0.7, 0.0);
        let max_z = (0..50)
            .map(|k| el.offset_at(el.period() * k as f64 / 50.0).z.abs())
            .fold(0.0_f64, f64::max);
        assert!(max_z > 1.0, "orbit never left the plane");
    }

    // ---- Tier 2: independent-physics cross-checks ----

    #[test]
    fn speed_matches_vis_viva() {
        let el = elements(1.5e11, 0.4, 0.3, 0.9, 0.6, 0.0);
        let dt = 60.0;
        for k in 1..20 {
            let t = el.period() * k as f64 / 20.0;
            let r = el.offset_at(t).length();
            let v_num = (el.offset_at(t + dt) - el.offset_at(t - dt)).length() / (2.0 * dt);
            let v_vis = (el.mu * (2.0 / r - 1.0 / el.a)).sqrt();
            assert!((v_num - v_vis).abs() / v_vis < 1e-3, "t={t}: {v_num} vs {v_vis}");
        }
    }

    #[test]
    fn angular_momentum_is_conserved() {
        let el = elements(1.5e11, 0.5, 0.3, 0.9, 0.6, 0.0);
        let dt = 60.0;
        let h_of = |t: f64| {
            let r = el.offset_at(t);
            let v = (el.offset_at(t + dt) - el.offset_at(t - dt)) / (2.0 * dt);
            r.cross(v).length()
        };
        let h0 = h_of(el.period() * 0.13);
        for k in 1..20 {
            let h = h_of(el.period() * k as f64 / 20.0);
            assert!((h - h0).abs() / h0 < 1e-3, "h drifted: {h} vs {h0}");
        }
    }

    // ---- Tier 3: rotation isolation (the #1 silent-bug spot) ----

    #[test]
    fn arg_pe_points_periapsis() {
        let w = 0.6;
        let el = elements(1.5e11, 0.4, 0.0, 0.0, w, 0.0); // i=lan=m0=0 ⇒ at periapsis on +x, spun by ω
        let dir = el.offset_at(0.0).normalize();
        let expected = DVec3::new(w.cos(), w.sin(), 0.0);
        assert!((dir - expected).length() < 1e-9, "periapsis dir {dir:?}");
    }

    #[test]
    fn lan_rotates_about_z() {
        // i=0 ⇒ lan and arg_pe both rotate within the xy-plane; +Δlan = rotate position by Δ about z
        let el_a = elements(1.5e11, 0.4, 0.0, 0.0, 0.6, 0.3);
        let el_b = elements(1.5e11, 0.4, 0.0, 0.5, 0.6, 0.3);
        let t = 1.0e6;
        let rotated = DQuat::from_rotation_z(0.5) * el_a.offset_at(t);
        assert!((el_b.offset_at(t) - rotated).length() < 1.0);
    }

    #[test]
    fn inclination_tilts_about_x() {
        // lan=arg_pe=0 ⇒ the orientation reduces to Rx(i)
        let inc = 0.5;
        let flat = elements(1.5e11, 0.4, 0.0, 0.0, 0.0, 1.2);
        let tilted = elements(1.5e11, 0.4, inc, 0.0, 0.0, 1.2);
        let expected = DQuat::from_rotation_x(inc) * flat.offset_at(0.0);
        assert!((tilted.offset_at(0.0) - expected).length() < 1.0);
    }
}
