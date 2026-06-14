use bevy::math::{DVec3, DQuat};
use std::f64::consts::TAU;
use serde::{Serialize, Deserialize};

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
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
    // 2pi / n = 2pi / (2pi / T) = T = period 
    pub(crate) fn period(&self) -> f64 {
        if self.e >= 1.0 { return f64::INFINITY }
        TAU / self.mean_motion() 
    }

    
    pub(crate) fn offset_at(&self, t: f64) -> DVec3 {
        self.state_vectors_at(t).0
    }

    pub(crate) fn velocity_at(&self, t: f64) -> DVec3 {
        self.state_vectors_at(t).1
    }

    // given a time t, calculate the offset in position and velocity 
    // from reference frame of parent -> (new pos, velocity)
    pub(crate) fn state_vectors_at(&self, t: f64) -> (DVec3, DVec3) {

        let n = self.mean_motion();
        // mean anomaly at time t: M = M_0 + n · (t - t_0)
        let m = self.m0 + self.mean_motion() * (t - self.epoch);
        let q = self.orientation();

        if self.e < 1.0 {
            let ea = solve_kepler(m, self.e); // Eccentric anomaly 

            let (sin_e, cos_e) = (ea.sin(), ea.cos());
            let b = self.a * (1.0 - self.e * self.e).sqrt(); // semi minor axis 
        
            // see Proposition 4
            let new_pos = DVec3::new(self.a * (cos_e - self.e), b * sin_e, 0.0);

            // see Proposition 6
            let edot = n / (1.0 - self.e * cos_e); // derivative of Eccentric Anomaly 
            // 0 change in non x-y plane because planar orbit 
            let vel = DVec3::new(-self.a * sin_e, b * cos_e, 0.0) * edot;

            (q * new_pos, q * vel)
        } else {
            let hea = solve_kepler_hyperbolic(m, self.e);
            let (sinh_e, cosh_e) = (hea.sinh(), hea.cosh());

            let bh = -self.a * (self.e * self.e - 1.0).sqrt();

            let new_pos = DVec3::new(self.a * (cosh_e - self.e), bh * sinh_e, 0.0);

            let hdot = n / (self.e * cosh_e - 1.0);
            let vel = DVec3::new(self.a * sinh_e, bh * cosh_e, 0.0) * hdot;
            (q * new_pos, q * vel)
        }
    }

    // see Proposition 7 
    // reconstructs orbital elements from distance and velocity to parent 
    pub(crate) fn from_state(r: DVec3, v: DVec3, mu: f64, epoch: f64) -> Self {
        let r_mag = r.length();
       
        // angular momentum 
        let h = r.cross(v);
        let h_mag = h.length();

        let node = DVec3::Z.cross(h);
        let node_mag = node.length();

        let e_vec = v.cross(h) / mu - r / r_mag;
        let e = e_vec.length();

        let energy = v.length_squared() / 2.0 - mu / r_mag;
        let a = -mu / (2.0 * energy);
        let i = (h.z / h_mag).clamp(-1.0, 1.0).acos();
        let (lan, arg_pe) = if node_mag > 1e-9 {
            let mut lan = (node.x / node_mag).clamp(-1.0, 1.0).acos();
            if node.y < 0.0 {
                lan = TAU - lan;
            }
            let mut arg_pe = (node.dot(e_vec) / (node_mag * e)).clamp(-1.0, 1.0).acos();
            if e_vec.z < 0.0 { arg_pe = TAU - arg_pe; } (lan, arg_pe)
        } else {
            // equatorial (i ≈ 0): node vanishes. Pin Ω = 0 and fold the whole angle
            // into ω = longitude of periapsis. (prograde assumed: h.z > 0)
            (0.0, e_vec.y.atan2(e_vec.x).rem_euclid(TAU))
        };

        // true anomaly v -> eccentric anomaly E -> mean anomaly M 
        let mut nu = (e_vec.dot(r) / (e * r_mag)).clamp(-1.0, 1.0).acos();

        if r.dot(v) < 0.0 { nu = TAU - nu; }
        
        let m0 = if e < 1.0 {
            let ea = 2.0 * ((1.0 - e).sqrt() * (nu * 0.5).sin())
                .atan2((1.0 + e).sqrt() * (nu * 0.5).cos());
            ea - e * ea.sin()
        } else {
            let hea = 2.0 * (((e - 1.0) / (e + 1.0)).sqrt() * (nu * 0.5).tan()).atanh();
            e * hea.sinh() - hea
        };

        OrbitalElements {
            a,
            e,
            i,
            lan,
            arg_pe,
            m0,
            epoch,
            mu,
        }
    }

    // constructs a new orbit given a change in velocity dv and time t
    pub(crate) fn with_burn(&self, t: f64, dv: DVec3) -> OrbitalElements {
        let (r, v) = self.state_vectors_at(t);
        OrbitalElements::from_state(r, v + dv, self.mu, t)
    }
    
    // works for both conics, constructs point at the ture anomoly 
    // conic polar equation: r = p / (1 + e·cos ν)
    // semi-latus rectum: p = a(1−e²)
    pub(crate) fn point_at_true_anomaly(&self, nu: f64) -> DVec3 {
        let p = self.a * (1.0 - self.e * self.e);
        let r = p / (1.0 + self.e * nu.cos());
        self.orientation() * DVec3::new(r * nu.cos(), r * nu.sin(), 0.0)
    }

    pub(crate) fn time_of_periapsis(&self) -> f64 {
        self.epoch - self.m0 / self.mean_motion()
    }

    // mean anomaly at time t, wrapped to [0, TAU). M = M_0 + n·(t - t_0)
    pub(crate) fn mean_anomaly_at(&self, t: f64) -> f64 {
        (self.m0 + self.mean_motion() * (t - self.epoch)).rem_euclid(TAU)
    }

    // outward normal of the orbital plane (perifocal +Z rotated into world axes)
    pub(crate) fn plane_normal(&self) -> DVec3 {
        self.orientation() * DVec3::Z
    }

    // rightmost applied first means spins by ω -> tilted by i -> swung by Ω
    fn orientation(&self) -> DQuat {
        DQuat::from_rotation_z(self.lan)
            * DQuat::from_rotation_x(self.i)
            *  DQuat::from_rotation_z(self.arg_pe)
    }

    // see Proposition 5 
    fn mean_motion(&self) -> f64 {
        // mean motion is given by G·M / a^{3/2 }
        (self.mu / self.a.abs().powi(3)).sqrt()
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

fn solve_kepler_hyperbolic(m: f64, e: f64) -> f64 {

    // seed 
    let mut hea = if m.abs() > 6.0 {
        m.signum() * (2.0 * m.abs() / e + 1.8).ln()
    } else {
        m / (e - 1.0) // linear near periapsis 
    };

    for _ in 0..50  {
        let f = e * hea.sinh() - hea - m;
        let fp = e * hea.cosh() - 1.0;
        let dx = f / fp;
        hea -= dx;
        if dx.abs() < 1e-12 { break; }
    }
    hea
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
 *
 * Proposition 6 
 *  E' = mean_motion / ( 1 - e * cos(E)) --- derivative of Keplers equation
 *  vx = -a * sin(E) * E'                --- x-coord derivative 
 *  vy = b * cos(E) * E'                 --- y-coord derivative 
 *  q = angle vector 
 *  Kv = q * (vx, vy, 0)
 * 
 * Proposition 7 
 *  |r| = distance 
 *  h = r.cross(v) := specific angular momentum 
 *  h is fixed and perpendicular to the orbital plane
 *  node = (0,0,1) x h = (-h_y, h_x, 0)
 *  d/dt(v × h) = v̇ × h = (−μ/r³)[ r × (r × v) ]
 *          = (−μ/r³)[ r(r·v) − v r² ]
 *          = μ( v/r − (r·v) r / r³ )
 *          = μ · d/dt( r / |r| ) 
 *  specific energy = kinetic + potential 
 *  ε = v²/2 − μ/r = μ/r − μ/(2a) − μ/r = −μ/(2a)
 *  cos i = (h·ẑ)/|h| = h_z/|h|
 *  cos Ω = node_x/|node|
 *  cos ω = (node·e_vec)/(|node|·e)
 *  tan(E/2) = √((1−e)/(1+e))·tan(ν/2)
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

    #[test]
    fn velocity_matches_finite_difference() {
        let el = elements(1.5e11, 0.4, 0.3, 0.9, 0.6, 0.2);
        let dt = 1.0;
        for k in 1..20 {
            let t = el.period() * k as f64 / 20.0;
            let v_analytic = el.velocity_at(t);
            let v_num = (el.offset_at(t + dt) - el.offset_at(t - dt)) / (2.0 * dt);
            assert!((v_analytic - v_num).length() / v_num.length() < 1e-6,
                    "t={t}: {v_analytic:?} vs {v_num:?}");
        }
    }

    #[test]
    fn velocity_magnitude_matches_vis_viva() {
        let el = elements(1.5e11, 0.4, 0.3, 0.9, 0.6, 0.0);
        for k in 1..20 {
            let t = el.period() * k as f64 / 20.0;
            let r = el.offset_at(t).length();
            let v = el.velocity_at(t).length();
            let v_vis = (el.mu * (2.0 / r - 1.0 / el.a)).sqrt();
            assert!((v - v_vis).abs() / v_vis < 1e-9, "t={t}: {v} vs {v_vis}");
        }
    }

    #[test]
    fn from_state_round_trips() {
        let el = elements(1.5e11, 0.4, 0.3, 0.9, 0.6, 0.2);   // inclined, generic
        for k in 0..12 {
            let t = el.period() * k as f64 / 12.0;
            let (r, v) = el.state_vectors_at(t);
            let el2 = OrbitalElements::from_state(r, v, el.mu, t);

            let (r2, v2) = el2.state_vectors_at(t);
            assert!((r - r2).length() < 1.0,  "pos mismatch at t={t}");
            assert!((v - v2).length() < 1e-6, "vel mismatch at t={t}");

            let t2 = t + 1.0e6;                                 // still locked together later
            assert!((el.offset_at(t2) - el2.offset_at(t2)).length() < 1.0, "drift at t={t}");
        }
    }

    #[test]
    fn from_state_round_trips_equatorial() {
        let el = elements(1.5e11, 0.3, 0.0, 0.0, 0.7, 0.4);    // i = 0, like a ship
        let t = el.period() * 0.37;
        let (r, v) = el.state_vectors_at(t);
        let el2 = OrbitalElements::from_state(r, v, el.mu, t);
        let t2 = t + 2.0e6;
        assert!((el.offset_at(t2) - el2.offset_at(t2)).length() < 1.0);
    }

    #[test]
    fn hyperbolic_round_trips() {
        let r = DVec3::new(1.5e11, 0.0, 0.0);
        let v = DVec3::new(0.0, 5.0e4, 1.0e4);            // |v| > escape ⇒ hyperbolic
        let el = OrbitalElements::from_state(r, v, MU_SUN, 0.0);
        assert!(el.e > 1.0, "e = {}", el.e);
        let (r2, v2) = el.state_vectors_at(0.0);
        assert!((r - r2).length() < 1.0);
        assert!((v - v2).length() < 1e-6);
    }

    #[test]
    fn hyperbolic_velocity_matches_vis_viva() {
        let el = OrbitalElements::from_state(
            DVec3::new(1.5e11, 0.0, 0.0), DVec3::new(0.0, 5.0e4, 1.0e4), MU_SUN, 0.0);
        for k in -10..=10 {
            let t = k as f64 * 1.0e5;
            let r = el.offset_at(t).length();
            let vmag = el.velocity_at(t).length();
            let vis = (el.mu * (2.0 / r - 1.0 / el.a)).sqrt(); // a<0 ⇒ 2/r − 1/a > 0
            assert!((vmag - vis).abs() / vis < 1e-9, "t={t}: {vmag} vs {vis}");
        }
    }
}
