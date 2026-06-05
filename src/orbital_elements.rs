use bevy::math::{DVec3, DQuat};
use std::f64::consts::TAU;

#[derive(Clone, Copy, Debug)]
struct OrbitalElements {
    // ==== SHAPE AND SIZE ====
    // semi major axis of the elpise 
    // half the length of the longest axis 
    a: f64,  
    // eccentricity, 0 => circle, (0, 1) -> elipse 
    e: f64,  

    // ==== ORIENTATION IN SPACE ==== 
    // inclination, title of orbital plain relative to reference plain 
    i: f64, 
    // Longitude of the ascending node: Ω
    // in regards to a plane of reference, it represents the horizontal angle 
    // to the Ascending Node (where the orbit intersects the plane) relative to 
    // a reference direction which is on that plane 
    lan: f64, 
    // Argument of the Periapsis: ω
    // Angle from the ascending node to the periapsis measured in the direction 
    // of motionl. The periapsis is the point in an eliptical orbit where an object 
    // is closest to the center of the mass it is orbitting
    arg_pe: f64, 
    // ==== EXTRA MISCL. DATA ====
    // mean anomaly at epoch 
    // mean anomaly  M = (2pi / time to complete orbit) * (t - time at which body is at periapsis)
    // epoch = t_0 
    m0: f64,
    // epoch is some t_0 in seconds 
    epoch: f64,
    // this is the parents G·M, which is (m^3 / s^2)
    // see Proposition 1 
    mu: f64,
}

impl OrbitalElements {

    // for checking values 
    fn new(a: f64, e: f64, i: f64, lan: f64, arg_pe: f64, m0: f64, epoch: f64, mu: f64) -> Self {
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
    fn period(&self) -> f64 {
        TAU / self.mean_motion() 
    }

    fn offset_at(&self, t: f64) -> DVec3 {
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
    ea
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
