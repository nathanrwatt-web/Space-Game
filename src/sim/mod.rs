// keplarian rails, contains propogate_orbits, the system which updates
// the position of every body with an orbit 
pub mod orbit;

// holds hohmann and lambert solver for interstellar travel
pub mod transfer;

// caculates the sphere of influence for planets 
// handles the entering of ships into soi
pub mod soi;

// handles time warp and stepping 
pub mod clock;

// handles execute_capture which is an analytic calculation
// of soi captures in moving ships 
pub mod capture;

// handles planning missions for inter body travel 
pub mod mission;

// handles numeric integration for continuous movement
pub mod integrate;

// what the ship is doing with thrust 
pub mod guidance;

// cell handling for frame calculations 
pub mod broadphase; 
