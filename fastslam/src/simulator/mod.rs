pub mod controller;
pub mod robot;
pub mod laserscanner;

// Re-export all base-types.
pub use self::robot::Robot;
pub use self::robot::Direction;
pub use self::laserscanner::LaserScanner;