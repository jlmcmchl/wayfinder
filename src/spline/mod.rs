mod hermite;

use crate::Waypoint;
pub use hermite::*;

use crate::math::{Vec2, Vector};
use crate::point::Point;

pub trait Spline<T> {
    fn from_wps(start: &Waypoint, end: &Waypoint) -> T;

    fn position(&self, t: f64) -> Vec2;
    fn velocity(&self, t: f64) -> Vec2;
    fn acceleration(&self, t: f64) -> Vec2;
    fn jerk(&self, t: f64) -> Vec2;

    fn point_at(&self, t: f64) -> Point {
        Point {
            position: self.position(t),
            velocity: self.velocity(t),
            acceleration: self.acceleration(t),
            jerk: self.jerk(t),
        }
    }

    fn rotation(&self, t: f64) -> Vec2 {
        let d = self.velocity(t);
        d.scale(1. / d.norm())
    }

    fn curvature(&self, _: f64) -> f64 {
        0.
    }

    fn d_curvature(&self, _: f64) -> f64 {
        0.
    }

    fn integral_d_curvature_d_t_squared(&self, _: u64) -> f64 {
        0.
    }
}
