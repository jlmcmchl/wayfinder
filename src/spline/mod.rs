mod hermite;

pub use hermite::*;

use crate::math::Vec2;

pub trait Spline {
    fn position(&self, t: f64) -> Vec2;
    fn velocity(&self, t: f64) -> Vec2;
    fn rotation(&self, t: f64) -> Vec2 {
        let d = self.velocity(t);
        d / d.norm()
    }
    fn curvature(&self, t: f64) -> f64;
}
