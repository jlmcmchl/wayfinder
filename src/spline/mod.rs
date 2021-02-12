mod hermite;

pub use hermite::*;
use nalgebra::{Rotation2, Scalar, Vector2};

use crate::{Point, Waypoint};

pub trait Spline<T, N: Scalar> {
    fn from_wps(start: &Waypoint<N>, end: &Waypoint<N>) -> T;

    fn position(&self, t: f32) -> Vector2<N>;
    fn velocity(&self, t: f32) -> Vector2<N>;
    fn acceleration(&self, t: f32) -> Vector2<N>;
    fn jerk(&self, t: f32) -> Vector2<N>;

    fn point_at(&self, t: f32) -> Point<N> {
        Point {
            position: self.position(t),
            velocity: self.velocity(t),
            acceleration: self.acceleration(t),
            jerk: self.jerk(t),
        }
    }

    fn rotation(&self, t: f32) -> Rotation2<N>;

    fn curvature(&self, t: f32) -> N;

    fn d_curvature(&self, t: f32) -> N;

    fn integral_d_curvature_d_t_squared(&self, samples: u64) -> N;
}
