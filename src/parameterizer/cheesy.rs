use crate::{Hermite, Parameterizer, Pose, Spline};
use nalgebra::{Point2, Scalar};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct Cheesy<N: Scalar> {
    max_dx: N,
    max_dy: N,
    max_dt: N,
}

impl Cheesy<f32> {
    pub fn new(max_dx: f32, max_dy: f32, max_dt: f32) -> Self {
        Cheesy {
            max_dx,
            max_dy,
            max_dt,
        }
    }
}

impl Parameterizer for Cheesy<f32> {
    fn should_subdivide(&self, spline: &Hermite<f32>, t_curr: f32, t_step: f32) -> bool {
        let p0 = spline.point_at(t_curr);
        let p1 = spline.point_at(t_curr + t_step);
        let r0 = p0.heading();
        let r1 = p1.heading();


        let tf = Pose::new(
            Point2::from(r0.inverse() * (p1.position - p0.position)),
            r1 * r0.inverse(),
        );
        let twist = tf.log();

        twist.dx().abs() > self.max_dx
            || twist.dy().abs() > self.max_dy
            || twist.dt().abs() > self.max_dt
    }
}
