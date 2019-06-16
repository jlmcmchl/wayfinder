use crate::math::*;
use crate::spline::{Hermite, Spline};
use crate::Parameterizer;
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
pub struct Cheesy {
    max_dx: f64,
    max_dy: f64,
    max_dt: f64,
}

#[wasm_bindgen]
impl Cheesy {
    #[wasm_bindgen(constructor)]
    pub fn new(max_dx: f64, max_dy: f64, max_dt: f64) -> Self {
        Cheesy {
            max_dx,
            max_dy,
            max_dt,
        }
    }
}

impl Parameterizer for Cheesy {
    fn should_subdivide(&self, spline: &Hermite, t_curr: f64, t_step: f64) -> bool {
        let p0 = spline.point_at(t_curr);
        let p1 = spline.point_at(t_curr + t_step);
        let r0 = p0.heading();
        let r1 = p1.heading();

        let tf = Pose::new(
            Translation::rotate_by(p0.position.scale(-1.).add(p1.position), r0.inverse()),
            Rotation::rotate_by(r1, r0.inverse()),
        );
        let twist = tf.log();

        twist.dx().abs() > self.max_dx
            || twist.dy().abs() > self.max_dy
            || twist.dt().abs() > self.max_dt
    }
}
