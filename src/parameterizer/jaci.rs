use nalgebra::{Matrix2, Scalar, Vector2};
use serde::{Deserialize, Serialize};

use crate::{Hermite, Parameterizer, Spline};

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct Jaci<N: Scalar> {
    max_ds: N,
    max_dc: N,
}

impl<N: Scalar> Jaci<N> {
    pub fn new(max_ds: N, max_dc: N) -> Self {
        Jaci { max_ds, max_dc }
    }
}

impl Parameterizer for Jaci<f32> {
    fn should_subdivide(&self, spline: &Hermite<f32>, t_curr: f32, t_step: f32) -> bool {
        let p0 = spline.position(t_curr);
        let p_mid = spline.position(t_curr + t_step / 2.0);
        let p1 = spline.position(t_curr + t_step);

        let k0 = spline.curvature(t_curr);
        let k1 = spline.curvature(t_curr + t_step);

        let arc_len = arc_length(p0, p_mid, p1);

        (k1 - k0).abs() > self.max_dc || arc_len > self.max_ds
    }

    fn subdivide(&self, spline: &Hermite<f32>, out: &mut Vec<crate::Point<f32>>, t0: f32, t1: f32) {
        let mut t_curr = t0;
        let mut t_step = t1 - t0;
        let t_eps = 1e-3;

        while ((t0 < t1 && t_curr < t1) || (t0 > t1 && t_curr > t1)) && (t1 - t_curr).abs() > t_eps
        {
            if self.should_subdivide(spline, t_curr, t_step) {
                t_step /= 2.0;
            } else {
                t_curr += t_step;
                t_step = if t_step.abs() >= (t1 - t_curr).abs() / 2.0 {
                    t1 - t_curr
                } else {
                    t_step * 2.0
                };

                let point = spline.point_at(t_curr);
                out.push(point);
            }
        }
    }
}

fn arc_length(start: Vector2<f32>, mid: Vector2<f32>, end: Vector2<f32>) -> f32 {
    let coeff = Matrix2::from_row_slice(&[
        2.0 * (start.x - end.x),
        2.0 * (start.y - end.y),
        2.0 * (start.x - mid.x),
        2.0 * (start.y - mid.y),
    ]);

    if coeff.determinant() == 0.0 {
        (end - start).norm()
    } else {
        let rvec = Vector2::from_row_slice(&[
            start.norm_squared() - end.norm_squared(),
            start.norm_squared() - mid.norm_squared(),
        ]);

        let _ref = -coeff.try_inverse().unwrap() * rvec;
        let d0 = _ref + start;
        let d1 = _ref + end;

        let curv = 1.0 / d0.norm();
        let a0 = d0.y.atan2(d0.x);
        let a1 = d1.y.atan2(d0.x);

        (a1 - a0).abs() / curv
    }
}
