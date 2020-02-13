use crate::{Coordinate, Hermite, MatMul, Matrix, Parameterizer, Spline, Vec2, Vector};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct Jaci {
    max_ds: f64,
    max_dc: f64,
}

impl Jaci {
    pub fn new(max_ds: f64, max_dc: f64) -> Self {
        Jaci { max_ds, max_dc }
    }
}

impl Parameterizer for Jaci {
    fn should_subdivide(&self, spline: &Hermite, t_curr: f64, t_step: f64) -> bool {
        let p0 = spline.position(t_curr);
        let p_mid = spline.position(t_curr + t_step / 2.0);
        let p1 = spline.position(t_curr + t_step);

        let k0 = spline.curvature(t_curr);
        let k1 = spline.curvature(t_curr + t_step);

        let arc_len = arc_length(p0, p_mid, p1);

        (k1 - k0).abs() > self.max_dc || arc_len > self.max_ds
    }
}

fn arc_length(start: Vec2, mid: Vec2, end: Vec2) -> f64 {
    let coeff = [
        [2.0 * (start.x() - end.x()), 2.0 * (start.y() - end.y())],
        [2.0 * (start.x() - mid.x()), 2.0 * (start.y() - mid.y())],
    ];

    if coeff.determinant() == 0.0 {
        start.scale(-1.).add(end).norm()
    } else {
        let rvec = &[
            start.norm_squared() - end.norm_squared(),
            start.norm_squared() - mid.norm_squared(),
        ];

        let _ref = Matrix::inverse(&coeff).unwrap().mul(rvec).scale(-1.);
        let d0 = _ref.add(start);
        let d1 = _ref.add(end);

        let curv = 1.0 / d0.norm();
        let a0 = d0.y().atan2(d0.x());
        let a1 = d1.y().atan2(d0.x());

        (a1 - a0).abs() / curv
    }
}
