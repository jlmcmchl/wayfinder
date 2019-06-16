mod cheesy;
mod jaci;

pub use cheesy::*;
pub use jaci::*;

use crate::spline::Spline;
use crate::Hermite;
use crate::Point;

pub enum Param {
    ParamCheesy(Cheesy),
    ParamJaci(Jaci),
}

pub trait Parameterizer {
    fn should_subdivide(&self, spline: &Hermite, t_curr: f64, t_step: f64) -> bool;

    fn parameterize(&self, splines: &[Hermite]) -> Vec<Point> {
        let mut pts = Vec::<Point>::new();
        pts.push(splines.get(0).unwrap().point_at(0.));

        for spline in splines {
            self.subdivide(&spline, &mut pts, 0., 1.);
        }

        pts
    }

    fn subdivide(&self, spline: &Hermite, out: &mut Vec<Point>, t0: f64, t1: f64) {
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
