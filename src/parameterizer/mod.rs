mod cheesy;
mod jaci;

pub use cheesy::*;
pub use jaci::*;

use crate::{Point, Spline};
use crate::Hermite;

pub trait Parameterizer {
    fn should_subdivide(&self, spline: &Hermite<f32>, t_curr: f32, t_step: f32) -> bool;

    fn parameterize(&self, splines: &[Hermite<f32>]) -> Vec<Point<f32>> {
        let mut pts = Vec::<Point<f32>>::new();
        pts.push(splines.get(0).unwrap().point_at(0.));

        for spline in splines {
            self.subdivide(&spline, &mut pts, 0., 1.);
        }

        pts
    }

    fn subdivide(&self, spline: &Hermite<f32>, out: &mut Vec<Point<f32>>, t0: f32, t1: f32) {
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
