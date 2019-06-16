use crate::math::*;
use crate::point::Point;
use crate::spline::Spline;
use crate::waypoint::*;

pub type Hermite = Mat2x6;

pub fn hermites(wps: &[Waypoint]) -> Vec<Hermite> {
    wps.iter()
        .zip(wps.iter().skip(1))
        .map(|(start, end)| Hermite::from_wps(start, end))
        .collect()
}

static COEFF_MATRIX: &Mat6 = &[
    [-6.0, 15.0, -10.0, 0.0, 0.0, 1.0],
    [-3.0, 8.0, -6.0, 0.0, 1.0, 0.0],
    [-0.5, 1.5, -1.5, 0.5, 0.0, 0.0],
    [6.0, -15.0, 10.0, 0.0, 0.0, 0.0],
    [-3.0, 7.0, -4.0, 0.0, 0.0, 0.0],
    [0.5, -1.0, 0.5, 0.0, 0.0, 0.0],
];

impl Spline<Hermite> for Hermite {
    fn from_wps(start: &Waypoint, end: &Waypoint) -> Hermite {
        let dist = 1.2 * end.point.add(start.point.scale(-1.)).norm();
        let scaled_tangent_0 = start.tangent.unit().scale(dist);
        let scaled_tangent_1 = end.tangent.unit().scale(dist);

        let coords = [
            start.point,
            scaled_tangent_0,
            start.curvature,
            end.point,
            scaled_tangent_1,
            end.curvature,
        ];

        coords.mul(COEFF_MATRIX)
    }

    fn position(&self, t: f64) -> Vec2 {
        let basis = &[t * t * t * t * t, t * t * t * t, t * t * t, t * t, t, 1.0];

        self.mul(basis)
    }

    fn velocity(&self, t: f64) -> Vec2 {
        let basis = &[
            5.0 * t * t * t * t,
            4.0 * t * t * t,
            3.0 * t * t,
            2.0 * t,
            1.0,
            0.0,
        ];

        self.mul(basis)
    }

    fn acceleration(&self, t: f64) -> Vec2 {
        let basis = &[20.0 * t * t * t, 12.0 * t * t, 6.0 * t, 2.0, 0.0, 0.0];

        self.mul(basis)
    }

    fn jerk(&self, t: f64) -> Vec2 {
        let basis = &[60.0 * t * t, 24.0 * t, 6.0, 0.0, 0.0, 0.0];

        self.mul(basis)
    }

    fn curvature(&self, t: f64) -> f64 {
        let vel = self.velocity(t);
        let acc = self.acceleration(t);

        (vel.x() * acc.y() - vel.y() * acc.x()) / vel.norm().powi(3)
    }

    fn d_curvature(&self, t: f64) -> f64 {
        let vel = self.velocity(t);
        let acc = self.acceleration(t);
        let jerk = self.jerk(t);

        let top = (vel.x() * jerk.y() - jerk.x() * vel.y()) * vel.norm_squared()
            - 3.0
                * (vel.x() * acc.y() - acc.x() * vel.y())
                * (vel.x() * acc.x() + vel.y() * acc.y());
        top / vel.norm().powi(5)
    }

    fn integral_d_curvature_d_t_squared(&self, samples: u64) -> f64 {
        (0..samples).fold(0.0, |acc, it| {
            let left = self.d_curvature(it as f64 / samples as f64);
            let right = self.d_curvature((it + 1) as f64 / samples as f64);
            acc + (left.powi(2) + right.powi(2)) / 2.0
        }) / (samples as f64)
    }
}
