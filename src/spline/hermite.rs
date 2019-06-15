use crate::math::*;
use crate::point::Point;
use crate::spline::Spline;
use crate::waypoint::*;

#[derive(Debug)]
pub struct Hermite {
    coords: Vec2x6,
}

#[allow(dead_code)]
pub fn hermites(wps: &[Waypoint]) -> Vec<Hermite> {
    wps.iter()
        .zip(wps.iter().skip(1))
        .map(|(start, end)| Hermite::new(start, end))
        .collect()
}

impl Hermite {
    pub fn hermites(wps: &[Waypoint]) -> Vec<Hermite> {
        wps.iter()
            .zip(wps.iter().skip(1))
            .map(|(start, end)| Hermite::new(start, end))
            .collect()
    }

    fn new(wp0: &Waypoint, wp1: &Waypoint) -> Self {
        let dist = 1.2 * (wp1.point - wp0.point).norm();
        let scaled_tangent_0 = wp0.tangent / wp0.tangent.norm() * dist;
        let scaled_tangent_1 = wp1.tangent / wp1.tangent.norm() * dist;

        let coords = Vec2x6::new(
            wp0.point,
            scaled_tangent_0,
            wp0.curvature,
            wp1.point,
            scaled_tangent_1,
            wp1.curvature,
        );

        let coeff = Vec6x6::new(
            Vec6::new(-6.0, 15.0, -10.0, 0.0, 0.0, 1.0),
            Vec6::new(-3.0, 8.0, -6.0, 0.0, 1.0, 0.0),
            Vec6::new(-0.5, 1.5, -1.5, 0.5, 0.0, 0.0),
            Vec6::new(6.0, -15.0, 10.0, 0.0, 0.0, 0.0),
            Vec6::new(-3.0, 7.0, -4.0, 0.0, 0.0, 0.0),
            Vec6::new(0.5, -1.0, 0.5, 0.0, 0.0, 0.0),
        );

        Hermite {
            coords: coords * coeff,
        }
    }

    fn acceleration(&self, t: f64) -> Vec2 {
        let basis = Vec6::new(20.0 * t * t * t, 12.0 * t * t, 6.0 * t, 2.0, 0.0, 0.0);

        self.coords.clone() * basis
    }

    fn jerk(&self, t: f64) -> Vec2 {
        let basis = Vec6::new(60.0 * t * t, 24.0 * t, 6.0, 0.0, 0.0, 0.0);

        self.coords.clone() * basis
    }

    fn d_curvature(&self, t: f64) -> f64 {
        let vel = self.velocity(t);
        let acc = self.acceleration(t);
        let jerk = self.jerk(t);

        let top = (vel.x * jerk.y - jerk.x * vel.y) * vel.norm_squared()
            - 3.0 * (vel.x * acc.y - acc.x * vel.y) * (vel.x * acc.x + vel.y * acc.y);
        top / vel.norm().powi(5)
    }

    pub fn integral_d_curvature_d_t_squared(&self, samples: u64) -> f64 {
        (0..samples).fold(0.0, |acc, it| {
            let left = self.d_curvature(it as f64 / samples as f64);
            let right = self.d_curvature((it + 1) as f64 / samples as f64);
            acc + (left.powi(2) + right.powi(2)) / 2.0
        }) / (samples as f64)
    }

    pub fn point_at(&self, t: f64) -> Point {
        Point {
            position: self.position(t),
            velocity: self.velocity(t),
            acceleration: self.acceleration(t),
            jerk: self.jerk(t),
        }
    }
}

impl Spline for Hermite {
    fn position(&self, t: f64) -> Vec2 {
        let basis = Vec6::new(t * t * t * t * t, t * t * t * t, t * t * t, t * t, t, 1.0);

        self.coords.clone() * basis
    }

    fn velocity(&self, t: f64) -> Vec2 {
        let basis = Vec6::new(
            5.0 * t * t * t * t,
            4.0 * t * t * t,
            3.0 * t * t,
            2.0 * t,
            1.0,
            0.0,
        );

        self.coords.clone() * basis
    }

    fn curvature(&self, t: f64) -> f64 {
        let vel = self.velocity(t);
        let acc = self.acceleration(t);

        (vel.x * acc.y - vel.y * acc.x) / vel.norm().powi(3)
    }
}
