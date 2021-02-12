use nalgebra::{Matrix2, Matrix2x6, Matrix6, Matrix6x2, Rotation2, Vector2, Vector6};

use crate::{Spline, Waypoint};

pub type Hermite<N> = Matrix2x6<N>;

pub fn hermites(wps: &[Waypoint<f32>]) -> Vec<Hermite<f32>> {
    wps.iter()
        .zip(wps.iter().skip(1))
        .map(|(start, end)| Hermite::from_wps(start, end))
        .collect()
}

static COEFF_MATRIX: &[f32] = &[
    -6.0, -3.0, -0.5, 0.5, -3.0, 6.0, 15.0, 8.0, 1.5, -1.0, 7.0, -15.0, -10.0, -6.0, -1.5, 0.5,
    -4.0, 10.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0,
];

impl Spline<Hermite<f32>, f32> for Hermite<f32> {
    fn from_wps(start: &Waypoint<f32>, end: &Waypoint<f32>) -> Self {
        let coords = Matrix6x2::from_rows(&[
            start.point.transpose(),
            start.tangent.transpose(),
            start.curvature.transpose(),
            end.curvature.transpose(),
            end.tangent.transpose(),
            end.point.transpose(),
        ]);

        coords.transpose() * Matrix6::from_row_slice(COEFF_MATRIX)
    }

    fn position(&self, t: f32) -> Vector2<f32> {
        let basis = Vector6::from_column_slice(&[
            t * t * t * t * t,
            t * t * t * t,
            t * t * t,
            t * t,
            t,
            1.0,
        ]);

        self * basis
    }

    fn velocity(&self, t: f32) -> Vector2<f32> {
        let basis = Vector6::from_column_slice(&[
            5.0 * t * t * t * t,
            4.0 * t * t * t,
            3.0 * t * t,
            2.0 * t,
            1.0,
            0.0,
        ]);

        self * basis
    }

    fn acceleration(&self, t: f32) -> Vector2<f32> {
        let basis =
            Vector6::from_column_slice(&[20.0 * t * t * t, 12.0 * t * t, 6.0 * t, 2.0, 0.0, 0.0]);

        self * basis
    }

    fn jerk(&self, t: f32) -> Vector2<f32> {
        let basis = Vector6::from_column_slice(&[60.0 * t * t, 24.0 * t, 6.0, 0.0, 0.0, 0.0]);

        self * basis
    }

    fn curvature(&self, t: f32) -> f32 {
        let vel = self.velocity(t);
        let acc = self.acceleration(t);

        (vel.x * acc.y - vel.y * acc.x) / vel.norm().powi(3)
    }

    fn d_curvature(&self, t: f32) -> f32 {
        let vel = self.velocity(t);
        let acc = self.acceleration(t);
        let jerk = self.jerk(t);

        let top = (vel.x * jerk.y - jerk.x * vel.y) * vel.norm_squared()
            - 3.0 * (vel.x * acc.y - acc.x * vel.y) * (vel.x * acc.x + vel.y * acc.y);
        top / vel.norm().powi(5)
    }

    fn integral_d_curvature_d_t_squared(&self, samples: u64) -> f32 {
        (0..samples).fold(0.0, |acc, it| {
            let left = self.d_curvature(it as f32 / samples as f32);
            let right = self.d_curvature((it + 1) as f32 / samples as f32);
            acc + (left.powi(2) + right.powi(2)) / 2.0
        }) / (samples as f32)
    }

    fn rotation(&self, t: f32) -> nalgebra::Rotation2<f32> {
        let heading = self.velocity(t).normalize();
        Rotation2::from_matrix(&Matrix2::from_column_slice(&[
            heading.x, heading.y, -heading.y, heading.x,
        ]))
    }
}
