use nalgebra::{Matrix2, Rotation2, Scalar, Vector2};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Point<N: Scalar> {
    pub position: Vector2<N>,
    pub velocity: Vector2<N>,
    pub acceleration: Vector2<N>,
    pub jerk: Vector2<N>,
}

impl Point<f32> {
    pub fn heading(&self) -> Rotation2<f32> {
        let heading = self.velocity.normalize();
        Rotation2::from_matrix(&Matrix2::from_column_slice(&[
            heading.x, heading.y, -heading.y, heading.x,
        ]))
    }

    pub fn curvature(&self) -> f32 {
        (self.velocity.x * self.acceleration.y - self.velocity.y * self.acceleration.x)
            / self.velocity.norm().powi(3)
    }
}
