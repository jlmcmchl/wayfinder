use nalgebra::{Scalar, Vector2};
use serde::{Deserialize, Serialize};

#[derive(Clone, Serialize, Deserialize)]
pub struct Waypoint<N: Scalar> {
    pub point: Vector2<N>,
    pub tangent: Vector2<N>,
    pub curvature: Vector2<N>,
}
