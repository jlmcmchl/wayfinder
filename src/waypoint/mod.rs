use crate::math::Vec2;

pub struct Waypoint {
    pub point: Vec2,
    pub tangent: Vec2,
    pub curvature: Vec2,
}

impl Waypoint {
    pub fn new(x: f64, y: f64, dx: f64, dy: f64, ddx: f64, ddy: f64) -> Waypoint {
        Waypoint {
            point: [x, y],
            tangent: [dx, dy],
            curvature: [ddx, ddy],
        }
    }
}
