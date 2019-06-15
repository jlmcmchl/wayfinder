use wasm_bindgen::prelude::*;

use crate::math::Vec2;

#[wasm_bindgen]
pub struct Waypoint {
    pub point: Vec2,
    pub tangent: Vec2,
    pub curvature: Vec2,
}

#[wasm_bindgen]
impl Waypoint {
    #[wasm_bindgen(constructor)]
    pub fn new(x: f64, y: f64, dx: f64, dy: f64, ddx: f64, ddy: f64) -> Waypoint {
        Waypoint {
            point: Vec2 { x, y },
            tangent: Vec2 { x: dx, y: dy },
            curvature: Vec2 { x: ddx, y: ddy },
        }
    }
}
