use crate::math::Vec2;
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
#[derive(Debug)]
pub struct Point {
    pub position: Vec2,
    pub velocity: Vec2,
    pub acceleration: Vec2,
    pub jerk: Vec2,
}

#[wasm_bindgen]
impl Point {
    pub fn heading(&self) -> Vec2 {
        self.velocity
    }

    pub fn curvature(&self) -> f64 {
        (self.velocity.x * self.acceleration.y - self.velocity.y * self.acceleration.x)
            / self.velocity.norm().powi(3)
    }
}
