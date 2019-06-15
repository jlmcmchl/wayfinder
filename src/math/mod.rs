use std::ops::Index;
use std::ops::{Div, Mul, Sub};
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
#[derive(Clone, Copy, Debug)]
pub struct Vec2 {
    pub x: f64,
    pub y: f64,
}
impl Index<i32> for Vec2 {
    type Output = f64;

    fn index(&self, index: i32) -> &Self::Output {
        match index {
            0 => &self.x,
            1 => &self.y,
            _ => panic!("Index {} is outside range 0..=1", index),
        }
    }
}

#[wasm_bindgen]
impl Vec2 {
    #[wasm_bindgen(constructor)]
    pub fn new(x: f64, y: f64) -> Self {
        Vec2 { x, y }
    }

    pub fn unit(self) -> Self {
        let den = self.norm();
        Vec2::new(self[0] / den, self[1] / den)
    }

    pub fn norm(self) -> f64 {
        self.norm_squared().sqrt()
    }

    pub fn norm_squared(self) -> f64 {
        self.x.powi(2) + self.y.powi(2)
    }
}

impl Sub for Vec2 {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Vec2::new(self[0] - rhs[0], self[1] - rhs[1])
    }
}

impl Mul<f64> for Vec2 {
    type Output = Self;

    fn mul(self, rhs: f64) -> Self::Output {
        Vec2::new(self[0] * rhs, self[1] * rhs)
    }
}

impl Mul<Vec2> for Vec2 {
    type Output = f64;

    fn mul(self, rhs: Vec2) -> Self::Output {
        self[0] * rhs[0] + self[1] * rhs[1]
    }
}

impl Div<f64> for Vec2 {
    type Output = Self;

    fn div(self, rhs: f64) -> Self::Output {
        Vec2::new(self[0] / rhs, self[1] / rhs)
    }
}

#[derive(Clone, Copy)]
pub struct Vec6 {
    x: f64,
    y: f64,
    z: f64,
    a: f64,
    b: f64,
    c: f64,
}

impl Index<i32> for Vec6 {
    type Output = f64;

    fn index(&self, index: i32) -> &Self::Output {
        match index {
            0 => &self.x,
            1 => &self.y,
            2 => &self.z,
            3 => &self.a,
            4 => &self.b,
            5 => &self.c,
            _ => panic!("Index {} is outside range 0..=5", index),
        }
    }
}

impl Vec6 {
    pub fn new(x: f64, y: f64, z: f64, a: f64, b: f64, c: f64) -> Self {
        Vec6 { x, y, z, a, b, c }
    }
}

impl Mul for Vec6 {
    type Output = f64;

    fn mul(self, rhs: Self) -> Self::Output {
        self[0] * rhs[0]
            + self[1] * rhs[1]
            + self[2] * rhs[2]
            + self[3] * rhs[3]
            + self[4] * rhs[4]
            + self[5] * rhs[5]
    }
}

/*
      0 1 2 3 4 5 THEN 0 1
    [ 0 0 0 0 0 0
      0 0 0 0 0 0]

*/
#[derive(Clone, Debug)]
pub struct Vec2x6 {
    inner: Vec<Vec2>,
}

impl Index<i32> for Vec2x6 {
    type Output = Vec2;

    fn index(&self, index: i32) -> &Self::Output {
        match index {
            0 | 1 | 2 | 3 | 4 | 5 => &self.inner[index as usize],
            _ => panic!("Index {} is outside range 0..=5", index),
        }
    }
}

impl Vec2x6 {
    pub fn new(v0: Vec2, v1: Vec2, v2: Vec2, v3: Vec2, v4: Vec2, v5: Vec2) -> Self {
        Vec2x6 {
            inner: vec![v0, v1, v2, v3, v4, v5],
        }
    }

    pub fn upper(&self) -> Vec6 {
        Vec6 {
            x: self[0][0],
            y: self[1][0],
            z: self[2][0],
            a: self[3][0],
            b: self[4][0],
            c: self[5][0],
        }
    }

    pub fn lower(&self) -> Vec6 {
        Vec6 {
            x: self[0][1],
            y: self[1][1],
            z: self[2][1],
            a: self[3][1],
            b: self[4][1],
            c: self[5][1],
        }
    }
}

impl Mul<Vec6> for Vec2x6 {
    type Output = Vec2;

    fn mul(self, rhs: Vec6) -> Self::Output {
        Vec2 {
            x: self[0][0] * rhs[0]
                + self[1][0] * rhs[1]
                + self[2][0] * rhs[2]
                + self[3][0] * rhs[3]
                + self[4][0] * rhs[4]
                + self[5][0] * rhs[5],
            y: self[0][1] * rhs[0]
                + self[1][1] * rhs[1]
                + self[2][1] * rhs[2]
                + self[3][1] * rhs[3]
                + self[4][1] * rhs[4]
                + self[5][1] * rhs[5],
        }
    }
}

impl Mul<Vec6x6> for Vec2x6 {
    type Output = Vec2x6;

    fn mul(self, rhs: Vec6x6) -> Self::Output {
        Vec2x6::new(
            Vec2 {
                x: self.upper() * rhs[0],
                y: self.lower() * rhs[0],
            },
            Vec2 {
                x: self.upper() * rhs[1],
                y: self.lower() * rhs[1],
            },
            Vec2 {
                x: self.upper() * rhs[2],
                y: self.lower() * rhs[2],
            },
            Vec2 {
                x: self.upper() * rhs[3],
                y: self.lower() * rhs[3],
            },
            Vec2 {
                x: self.upper() * rhs[4],
                y: self.lower() * rhs[4],
            },
            Vec2 {
                x: self.upper() * rhs[5],
                y: self.lower() * rhs[5],
            },
        )
    }
}

pub struct Vec6x6 {
    inner: Vec<Vec6>,
}

impl Index<i32> for Vec6x6 {
    type Output = Vec6;

    fn index(&self, index: i32) -> &Self::Output {
        match index {
            0 | 1 | 2 | 3 | 4 | 5 => &self.inner[index as usize],
            _ => panic!("Index {} is outside range 0..=5", index),
        }
    }
}

impl Vec6x6 {
    pub fn new(v0: Vec6, v1: Vec6, v2: Vec6, v3: Vec6, v4: Vec6, v5: Vec6) -> Self {
        Vec6x6 {
            inner: vec![v0, v1, v2, v3, v4, v5],
        }
    }
}

pub struct Vec2x2 {
    inner: Vec<Vec2>,
}

impl Index<i32> for Vec2x2 {
    type Output = Vec2;

    fn index(&self, index: i32) -> &Self::Output {
        match index {
            0 | 1 => &self.inner[index as usize],
            _ => panic!("Index {} is outside range 0..=1", index),
        }
    }
}

impl Vec2x2 {
    pub fn new(x0: f64, y0: f64, x1: f64, y1: f64) -> Self {
        Vec2x2 {
            inner: vec![Vec2 { x: x0, y: x1 }, Vec2 { x: y0, y: y1 }],
        }
    }

    pub fn x(&self) -> Vec2 {
        self[0].clone()
    }

    pub fn y(&self) -> Vec2 {
        self[1].clone()
    }

    pub fn determinant(&self) -> f64 {
        self.x().x * self.y().y - self.y().x * self.x().y
    }

    pub fn inverse(&self) -> Option<Vec2x2> {
        let det = self.determinant();

        if det == 0. {
            return None;
        }

        Some(Vec2x2::new(
            self.y().y / det,
            -self.x().y / det,
            -self.y().x / det,
            self.x().x / det,
        ))
    }
}

impl Mul<Vec2> for Vec2x2 {
    type Output = Vec2;

    fn mul(self, rhs: Vec2) -> Self::Output {
        Vec2::new(
            self.x().x * rhs.x + self.y().x * rhs.y,
            self.x().y * rhs.x + self.y().y * rhs.y,
        )
    }
}

pub trait Rotation {
    fn inverse(self) -> Self;
    fn rotate_by(self, other: Self) -> Self;
    fn as_radians(self) -> f64;
    fn from_radians(rad: f64) -> Self;
}

impl Rotation for Vec2 {
    fn inverse(self) -> Vec2 {
        Vec2::new(self.x, -self.y)
    }

    fn rotate_by(self, r: Vec2) -> Vec2 {
        Vec2::from_radians(self.as_radians() + r.as_radians())
    }

    fn as_radians(self) -> f64 {
        self.y.atan2(self.x)
    }

    fn from_radians(rad: f64) -> Self {
        let (x, y) = rad.sin_cos();
        Vec2 { x, y }
    }
}

pub trait Translation {
    fn rotate_by(self, other: Self) -> Self;
}

impl Translation for Vec2 {
    fn rotate_by(self, r: Vec2) -> Vec2 {
        Vec2 {
            x: self.x * r.x - self.y * r.y,
            y: self.x * r.y + self.y * r.x,
        }
    }
}

pub struct Pose {
    t: Vec2,
    r: Vec2,
}

impl Pose {
    pub fn new(t: Vec2, r: Vec2) -> Self {
        Pose { t, r }
    }

    pub fn log(self) -> Twist {
        let dt = self.r.as_radians();
        let half_dt = dt / 2.;
        let cos_minus_one = self.r.x - 1.;
        let halftheta_by_tan_of_halfdtheta = if cos_minus_one.abs() < 1e-9 {
            1. - 1. / 12. * dt * dt
        } else {
            -(half_dt * self.r.y) / cos_minus_one
        };

        let rot = Vec2::new(halftheta_by_tan_of_halfdtheta, -half_dt).unit();

        let t_part = Translation::rotate_by(self.t, rot);

        Twist {
            ds: t_part,
            dt: rot,
        }
    }
}

pub struct Twist {
    ds: Vec2,
    dt: Vec2,
}

impl Twist {
    pub fn dx(&self) -> f64 {
        self.ds.x
    }

    pub fn dy(&self) -> f64 {
        self.ds.y
    }

    pub fn dt(&self) -> f64 {
        self.dt.as_radians()
    }
}
