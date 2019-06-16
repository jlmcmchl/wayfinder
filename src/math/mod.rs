pub trait Vector {
    fn add(&self, rhs: Self) -> Self;
    fn scale(&self, rhs: f64) -> Self;
    fn dot(&self, rhs: &Self) -> f64;

    fn norm_squared(&self) -> f64 {
        self.dot(self)
    }

    fn norm(&self) -> f64 {
        self.norm_squared().sqrt()
    }

    fn unit(&self) -> Self
    where
        Self: Sized,
    {
        let scalar = 1. / self.norm();
        self.scale(scalar)
    }
}

pub trait Coordinate {
    fn x(&self) -> f64;
    fn y(&self) -> f64;
}

pub type Vec2 = [f64; 2];

impl Vector for Vec2 {
    fn add(&self, rhs: Self) -> Self {
        [self[0] + rhs[0], self[1] + rhs[1]]
    }

    fn scale(&self, rhs: f64) -> Self {
        [self[0] * rhs, self[1] * rhs]
    }

    fn dot(&self, rhs: &Self) -> f64 {
        self[0] * rhs[0] + self[1] * rhs[1]
    }
}

impl Coordinate for Vec2 {
    fn x(&self) -> f64 {
        self[0]
    }

    fn y(&self) -> f64 {
        self[1]
    }
}

pub type Vec6 = [f64; 6];

impl Vector for Vec6 {
    fn add(&self, rhs: Self) -> Self {
        [
            self[0] + rhs[0],
            self[1] + rhs[1],
            self[2] + rhs[2],
            self[3] + rhs[3],
            self[4] + rhs[4],
            self[5] + rhs[5],
        ]
    }

    fn scale(&self, rhs: f64) -> Self {
        [
            self[0] * rhs,
            self[1] * rhs,
            self[2] * rhs,
            self[3] * rhs,
            self[4] * rhs,
            self[5] * rhs,
        ]
    }

    fn dot(&self, rhs: &Self) -> f64 {
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

pub trait Matrix {
    type Row;
    type Col;
    fn row(&self, index: usize) -> Self::Row;
    fn col(&self, index: usize) -> Self::Col;

    fn determinant(&self) -> f64 {
        0.
    }

    fn inverse(&self) -> Option<Self>
    where
        Self: Sized,
    {
        None
    }
}

pub trait MatMul<T> {
    type Output;

    fn mul(&self, other: &T) -> Self::Output;
}

pub type Mat2x6 = [Vec2; 6];

impl Matrix for Mat2x6 {
    type Row = Vec6;
    type Col = Vec2;

    fn row(&self, index: usize) -> Self::Row {
        [
            self[0][index],
            self[1][index],
            self[2][index],
            self[3][index],
            self[4][index],
            self[5][index],
        ]
    }

    fn col(&self, index: usize) -> Self::Col {
        self[index]
    }
}

impl MatMul<Vec6> for Mat2x6 {
    type Output = Vec2;

    fn mul(&self, other: &Vec6) -> Self::Output {
        [
            self[0][0] * other[0]
                + self[1][0] * other[1]
                + self[2][0] * other[2]
                + self[3][0] * other[3]
                + self[4][0] * other[4]
                + self[5][0] * other[5],
            self[0][1] * other[0]
                + self[1][1] * other[1]
                + self[2][1] * other[2]
                + self[3][1] * other[3]
                + self[4][1] * other[4]
                + self[5][1] * other[5],
        ]
    }
}

pub type Mat6 = [Vec6; 6];

impl Matrix for Mat6 {
    type Row = Vec6;
    type Col = Vec6;

    fn row(&self, index: usize) -> Self::Row {
        [
            self[index][0],
            self[index][1],
            self[index][2],
            self[index][3],
            self[index][4],
            self[index][5],
        ]
    }

    fn col(&self, index: usize) -> Self::Col {
        self[index]
    }
}

impl MatMul<Mat6> for Mat2x6 {
    type Output = Mat2x6;

    fn mul(&self, rhs: &Mat6) -> Self::Output {
        let up = self.row(0);
        let down = self.row(1);
        [
            [up.dot(&rhs[0]), down.dot(&rhs[0])],
            [up.dot(&rhs[1]), down.dot(&rhs[1])],
            [up.dot(&rhs[2]), down.dot(&rhs[2])],
            [up.dot(&rhs[3]), down.dot(&rhs[3])],
            [up.dot(&rhs[4]), down.dot(&rhs[4])],
            [up.dot(&rhs[5]), down.dot(&rhs[5])],
        ]
    }
}

pub type Mat2 = [Vec2; 2];

impl Matrix for Mat2 {
    type Row = Vec2;
    type Col = Vec2;

    fn row(&self, index: usize) -> Self::Row {
        [self[index][0], self[index][1]]
    }

    fn col(&self, index: usize) -> Self::Col {
        self[index]
    }

    fn determinant(&self) -> f64 {
        self[0][0] * self[1][1] - self[1][0] * self[0][1]
    }

    fn inverse(&self) -> Option<Self> {
        let det = self.determinant();

        if det == 0. {
            return None;
        }

        Some([
            [self[1][1] / det, -self[0][1] / det],
            [-self[1][0] / det, self[0][0] / det],
        ])
    }
}

impl MatMul<Vec2> for Mat2 {
    type Output = Vec2;

    fn mul(&self, rhs: &Vec2) -> Self::Output {
        let up = self.row(0);
        let down = self.row(1);
        [up.dot(rhs), down.dot(rhs)]
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
        [self.x(), -self.y()]
    }

    fn rotate_by(self, r: Vec2) -> Vec2 {
        Vec2::from_radians(self.as_radians() + r.as_radians())
    }

    fn as_radians(self) -> f64 {
        self.y().atan2(self.x())
    }

    fn from_radians(rad: f64) -> Self {
        let (x, y) = rad.sin_cos();
        [x, y]
    }
}

pub trait Translation {
    fn rotate_by(self, other: Self) -> Self;
}

impl Translation for Vec2 {
    fn rotate_by(self, r: Vec2) -> Vec2 {
        [
            self.x() * r.x() - self.y() * r.y(),
            self.x() * r.y() + self.y() * r.x(),
        ]
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
        let cos_minus_one = self.r.x() - 1.;
        let halftheta_by_tan_of_halfdtheta = if cos_minus_one.abs() < 1e-9 {
            1. - 1. / 12. * dt * dt
        } else {
            -(half_dt * self.r.y()) / cos_minus_one
        };

        let rot = [halftheta_by_tan_of_halfdtheta, -half_dt].unit();

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
        self.ds.x()
    }

    pub fn dy(&self) -> f64 {
        self.ds.y()
    }

    pub fn dt(&self) -> f64 {
        self.dt.as_radians()
    }
}
