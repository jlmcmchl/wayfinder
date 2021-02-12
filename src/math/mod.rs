use nalgebra::{Matrix2, Point2, Rotation2, Scalar, Translation2, UnitComplex};

pub struct Pose<N: Scalar> {
    t: Point2<N>,
    r: UnitComplex<N>,
}

impl Pose<f32> {
    pub fn new(t: Point2<f32>, r: Rotation2<f32>) -> Self {
        Pose {
            t,
            r: UnitComplex::from_rotation_matrix(&r),
        }
    }

    pub fn log(self) -> Twist<f32> {
        let dt = self.r.angle();
        let half_dt = dt / 2.;
        let cos_minus_one = self.r.cos_angle() - 1.;
        let halftheta_by_tan_of_halfdtheta = if cos_minus_one.abs() < 1e-9 {
            1. - 1. / 12. * dt * dt
        } else {
            -(half_dt * self.r.sin_angle()) / cos_minus_one
        };

        let rot = UnitComplex::from_rotation_matrix(&Rotation2::from_matrix(
            &Matrix2::from_column_slice(&[
                halftheta_by_tan_of_halfdtheta,
                -half_dt,
                half_dt,
                halftheta_by_tan_of_halfdtheta,
            ]),
        ));

        let t_part = Translation2::new(
            self.t.x * rot.cos_angle() - self.t.y * rot.sin_angle(),
            self.t.x * rot.sin_angle() + self.t.y * rot.cos_angle(),
        );

        Twist {
            ds: t_part,
            dt: rot.into(),
        }
    }
}

pub struct Twist<N: Scalar> {
    ds: Translation2<N>,
    dt: Rotation2<N>,
}

impl Twist<f32> {
    pub fn dx(&self) -> f32 {
        self.ds.x
    }

    pub fn dy(&self) -> f32 {
        self.ds.y
    }

    pub fn dt(&self) -> f32 {
        self.dt.angle()
    }
}
