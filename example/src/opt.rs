use argmin::prelude::*;
use serde::Serialize;
use wasm_bindgen::prelude::*;
use wayfinder::*;
use nalgebra::Vector2;

#[wasm_bindgen]
extern "C" {
    #[wasm_bindgen(js_namespace = console)]
    fn log(s: &str);
}

#[derive(Clone, Serialize)]
pub struct OptProblem {
    pub waypoints: Vec<Waypoint<f32>>,
    pub eps: f32,
    pub samples: u64,
}

impl ArgminOp for OptProblem {
    type Param = Vec<f32>;
    type Output = f32;
    type Hessian = ();
    type Jacobian = ();
    type Float = f32;

    fn apply(&self, param: &Vec<f32>) -> Result<Self::Output, Error> {
        let mut wps = self.waypoints.clone();
        wps.iter_mut()
            .skip(1)
            .zip(param.chunks(2))
            .for_each(|(wp, acc)| wp.curvature = Vector2::from_column_slice(acc));

        let integral = hermites(&wps)
            .iter()
            .map(|h| h.integral_d_curvature_d_t_squared(self.samples))
            .fold(0.0, |acc, x| acc + x);

        Ok(integral)
    }

    fn gradient(&self, param: &Vec<f32>) -> Result<Vec<f32>, Error> {
        let integral_ref = self.apply(param)?;
        let mut wps = self.waypoints.clone();

        wps.iter_mut()
            .skip(1)
            .zip(param.chunks(2))
            .for_each(|(wp, acc)| wp.curvature = Vector2::from_column_slice(acc));

        let res = (0..param.len())
            .map(|i| {
                let mut wps_forward: Vec<Waypoint<f32>> = wps.clone();

                wps_forward
                    .iter_mut()
                    .enumerate()
                    .filter(|(j, _)| *j == i / 2 + 1)
                    .for_each(|(_, wp)| wp.curvature[i % 2] += self.eps);

                let integral_forward = hermites(&wps_forward)
                    .iter()
                    .map(|h| h.integral_d_curvature_d_t_squared(self.samples))
                    .fold(0.0, |acc, x| acc + x);

                (integral_forward - integral_ref) / self.eps
            })
            .collect();

        Ok(res)
    }
}
