//use argmin::prelude::*;
//use argmin::solver::gradientdescent::SteepestDescent;
//use argmin::solver::linesearch::HagerZhangLineSearch;
use cfg_if::cfg_if;
//use nalgebra::Vector2;
//use serde_wasm_bindgen;
use wasm_bindgen::prelude::*;

mod math;
mod parameterizer;
mod point;
mod spline;
mod waypoint;

pub use parameterizer::*;
pub use point::Point;
pub use spline::Hermite;
pub use waypoint::Waypoint;

cfg_if! {
    // When the `console_error_panic_hook` feature is enabled, we can call the
    // `set_panic_hook` function to get better error messages if we ever panic.
    if #[cfg(feature = "console_error_panic_hook")] {
        extern crate console_error_panic_hook;
        use console_error_panic_hook::set_once as set_panic_hook;
    } else {
        #[inline]
        pub fn set_panic_hook() {}
    }
}

cfg_if! {
    // When the `wee_alloc` feature is enabled, use `wee_alloc` as the global
    // allocator.
    if #[cfg(feature = "wee_alloc")] {
        extern crate wee_alloc;
        #[global_allocator]
        static ALLOC: wee_alloc::WeeAlloc = wee_alloc::WeeAlloc::INIT;
    }
}

/*#[derive(Clone)]
struct OptimizationProblem {
    waypoints: Vec<waypoint::Waypoint>,
    eps: f64,
    samples: u64,
}

impl ArgminOperator for OptimizationProblem {
    type Parameters = Vec<f64>;
    type OperatorOutput = f64;
    type Hessian = ();

    fn apply(&self, param: &Vec<f64>) -> Result<f64, Error> {
        let mut wps = self.waypoints.clone();
        wps.iter_mut()
            .skip(1)
            .zip(param.chunks(2))
            .for_each(|(wp, acc)| wp.tangent_slope = Vector2::new(acc[0], acc[1]));

        let integral = spline::hermite::Hermite::generate(&wps)
            .iter()
            .map(|h| h.integral_d_curvature_d_t_squared(self.samples))
            .fold(0.0, |acc, x| acc + x);

        Ok(integral)
    }

    fn gradient(&self, param: &Vec<f64>) -> Result<Vec<f64>, Error> {
        let integral_ref = self.apply(param)?;

        let mut wps = self.waypoints.clone();

        wps.iter_mut()
            .skip(1)
            .zip(param.chunks(2))
            .for_each(|(wp, acc)| wp.tangent_slope = Vector2::new(acc[0], acc[1]));

        let res = (0..param.len())
            .map(|i| {
                let mut wps_eps: Vec<waypoint::Waypoint> = wps.clone();

                wps_eps
                    .iter_mut()
                    .enumerate()
                    .filter(|(j, _)| *j == i / 2 + 1)
                    .for_each(|(_, wp)| wp.tangent_slope[i % 2] += self.eps);

                let integral_eps = spline::hermite::Hermite::generate(&wps_eps)
                    .iter()
                    .map(|h| h.integral_d_curvature_d_t_squared(self.samples))
                    .fold(0.0, |acc, x| acc + x);

                (integral_eps - integral_ref) / self.eps
            })
            .collect();

        Ok(res)
    }
}*/

#[wasm_bindgen]
extern "C" {
    #[wasm_bindgen(js_namespace = console)]
    fn log(s: &str);
}

#[wasm_bindgen]
pub fn wps_to_path(
    //wps_value: JsValue,     // Vec<Waypoint>
    //param_value: JsValue,   // Param
) { //->  JsValue { // Vec<Point>
    set_panic_hook();
/*
    let wps: Vec<Waypoint> = match serde_wasm_bindgen::from_value(wps_value) {
        Ok(res) => res,
        Err(err) => {
            log(&format!("{}", err));
            return JsValue::NULL;
        }
    };

    let param: Param = match serde_wasm_bindgen::from_value(param_value) {
        Ok(res) => res,
        Err(err) => {
            log(&format!("{}", err));
            return JsValue::NULL;
        }
    };
*/
    let param = Cheesy::new(1.0, 1.0, 3.);
    let wps = vec![
        Waypoint::new(0.,0.,10.,0.,0.,0.),
        Waypoint::new(100., 100., 0., 10., 0., 0.)
    ];

    let hermites = Hermite::hermites(&wps);

    for hermite in &hermites {
        log(&format!("{:?}", hermite));
    }

    
    let segments = param.parameterize(&hermites);

    log(&format!("Len: {}", segments.len()));
    /*for segment in segments {
        log(&format!("{:?}", segment));
    }*/
    
/*
    let segments = match param {
        Param::ParamCheesy(cheesy) => cheesy.parameterize(hermites),
        Param::ParamJaci(jaci) => jaci.parameterize(hermites),
    };

    //Some(param.parameterize(hermites))

    match serde_wasm_bindgen::to_value(&segments) {
        Ok(segments_value) => segments_value,
        Err(err) => {
            log(&format!("{}", err));
            return JsValue::NULL;
        }
    }
*/
}
/*
#[wasm_bindgen]
pub fn optimize(wp_repr: &mut [f64]) -> Vec<f64> {
    set_panic_hook();

    let wps: Vec<waypoint::Waypoint> = wp_repr
        .chunks_mut(6)
        .map(|chunk| waypoint::Waypoint::from_array(chunk))
        .collect();

    if wps.len() <= 2 {
        return Vec::new();
    }

    let cost = OptimizationProblem {
        waypoints: wps.clone(),
        eps: 1e-5,
        samples: 100,
    };

    let init_acc = wps
        .iter()
        .enumerate()
        .filter(|(j, _)| *j > 0 && *j < wps.len() - 1)
        .map(|(_, wp)| wp.tangent_slope.as_slice().to_vec())
        .flatten()
        .collect();

    let mut linesearch = HagerZhangLineSearch::new();

    let mut solver = SteepestDescent::new(linesearch);
    //solver.set_linesearch(Box::new(linesearch));
    //solver.set_max_iters(100);
    solver.set_min_improvement(0.00001);

    solver.run_fast().unwrap();

    log(&format!("{:?}", solver.result()));

    solver.unwrap()
}
*/
