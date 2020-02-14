mod opt;

use opt::*;

use argmin::prelude::*;
use argmin::solver::gradientdescent::SteepestDescent;
use argmin::solver::linesearch::MoreThuenteLineSearch;
use serde::Deserialize;

use cfg_if::cfg_if;
use wasm_bindgen::prelude::*;
use wayfinder::*;

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

#[derive(Deserialize)]
pub struct OptParam {
    pub eps: f64,
    pub samples: u64,
    pub max_iters: u64,
    pub target_cost: f64,
}

#[wasm_bindgen]
extern "C" {
    #[wasm_bindgen(js_namespace = console)]
    fn log(s: &str);
}

#[wasm_bindgen(start)]
pub fn main() {
    set_panic_hook();
}

#[wasm_bindgen]
pub fn wps_to_cheesy_path(wps_value: &JsValue, param_value: &JsValue) -> JsValue {
    let param: Cheesy = param_value.into_serde().unwrap();
    let wps: Vec<Waypoint> = wps_value.into_serde().unwrap();

    let hermites = hermites(&wps);

    let segments = param.parameterize(&hermites);

    JsValue::from_serde(&segments).unwrap()
}

#[wasm_bindgen]
pub fn wps_to_jaci_path(wps_value: &JsValue, param_value: &JsValue) -> JsValue {
    let param: Jaci = param_value.into_serde().unwrap();
    let wps: Vec<Waypoint> = wps_value.into_serde().unwrap();

    let hermites = hermites(&wps);

    let segments = param.parameterize(&hermites);

    JsValue::from_serde(&segments).unwrap()
}

#[wasm_bindgen]
pub fn optimize(wps_value: &JsValue, param_value: &JsValue) -> JsValue {
    let param: OptParam = param_value.into_serde().unwrap();

    let wps: Vec<Waypoint> = wps_value.into_serde().unwrap();

    if wps.len() <= 2 {
        return JsValue::from_serde(&Vec::<f64>::new()).unwrap();
    }

    let init_acc = wps
        .iter()
        .enumerate()
        .filter(|(j, _)| *j > 0 && *j < wps.len() - 1)
        .map(|(_, wp)| wp.curvature.to_vec())
        .flatten()
        .collect();

    let linesearch = MoreThuenteLineSearch::new();

    let solver = SteepestDescent::new(linesearch);

    let cost = OptProblem {
        waypoints: wps,
        eps: param.eps,
        samples: param.samples,
    };

    let res = Executor::new(cost, solver, init_acc)
        .max_iters(param.max_iters)
        .target_cost(param.target_cost)
        .run()
        .unwrap();

    JsValue::from_serde(&res.state).unwrap()
}

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
