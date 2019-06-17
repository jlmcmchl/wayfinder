mod opt;

use opt::*;

use argmin::prelude::*;
use argmin::solver::gradientdescent::SteepestDescent;
use argmin::solver::linesearch::HagerZhangLineSearch;

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
pub fn wps_to_path(//wps_value: JsValue,     // Vec<Waypoint>
    //param_value: JsValue,   // Param
) {
    //->  JsValue { // Vec<Point>

    let param = Cheesy::new(1., 1., 5.);
    let wps = vec![
        Waypoint::new(0., 0., 10., 0., 0., 0.),
        Waypoint::new(10., 10., 0., 10., 0., 0.),
    ];

    let hermites = hermites(&wps);

    for hermite in &hermites {
        log(&format!("{:?}", hermite));
    }

    let segments = param.parameterize(&hermites);

    log(&format!("Len: {}", segments.len()));
    for segment in segments {
        log(&format!("{:?}", segment));
    }
}

#[wasm_bindgen]
pub fn optimize() {
    //wp_repr: &mut [f64]) -> Vec<f64> {
    /*let wps: Vec<waypoint::Waypoint> = wp_repr
            .chunks_mut(6)
            .map(|chunk| waypoint::Waypoint::from_array(chunk))
            .collect();
    */

    let wps = vec![
        Waypoint::new(0., 0., 10., 0., 0., 0.),
        Waypoint::new(10., 10., 0., 10., 0., 0.),
        Waypoint::new(20., 20., 10., 0., 0., 0.),
        Waypoint::new(30., 30., 0., 10., 0., 0.),
    ];

    if wps.len() <= 2 {
        return; // Vec::new();
    }

    let init_acc = wps
        .iter()
        .enumerate()
        .filter(|(j, _)| *j > 0 && *j < wps.len() - 1)
        .map(|(_, wp)| wp.curvature.to_vec())
        .flatten()
        .collect();

    let linesearch = HagerZhangLineSearch::new();

    let solver = SteepestDescent::new(linesearch);

    let cost = OptProblem {
        waypoints: wps.clone(),
        eps: 1e-2,
        samples: 10,
    };

    log("Optimizing");

    let res = match Executor::new(cost, solver, init_acc)
        .max_iters(100)
        .target_cost(0.001)
        .run()
    {
        Ok(result) => result,
        Err(err) => panic!("{:#?}", err),
    };

    log(&format!("{}", res));
}

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
