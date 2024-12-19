use dive_deco_x86::*;
use embassy_time::Timer;

#[embassy_executor::task]
pub async fn deco_task() {
    let mut tissues = [Tissue::default(); 16];
    let temperature = 20.0;
    let mut amb_pressure = 1.0;
    for i in 0..tissues.len() {
        tissues[i].load_n2 = (amb_pressure - water_vapor_pressure(temperature)) * FN2;
        tissues[i].load_he = (amb_pressure - water_vapor_pressure(temperature)) * FHE;
    }
    let time_since_last_check = 1.0; // minutes
    let mut i = 0;
    loop {
        if amb_pressure < 5.0 {
            amb_pressure += 0.1;
        }
        let result = run_deco_loop(&mut tissues, amb_pressure, temperature, 1.0 / 60.0);
        defmt::info!("{:?}", result);
        defmt::info!("pressure {:?}", amb_pressure);
        Timer::after_millis(1000).await;
    }
}
