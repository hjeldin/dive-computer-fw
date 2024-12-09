use dive_deco::{BuehlmannConfig, BuehlmannModel, Depth, Gas, Time, DecoModel};
use embassy_time::Timer;

fn record_travel(model: &mut BuehlmannModel, depth: Depth, rate: f32, gas: &Gas) {
    model.record_travel_with_rate(depth, rate, gas);
}

pub type BananaParam = f32;

pub type BananaParams = (BananaParam, BananaParam, BananaParam, BananaParam, BananaParam, BananaParam);
pub const ZHL_16C_N2_16A_HE_VALUES: [BananaParams; 16] = [
    (4., 1.2599, 0.5050, 1.51, 01.7424, 0.4245),
    (8., 1., 0.6514, 3.02, 1.3830, 0.5747),
    (12.5, 0.8618, 0.7222, 4.72, 1.1919, 0.6527),
    (18.5, 0.7562, 0.7825, 6.99, 1.0458, 0.7223),
    (27., 0.6200, 0.8126, 10.21, 0.9220, 0.7582),
    (38.3, 0.5043, 0.8434, 14.48, 0.8205, 0.7957),
    (54.3, 0.4410, 0.8693, 20.53, 0.7305, 0.8279),
    (77., 0.4000, 0.8910, 29.11, 0.6502, 0.8553),
    (109., 0.3750, 0.9092, 41.2, 0.5950, 0.8757),
    (146., 0.3500, 0.9222, 55.19, 0.5545, 0.8903),
    (187., 0.3295, 0.9319, 70.69, 0.5333, 0.8997),
    (239., 0.3065, 0.9403, 90.34, 0.5189, 0.9073),
    (305., 0.2835, 0.9477, 115.29, 0.5181, 0.9122),
    (390., 0.2610, 0.9544, 147.42, 0.5176, 0.9171),
    (498., 0.2480, 0.9602, 188.24, 0.5172, 0.9217),
    (635., 0.2327, 0.9653, 240.03, 0.5119, 0.9267),
];


#[embassy_executor::task]
pub async fn deco_task() {
    let config = BuehlmannConfig::new().with_gradient_factors(30, 70);
    let mut model = BuehlmannModel::new(config);
    defmt::info!("Prepare dive");
    // bottom gas
    let air = Gas::air();
    // deco gases
    let ean_50 = Gas::new(0.5, 0.);
    let oxygen = Gas::new(1., 0.);
    let mut available_gas_mixes = [Gas::default(); 16];
    available_gas_mixes[0] = air;
    available_gas_mixes[1] = ean_50;
    available_gas_mixes[2] = oxygen;

    defmt::info!("Starting dive");
    let bottom_depth = Depth::from_meters(40.);
    let bottom_time = Time::from_minutes(20.);

    defmt::info!("Descent to 40m at a rate of 9m/min using air");
    let params  = [(4., 1.2599, 0.5050, 1.51, 01.7424, 0.4245); 1];
    ZHL_16C_N2_16A_HE_VALUES.iter().for_each(|compartment| {
        defmt::info!("{}", compartment);
    });
    defmt::info!("banana");
    model.compartments.iter().for_each(|compartment| {
        defmt::info!("{}", compartment.params);
    });
    // defmt::info!("model {:?}", );
    // descent to 40m at a rate of 9min/min using air
    record_travel(&mut model, bottom_depth, 9., &available_gas_mixes[0]);

    defmt::info!("Bottom time of 20 minutes at 40m");

    // // 20 min bottom time
    // model.record(bottom_depth, bottom_time, &air);


    // Timer::after_millis(1000).await;
    let mut i = 0;
    loop {
    //     defmt::info!("Calculating deco runtime");
    //     // calculate deco runtime providing available gasses
    //     let deco_runtime = model.deco(available_gas_mixes);
    //     let data = deco_runtime.unwrap();
    //     defmt::info!("Deco runtime: {:?}", data);
        i += 1;
        defmt::info!("{}", i);
        Timer::after_millis(1000).await;
    }
}
