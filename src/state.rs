use dive_computer_deco::tracker::Guidance;

#[derive(Clone)]
pub struct State {
    pub time: [u8; 4],
    pub pressure: f32,
    pub temperature: f32,
    pub accel_x: f32,
    pub accel_y: f32,
    pub accel_z: f32,
    pub mag_x: f32,
    pub mag_y: f32,
    pub mag_z: f32,
    pub mag_heading: f32,
    pub batt_voltage: f32,
    pub low_batt: bool,
    pub power_good: bool,
    pub charge_status: bool,
    /// Counts MS5837 readings, so a stalled sensor task can be noticed.
    pub pressure_samples: u32,
    /// Depth below the surface, metres.
    pub depth_m: f32,
    pub max_depth_m: f32,
    pub dive_time_s: u32,
    /// Positive when ascending.
    pub ascent_rate_m_min: f32,
    pub diving: bool,
    pub ceiling_m: Option<f32>,
    /// First stop or NDL, as the diver is told.
    pub guidance: Guidance,
    pub tts_min: Option<u16>,
}
