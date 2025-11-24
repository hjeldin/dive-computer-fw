pub struct State {
    pub time: [u8; 4],
    pub pressure: f32,
    pub temperature: f32,
    pub accel_x: f32,
    pub accel_y: f32,
    pub accel_z: f32,
    pub gyro_x: f32,
    pub gyro_y: f32,
    pub gyro_z: f32,
}
