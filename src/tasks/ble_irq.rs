use defmt::info;
use embassy_stm32::exti::ExtiInput;
use embassy_time::Timer;

#[embassy_executor::task]
pub async fn ble_irq_task(mut input: ExtiInput<'static>) {
    info!("BLE IRQ task started, listening for pin changes on PB2");
    
    loop {
        // Wait for rising edge (BLE IRQ is typically active low)
        input.wait_for_rising_edge().await;
        info!("BLE IRQ triggered (rising edge)");
        
        // Small debounce delay
        Timer::after_millis(1).await;
    }
}











