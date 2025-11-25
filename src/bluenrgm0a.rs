use embassy_stm32::gpio::Output;
use embassy_stm32::mode::Async;
use embassy_stm32::spi::mode::Master as SpiMaster;
use embassy_sync::mutex::Mutex;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_time::Timer;
use defmt::{info, warn, error};
use core::convert::TryInto;
use crate::state::State;

pub struct BLUENRGM0A {
    driver: &'static Mutex<ThreadModeRawMutex, embassy_stm32::spi::Spi<'static, Async, SpiMaster>>,
    rst: &'static Mutex<ThreadModeRawMutex, Output<'static>>,
    cs: &'static Mutex<ThreadModeRawMutex, Output<'static>>,
}

impl BLUENRGM0A {
    pub fn new(
        driver: &'static Mutex<ThreadModeRawMutex, embassy_stm32::spi::Spi<'static, Async, SpiMaster>>,
        rst: &'static Mutex<ThreadModeRawMutex, Output<'static>>,
        cs: &'static Mutex<ThreadModeRawMutex, Output<'static>>,
    ) -> Self {
        BLUENRGM0A {
            driver,
            rst,
            cs,
        }
    }

    /// Reset the BlueNRG-M0A module
    pub async fn reset(&self) {
        info!("Resetting BlueNRG-M0A...");
        let mut rst = self.rst.try_lock().unwrap();
        rst.set_low();
        Timer::after_millis(10).await;
        rst.set_high();
        // Wait longer for module to stabilize after reset
        Timer::after_millis(500).await;
        info!("Reset complete");
    }
    
    /// Test basic SPI communication with the BlueNRG-M0A module
    /// This verifies that the module is responding before attempting BLE initialization
    pub async fn test_communication(&self) -> Result<bool, AciError> {
        info!("Testing BlueNRG-M0A communication...");
        // Poll SPI header multiple times to check for proper response
        let mut ready_count = 0;
        let mut write_size_samples = [0u8; 20];
        let mut read_size_samples = [0u8; 20];
        
        for i in 0..20 {
            let (spi_ready, write_size, read_size) = self.read_status();
            write_size_samples[i] = write_size;
            read_size_samples[i] = read_size;
            
            if spi_ready == 0x02 {
                ready_count += 1;
                info!("Status sample {}: SPI_READY=0x{:02X}, write_buffer={}, read_buffer={}", 
                      i, spi_ready, write_size, read_size);
            } else {
                warn!("Status sample {}: SPI_READY=0x{:02X} (not ready), write_buffer={}, read_buffer={}", 
                      i, spi_ready, write_size, read_size);
            }
            
            Timer::after_millis(50).await;
        }
        
        info!("Communication test results:");
        info!("  Total samples: 20");
        info!("  SPI_READY=0x02 count: {}", ready_count);
        info!("  Write buffer sizes: {:?}", write_size_samples);
        info!("  Read buffer sizes: {:?}", read_size_samples);
        
        // Check if module ever becomes ready
        if ready_count == 0 {
            error!("No SPI_READY=0x02 response from BlueNRG-M0A");
            error!("Possible issues:");
            error!("  1. CS (chip select) not controlled properly");
            error!("  2. SPI wiring incorrect (MISO not connected)");
            error!("  3. Module not powered");
            error!("  4. Module not responding after reset");
            error!("  5. SPI mode/polarity/phase incorrect");
            return Ok(false);
        }
        
        info!("Communication test PASSED - module is responding correctly");
        Ok(true)
    }

    /// Poll SPI header to check buffer sizes
    /// Returns: (spi_ready, write_buffer_size, read_buffer_size)
    /// According to BlueNRG-MS protocol:
    /// - Master sends: [CTRL (0x0B for read), 0x00, 0x00, 0x00, 0x00]
    /// - Slave returns: [SPI_READY (0x02), write_buffer_size, 0x00, read_buffer_size, 0x00]
    /// If SPI_READY is not 0x02, the transaction should be aborted
    fn poll_spi_header(&self, ctrl_byte: u8) -> Result<(u8, u8, u8), AciError> {
        let mut cs = self.cs.try_lock().unwrap();
        let mut spi = self.driver.try_lock().unwrap();
    
        // SPI frame header: [CTRL byte, 0x00, 0x00, 0x00, 0x00]
        let tx_header = [ctrl_byte, 0x00, 0x00, 0x00, 0x00];
        let mut rx_header = [0u8; 5];
    
        cs.set_low();
        let transfer_ok = spi.blocking_transfer(&mut rx_header, &tx_header).is_ok();
        cs.set_high();
    
        if !transfer_ok {
            return Err(AciError::SpiError);
        }
    
        let spi_ready = rx_header[0];
        let write_buffer_size = rx_header[1];
        let read_buffer_size = rx_header[3];
    
        // Check if SPI is ready (must be 0x02)
        if spi_ready != 0x02 {
            warn!("SPI not ready: 0x{:02X} (expected 0x02)", spi_ready);
            return Err(AciError::InvalidResponse);
        }
    
        Ok((spi_ready, write_buffer_size, read_buffer_size))
    }

    /// Read status byte from SPI (0x02 = ready for command)
    /// Returns: (spi_ready, write_buffer_size, read_buffer_size)
    /// This is a convenience wrapper around poll_spi_header with CTRL=0x0B (read)
    fn read_status(&self) -> (u8, u8, u8) {
        match self.poll_spi_header(0x0B) {
            Ok((ready, write_size, read_size)) => (ready, write_size, read_size),
            Err(_) => (0x00, 0x00, 0x00),
        }
    }
    

    /// Wait for module to be ready (SPI_READY = 0x02)
    async fn wait_ready(&self) {
        let mut attempts = 0;
        loop {
            let (spi_ready, _write_size, _read_size) = self.read_status();
            // SPI_READY must be exactly 0x02 to indicate ready
            if spi_ready == 0x02 {
                break;
            }
            attempts += 1;
            if attempts % 100 == 0 {
                warn!("Waiting for ready, spi_ready=0x{:02X}, attempts={}", spi_ready, attempts);
            }
            Timer::after_millis(1).await;
        }
    }

    /// Send a command packet and wait for response
    /// Implements BlueNRG-MS SPI protocol:
    /// 1. Send header with CTRL=0x0A (write) + command data in one transaction
    /// 2. Check SPI_READY and write_buffer_size from header response
    /// 3. Verify command fits within write_buffer_size
    async fn send_command(&self, cmd: &[u8]) -> Result<(), AciError> {
        self.wait_ready().await;
        
        // Extract opcode from command: [opcode_lo, opcode_hi, ...]
        if cmd.len() < 2 {
            return Err(AciError::InvalidResponse);
        }
        let opcode = cmd[0] as u16 | ((cmd[1] as u16) << 8);
        
        info!("Sending command: opcode=0x{:04X}, len={}", opcode, cmd.len());
        
        // Send header + command data in a single SPI transaction
        // Format: [CTRL (0x0A), 0x00, 0x00, 0x00, 0x00] + cmd_data
        {
            let mut cs = self.cs.try_lock().unwrap();
            let mut spi = self.driver.try_lock().unwrap();
            
            // Build complete frame: header + command data
            let mut tx_frame = heapless::Vec::<u8, 260>::new();
            tx_frame.push(0x0A).map_err(|_| AciError::TooLarge)?; // CTRL byte for write
            tx_frame.push(0x00).map_err(|_| AciError::TooLarge)?; // Dummy byte 1
            tx_frame.push(0x00).map_err(|_| AciError::TooLarge)?; // Dummy byte 2
            tx_frame.push(0x00).map_err(|_| AciError::TooLarge)?; // Dummy byte 3
            tx_frame.push(0x00).map_err(|_| AciError::TooLarge)?; // Dummy byte 4
            tx_frame.extend_from_slice(cmd).map_err(|_| AciError::TooLarge)?; // Command data
            
            // Receive header response + dummy bytes for command data
            let mut rx_frame = heapless::Vec::<u8, 260>::new();
            rx_frame.resize(5 + cmd.len(), 0).map_err(|_| AciError::TooLarge)?;
            
            cs.set_low();
            let result = spi.blocking_transfer(rx_frame.as_mut_slice(), tx_frame.as_slice()).map_err(|_| AciError::SpiError);
            cs.set_high();
            result?;
            
            // Check SPI_READY from header response (must be 0x02)
            if rx_frame[0] != 0x02 {
                warn!("SPI not ready after command send: 0x{:02X}", rx_frame[0]);
                return Err(AciError::InvalidResponse);
            }
            
            // Check write_buffer_size to ensure command was accepted
            let write_buffer_size = rx_frame[1];
            if cmd.len() > write_buffer_size as usize {
                warn!("Command may be too large: {} bytes, write_buffer_size={}", cmd.len(), write_buffer_size);
                // Note: We already sent it, but log a warning
            }
        }
        
        // Small delay to allow command to be processed
        Timer::after_millis(5).await;
        
        // Wait for Command Complete event (SPI lock is now released)
        let (status, _data) = self.wait_command_complete(opcode, 2000).await?;
        
        if status != 0x00 {
            // Status 0x00 = success, non-zero = error
            warn!("Command failed: opcode=0x{:04X}, status=0x{:02X}", opcode, status);
            return Err(AciError::CommandFailed(status));
        }
        
        info!("Command successful: opcode=0x{:04X}", opcode);
        Ok(())
    }
    
    /// Send a command and return the response data
    /// Implements BlueNRG-MS SPI protocol similar to send_command
    async fn send_command_with_response(&self, cmd: &[u8]) -> Result<heapless::Vec<u8, 255>, AciError> {
        self.wait_ready().await;
        
        // Extract opcode from command: [opcode_lo, opcode_hi, ...]
        if cmd.len() < 2 {
            return Err(AciError::InvalidResponse);
        }
        let opcode = cmd[0] as u16 | ((cmd[1] as u16) << 8);
        
        // Send header + command data in a single SPI transaction
        // Format: [CTRL (0x0A), 0x00, 0x00, 0x00, 0x00] + cmd_data
        {
            let mut cs = self.cs.try_lock().unwrap();
            let mut spi = self.driver.try_lock().unwrap();
            
            // Build complete frame: header + command data
            let mut tx_frame = heapless::Vec::<u8, 260>::new();
            tx_frame.push(0x0A).map_err(|_| AciError::TooLarge)?; // CTRL byte for write
            tx_frame.push(0x00).map_err(|_| AciError::TooLarge)?; // Dummy byte 1
            tx_frame.push(0x00).map_err(|_| AciError::TooLarge)?; // Dummy byte 2
            tx_frame.push(0x00).map_err(|_| AciError::TooLarge)?; // Dummy byte 3
            tx_frame.push(0x00).map_err(|_| AciError::TooLarge)?; // Dummy byte 4
            tx_frame.extend_from_slice(cmd).map_err(|_| AciError::TooLarge)?; // Command data
            
            // Receive header response + dummy bytes for command data
            let mut rx_frame = heapless::Vec::<u8, 260>::new();
            rx_frame.resize(5 + cmd.len(), 0).map_err(|_| AciError::TooLarge)?;
            
            cs.set_low();
            let result = spi.blocking_transfer(rx_frame.as_mut_slice(), tx_frame.as_slice()).map_err(|_| AciError::SpiError);
            cs.set_high();
            result?;
            
            // Check SPI_READY from header response (must be 0x02)
            if rx_frame[0] != 0x02 {
                warn!("SPI not ready after command send: 0x{:02X}", rx_frame[0]);
                return Err(AciError::InvalidResponse);
            }
            
            // Check write_buffer_size to ensure command was accepted
            let write_buffer_size = rx_frame[1];
            if cmd.len() > write_buffer_size as usize {
                warn!("Command may be too large: {} bytes, write_buffer_size={}", cmd.len(), write_buffer_size);
                // Note: We already sent it, but log a warning
            }
        }
        
        // Small delay to allow command to be processed
        Timer::after_millis(5).await;
        
        // Wait for Command Complete event (SPI lock is now released)
        let (status, data) = self.wait_command_complete(opcode, 2000).await?;
        
        if status != 0x00 {
            // Status 0x00 = success, non-zero = error
            return Err(AciError::CommandFailed(status));
        }
        
        Ok(data)
    }

    /// Read event packet using BlueNRG-MS SPI protocol
    /// Format per UM1865: [Event Code (1 byte), Parameter Length (1 byte), Parameters...]
    async fn read_event(&self) -> Option<heapless::Vec<u8, 255>> {
        // Step 1: Poll header with CTRL=0x0B (read) to check read_buffer_size
        let (_spi_ready, _write_buffer_size, read_buffer_size) = match self.poll_spi_header(0x0B) {
            Ok(result) => result,
            Err(_) => return None,
        };
        
        // Check if there's data to read
        if read_buffer_size == 0 {
            return None;
        }
        
        // Step 2: Read the event data
        // According to protocol, we send header + read data in one transaction
        // But since we already sent the header, we need to read the data
        // Actually, we should send header + dummy bytes for reading
        let mut cs = self.cs.try_lock().unwrap();
        let mut spi = self.driver.try_lock().unwrap();
        
        // Build frame: [CTRL (0x0B), 0x00, 0x00, 0x00, 0x00] + dummy bytes for reading
        let mut tx_frame = heapless::Vec::<u8, 260>::new();
        if tx_frame.push(0x0B).is_err() { return None; } // CTRL byte for read
        if tx_frame.push(0x00).is_err() { return None; } // Dummy byte 1
        if tx_frame.push(0x00).is_err() { return None; } // Dummy byte 2
        if tx_frame.push(0x00).is_err() { return None; } // Dummy byte 3
        if tx_frame.push(0x00).is_err() { return None; } // Dummy byte 4
        
        // Add dummy bytes for reading event data (up to read_buffer_size)
        let read_len = read_buffer_size.min(250) as usize; // Limit to reasonable size
        for _ in 0..read_len {
            if tx_frame.push(0xFF).is_err() { return None; } // Dummy bytes for reading
        }
        
        // Receive header + event data
        let mut rx_frame = heapless::Vec::<u8, 260>::new();
        if rx_frame.resize(5 + read_len, 0).is_err() { return None; }
        
        cs.set_low();
        let transfer_ok = spi.blocking_transfer(rx_frame.as_mut_slice(), tx_frame.as_slice()).is_ok();
        cs.set_high();
        
        if !transfer_ok {
            warn!("Failed to read event data");
            return None;
        }
        
        // Check SPI_READY from header response
        if rx_frame[0] != 0x02 {
            warn!("SPI not ready during event read: 0x{:02X}", rx_frame[0]);
            return None;
        }
        
        // Extract event data (skip the 5-byte header)
        let event_data = &rx_frame[5..];
        
        if event_data.is_empty() {
            return None;
        }
        
        // Parse event: [Event Code, Parameter Length, Parameters...]
        let event_code = event_data[0];
        if event_data.len() < 2 {
            warn!("Event data too short: {} bytes", event_data.len());
            return None;
        }
        
        let param_len = event_data[1] as usize;
        
        // Validate parameter length
        if param_len > 250 || event_data.len() < 2 + param_len {
            warn!("Invalid event: code=0x{:02X}, param_len={}, data_len={}", 
                  event_code, param_len, event_data.len());
            return None;
        }
        
        // Build event packet
        let mut event = heapless::Vec::<u8, 255>::new();
        if event.extend_from_slice(&event_data[..2 + param_len]).is_err() {
            warn!("Event too large for buffer");
            return None;
        }
        
        info!("Event header: code=0x{:02X}, param_len={}", event_code, param_len);
        Some(event)
    }
    
    /// Wait for and read a Command Complete event
    /// Returns: (Status, Event Data)
    /// 
    /// Note: Event format may vary between HCI standard (0x0E) and ACI vendor-specific events.
    /// This implementation handles both formats. Verify against UM1865 for your specific firmware version.
    async fn wait_command_complete(&self, expected_opcode: u16, timeout_ms: u64) -> Result<(u8, heapless::Vec<u8, 255>), AciError> {
        let start_time = embassy_time::Instant::now();
        let mut event_count = 0;
        
        loop {
            // Check timeout first
            if start_time.elapsed().as_millis() > timeout_ms {
                warn!("Timeout waiting for Command Complete (opcode=0x{:04X}), received {} events", expected_opcode, event_count);
                return Err(AciError::Timeout);
            }
            
            if let Some(event) = self.read_event().await {
                event_count += 1;
                if event.len() >= 2 {
                    let event_code = event[0];
                    let param_len = event[1] as usize;
                    
                    info!("Received event: code=0x{:02X}, param_len={}, total_len={}", event_code, param_len, event.len());
                    
                    // Command Complete event code is 0x0E (HCI standard) or vendor-specific codes
                    // For ACI commands, vendor-specific Command Complete events may use different codes
                    // Check for standard HCI (0x0E) and common vendor-specific patterns
                    if event_code == 0x0E {
                        // Standard HCI Command Complete: [Event Code, Param Len, Num Commands, Opcode Lo, Opcode Hi, Status, ...]
                        if event.len() >= 6 {
                            let num_commands = event[2];
                            if num_commands > 0 {
                                let opcode_lo = event[3];
                                let opcode_hi = event[4];
                                let received_opcode = opcode_lo as u16 | ((opcode_hi as u16) << 8);
                                
                                info!("HCI Command Complete: num_cmds={}, opcode=0x{:04X}, expected=0x{:04X}", 
                                      num_commands, received_opcode, expected_opcode);
                                
                                if received_opcode == expected_opcode {
                                    let status = event[5];
                                    let mut data = heapless::Vec::<u8, 255>::new();
                                    if event.len() > 6 {
                                        data.extend_from_slice(&event[6..]).map_err(|_| AciError::TooLarge)?;
                                    }
                                    info!("Command Complete: status=0x{:02X}, data_len={}", status, data.len());
                                    return Ok((status, data));
                                }
                            }
                        }
                    } else if event.len() >= 5 {
                        // Vendor-specific ACI Command Complete: try multiple formats
                        // Format 1: [Event Code, Param Len, Opcode Lo, Opcode Hi, Status, ...]
                        let opcode_lo = event[2];
                        let opcode_hi = event[3];
                        let received_opcode = opcode_lo as u16 | ((opcode_hi as u16) << 8);
                        
                        info!("Vendor event: code=0x{:02X}, opcode=0x{:04X}, expected=0x{:04X}", 
                              event_code, received_opcode, expected_opcode);
                        
                        if received_opcode == expected_opcode {
                            let status = event[4];
                            let mut data = heapless::Vec::<u8, 255>::new();
                            if event.len() > 5 {
                                data.extend_from_slice(&event[5..]).map_err(|_| AciError::TooLarge)?;
                            }
                            info!("Command Complete (vendor): status=0x{:02X}, data_len={}", status, data.len());
                            return Ok((status, data));
                        }
                        
                        // Format 2: [Event Code, Param Len, Status, Opcode Lo, Opcode Hi, ...]
                        if event.len() >= 6 {
                            let status_alt = event[2];
                            let opcode_lo_alt = event[3];
                            let opcode_hi_alt = event[4];
                            let received_opcode_alt = opcode_lo_alt as u16 | ((opcode_hi_alt as u16) << 8);
                            
                            if received_opcode_alt == expected_opcode {
                                let mut data = heapless::Vec::<u8, 255>::new();
                                if event.len() > 5 {
                                    data.extend_from_slice(&event[5..]).map_err(|_| AciError::TooLarge)?;
                                }
                                info!("Command Complete (vendor alt): status=0x{:02X}, data_len={}", status_alt, data.len());
                                return Ok((status_alt, data));
                            }
                        }
                    }
                } else {
                    warn!("Event too short: {} bytes", event.len());
                }
            }
            
            Timer::after_millis(1).await;
        }
    }

    /// Initialize BLE stack
    pub async fn init_ble(&self) -> Result<(u16, u16), AciError> {
        info!("Initializing BLE stack...");

        // 1. HCI Reset
        info!("Sending HCI Reset...");
        let reset_cmd = hci_reset()?.to_bytes::<16>()?;
        self.send_command(&reset_cmd).await?;

        // 2. Configure public address (offset 0x00-0x05 in config data)
        // Using a simple public address - in production you'd want a proper one
        let public_addr: [u8; 6] = [0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC];
        info!("Setting public address...");
        let addr_cmd = aci_hal_write_config_data(0x00, &public_addr)?;
        let addr_bytes = addr_cmd.to_bytes::<32>()?;
        self.send_command(&addr_bytes).await?;

        // 3. Set TX power
        info!("Setting TX power...");
        let tx_power_cmd = aci_hal_set_tx_power_level(0, 4)?; // Normal power, level 4
        let tx_power_bytes = tx_power_cmd.to_bytes::<16>()?;
        self.send_command(&tx_power_bytes).await?;

        // 4. Initialize GAP as peripheral
        info!("Initializing GAP...");
        let device_name = b"DiveComputer";
        let gap_init_cmd = aci_gap_init(0x01, 0x00, device_name)?; // Role: peripheral, no privacy
        let gap_init_bytes = gap_init_cmd.to_bytes::<64>()?;
        self.send_command(&gap_init_bytes).await?;

        // 5. Initialize GATT
        info!("Initializing GATT...");
        let gatt_init_cmd = aci_gatt_init()?;
        let gatt_init_bytes = gatt_init_cmd.to_bytes::<16>()?;
        self.send_command(&gatt_init_bytes).await?;

        // 6. Add GATT service (using Environmental Sensing Service UUID: 0x181A)
        info!("Adding GATT service...");
        let service_uuid: [u8; 2] = [0x1A, 0x18]; // Little-endian 16-bit UUID
        let add_service_cmd = aci_gatt_add_service(
            0x01, // 16-bit UUID
            &service_uuid,
            0x01, // Primary service
            10,   // Max attribute records
        )?;
        let add_service_bytes = add_service_cmd.to_bytes::<32>()?;
        let service_response = self.send_command_with_response(&add_service_bytes).await?;
        
        // Extract service handle from response: [Service Handle (2 bytes), ...]
        let service_handle = if service_response.len() >= 2 {
            service_response[0] as u16 | ((service_response[1] as u16) << 8)
        } else {
            error!("Invalid service response, using default handle");
            0x0001 // Fallback
        };
        info!("Service handle: 0x{:04X}", service_handle);

        // 7. Add characteristic for sensor data (Temperature and Pressure)
        // Using Temperature UUID: 0x2A6E
        info!("Adding sensor characteristic...");
        let char_uuid: [u8; 2] = [0x6E, 0x2A]; // Temperature UUID (little-endian)
        let add_char_cmd = aci_gatt_add_char(
            service_handle,
            0x01, // 16-bit UUID
            &char_uuid,
            20,   // Max value length (enough for pressure + temperature)
            0x12, // Properties: Read (0x02) + Notify (0x10)
            0x00, // Security: No security
            0x01, // GATT event mask: notifications enabled
            16,   // Encryption key size
            0x01, // Variable length
        )?;
        let add_char_bytes = add_char_cmd.to_bytes::<64>()?;
        let char_response = self.send_command_with_response(&add_char_bytes).await?;
        
        // Extract characteristic handle from response: [Char Handle (2 bytes), ...]
        let char_handle = if char_response.len() >= 2 {
            char_response[0] as u16 | ((char_response[1] as u16) << 8)
        } else {
            error!("Invalid characteristic response, using default handle");
            0x0002 // Fallback
        };
        info!("Characteristic handle: 0x{:04X}", char_handle);

        // 8. Start advertising
        info!("Starting advertising...");
        let adv_cmd = aci_gap_set_undirected_connectable(
            0x00, // ADV_IND (connectable undirected)
            0x0020, // Min interval: 20ms
            0x0040, // Max interval: 40ms
            0x00, // Public address
            0x00, // No filter policy
            device_name,
            Some(&service_uuid), // Advertise our service UUID
            0x0018, // Connection interval min: 30ms
            0x0028, // Connection interval max: 50ms
        )?;
        let adv_bytes = adv_cmd.to_bytes::<128>()?;
        self.send_command(&adv_bytes).await?;

        info!("BLE initialization complete!");
        Ok((service_handle, char_handle))
    }

    /// Update sensor characteristic value
    pub async fn update_sensor_data(&self, service_handle: u16, char_handle: u16, pressure: f32, temperature: f32) -> Result<(), AciError> {
        // Pack sensor data: [pressure (f32, 4 bytes), temperature (f32, 4 bytes)]
        let pressure_bytes = pressure.to_le_bytes();
        let temp_bytes = temperature.to_le_bytes();
        
        let mut value = heapless::Vec::<u8, 16>::new();
        value.extend_from_slice(&pressure_bytes).map_err(|_| AciError::TooLarge)?;
        value.extend_from_slice(&temp_bytes).map_err(|_| AciError::TooLarge)?;

        let update_cmd = aci_gatt_update_char_value(
            service_handle,
            char_handle,
            0, // Offset
            &value,
        )?;
        let update_bytes = update_cmd.to_bytes::<64>()?;
        self.send_command(&update_bytes).await?;
        
        Ok(())
    }
}

// // Produces HCI/ACI command packet bytes: [OpCode (LE, 2 bytes), ParamLen (1 byte), Params...]
// // Use these buffers with your SPI driver: `driver.write_command(&buf).await`.

use heapless::Vec;

/// Max parameter length we build here (adjust if needed)
const MAX_PARAMS: usize = 248;

/// Small error type
#[derive(Debug, defmt::Format)]
pub enum AciError {
    TooLarge,
    SpiError,
    InvalidResponse,
    CommandFailed(u8), // Status code from Command Complete event
    Timeout,
}

/// Simple container for an ACI/HCI command
pub struct AciCommand {
    /// The opcode (u16 - as written in UM1865 e.g. 0xFC8A for Aci_Gap_Init)
    pub opcode: u16,
    /// Parameter buffer (length <= 255)
    pub params: Vec<u8, MAX_PARAMS>,
}

impl AciCommand {
    pub fn new(opcode: u16) -> Self {
        Self {
            opcode,
            params: Vec::new(),
        }
    }

    /// push a single byte param
    pub fn push_u8(&mut self, v: u8) -> Result<(), AciError> {
        self.params.push(v).map_err(|_| AciError::TooLarge)
    }

    /// push a u16 LE
    pub fn push_u16(&mut self, v: u16) -> Result<(), AciError> {
        self.push_u8((v & 0xff) as u8)?;
        self.push_u8((v >> 8) as u8)?;
        Ok(())
    }

    /// push raw bytes
    pub fn push_bytes(&mut self, b: &[u8]) -> Result<(), AciError> {
        for &x in b {
            self.push_u8(x)?;
        }
        Ok(())
    }

    /// final serialized packet bytes
    /// Format: [opcode_lo, opcode_hi, param_len, params...]
    pub fn to_bytes<const N: usize>(&self) -> Result<Vec<u8, N>, AciError> {
        let mut out = Vec::<u8, N>::new();
        out.push((self.opcode & 0xff) as u8).map_err(|_| AciError::TooLarge)?;
        out.push((self.opcode >> 8) as u8).map_err(|_| AciError::TooLarge)?;
        out.push(self.params.len() as u8).map_err(|_| AciError::TooLarge)?;
        out.extend_from_slice(&self.params).map_err(|_| AciError::TooLarge)?;
        Ok(out)
    }
}

/* ============================
   Opcodes used (from UM1865)
   - GAP VS: Aci_Gap_Init = 0xFC8A
   - GATT VS: Aci_Gatt_Init = 0xFD01
   - Aci_Gatt_Add_Service = 0xFD02
   - Aci_Gatt_Add_Characteristic = 0xFD04
   - Aci_Gatt_Update_Char_Value = 0xFD06
   - HCI vendor HAL: Aci_Hal_Write_Config_Data = 0xFC0C
   - Aci_Hal_Set_Tx_Power_Level = 0xFC0F
   - HCI Reset (standard HCI): 0x0C03
   See UM1865 for full table and parameter descriptions.
   ============================ */

pub mod opcodes {
    pub const HCI_RESET: u16 = 0x0C03;
    pub const ACI_HAL_WRITE_CONFIG_DATA: u16 = 0xFC0C;
    pub const ACI_HAL_SET_TX_POWER_LEVEL: u16 = 0xFC0F;

    pub const ACI_GAP_INIT: u16 = 0xFC8A;
    pub const ACI_GAP_SET_DISCOVERABLE: u16 = 0xFC83;
    // ... (add more GAP opcodes as needed)

    pub const ACI_GATT_INIT: u16 = 0xFD01;
    pub const ACI_GATT_ADD_SERVICE: u16 = 0xFD02;
    pub const ACI_GATT_ADD_CHAR: u16 = 0xFD04;
    pub const ACI_GATT_UPDATE_CHAR_VALUE: u16 = 0xFD06;
}

/// helper: HCI Reset
pub fn hci_reset() -> Result<AciCommand, AciError> {
    Ok(AciCommand::new(opcodes::HCI_RESET))
}

/// HAL: write config data (offset,u8 length, value[])
pub fn aci_hal_write_config_data(offset: u8, value: &[u8]) -> Result<AciCommand, AciError> {
    let mut cmd = AciCommand::new(opcodes::ACI_HAL_WRITE_CONFIG_DATA);
    cmd.push_u8(offset)?;
    cmd.push_u8(value.len() as u8)?;
    cmd.push_bytes(value)?;
    Ok(cmd)
}

/// HAL: set tx power (EN_HIGH_POWER: 0|1, PA_LEVEL: 0..7)
pub fn aci_hal_set_tx_power_level(en_high_power: u8, pa_level: u8) -> Result<AciCommand, AciError> {
    let mut cmd = AciCommand::new(opcodes::ACI_HAL_SET_TX_POWER_LEVEL);
    cmd.push_u8(en_high_power)?;
    cmd.push_u8(pa_level)?;
    Ok(cmd)
}

/// GAP: aci_gap_init
///
/// role: 1 byte as in ST (0x01 = PERIPHERAL? check UM1865 for roles if you want other modes)
/// privacy: 0/1
/// device_name: bytes (max length limited by your stack; pass desired name)
pub fn aci_gap_init(role: u8, privacy: u8, device_name: &[u8]) -> Result<AciCommand, AciError> {
    let mut cmd = AciCommand::new(opcodes::ACI_GAP_INIT);
    cmd.push_u8(role)?;
    cmd.push_u8(privacy)?;
    // device name length is a single byte param
    cmd.push_u8(device_name.len() as u8)?;
    cmd.push_bytes(device_name)?;
    Ok(cmd)
}

/// GAP: set discoverable - convenience builder for undirected connectable advertising
///
/// advertising_type: see UM1865 (Adv event type values). Here we provide a simple wrapper.
/// adv_interval_min/adv_interval_max: u16 (in units per spec)
/// address_type: u8
/// adv_filter_policy: u8
/// local_name: slice
pub fn aci_gap_set_undirected_connectable(
    adv_event_type: u8,
    adv_interval_min: u16,
    adv_interval_max: u16,
    address_type: u8,
    adv_filter_policy: u8,
    local_name: &[u8],
    service_uuid_list: Option<&[u8]>,
    conn_int_min: u16,
    conn_int_max: u16,
) -> Result<AciCommand, AciError> {
    let mut cmd = AciCommand::new(opcodes::ACI_GAP_SET_DISCOVERABLE);
    cmd.push_u8(adv_event_type)?;
    cmd.push_u16(adv_interval_min)?;
    cmd.push_u16(adv_interval_max)?;
    cmd.push_u8(address_type)?;
    cmd.push_u8(adv_filter_policy)?;
    cmd.push_u8(local_name.len() as u8)?;
    cmd.push_bytes(local_name)?;
    if let Some(s) = service_uuid_list {
        let len: u8 = s.len().try_into().map_err(|_| AciError::TooLarge)?;
        cmd.push_u8(len)?;
        cmd.push_bytes(s)?;
    } else {
        cmd.push_u8(0)?;
    }
    cmd.push_u16(conn_int_min)?;
    cmd.push_u16(conn_int_max)?;
    Ok(cmd)
}

/// GATT: initialize GATT server
pub fn aci_gatt_init() -> Result<AciCommand, AciError> {
    Ok(AciCommand::new(opcodes::ACI_GATT_INIT))
}

/// GATT: add service
///
/// service_uuid_type: 0x01=16-bit, 0x02=128-bit
/// service_uuid: 2 bytes (if type=1) or 16 bytes (if type=2)
/// service_type: 0x01 primary, 0x02 secondary
/// max_attribute_records: u8
pub fn aci_gatt_add_service(
    uuid_type: u8,
    uuid: &[u8],
    service_type: u8,
    max_attribute_records: u8,
) -> Result<AciCommand, AciError> {
    let mut cmd = AciCommand::new(opcodes::ACI_GATT_ADD_SERVICE);
    cmd.push_u8(uuid_type)?;
    cmd.push_bytes(uuid)?;
    cmd.push_u8(service_type)?;
    cmd.push_u8(max_attribute_records)?;
    Ok(cmd)
}

/// GATT: add characteristic
///
/// serv_handle: 2 bytes (handle returned by add_service command)
/// char_uuid_type: 0x01=16-bit, 0x02=128-bit
/// char_uuid: 2 or 16 bytes accordingly
/// char_value_length: u16 (max length)
/// char_properties: u8 (bitmask per BLE spec)
/// security_permissions: u8
/// gatt_evt_mask: u8
/// encryption_key_size: u8
/// is_variable: u8 (0/1)
pub fn aci_gatt_add_char(
    serv_handle: u16,
    char_uuid_type: u8,
    char_uuid: &[u8],
    char_value_length: u16,
    char_properties: u8,
    security_permissions: u8,
    gatt_evt_mask: u8,
    encryption_key_size: u8,
    is_variable: u8,
) -> Result<AciCommand, AciError> {
    let mut cmd = AciCommand::new(opcodes::ACI_GATT_ADD_CHAR);
    cmd.push_u16(serv_handle)?;
    cmd.push_u8(char_uuid_type)?;
    cmd.push_bytes(char_uuid)?;
    // Note: UM1865 states for some FW versions value length used to be 1 byte; modern stacks use u16.
    cmd.push_u16(char_value_length)?;
    cmd.push_u8(char_properties)?;
    cmd.push_u8(security_permissions)?;
    cmd.push_u8(gatt_evt_mask)?;
    cmd.push_u8(encryption_key_size)?;
    cmd.push_u8(is_variable)?;
    Ok(cmd)
}

/// GATT: update characteristic value (typical usage to notify clients)
///
/// serv_handle, char_handle = handles returned when creating service/char
/// val_offset: offset (0 for normal update)
/// value: payload bytes
pub fn aci_gatt_update_char_value(
    serv_handle: u16,
    char_handle: u16,
    val_offset: u8,
    value: &[u8],
) -> Result<AciCommand, AciError> {
    let mut cmd = AciCommand::new(opcodes::ACI_GATT_UPDATE_CHAR_VALUE);
    cmd.push_u16(serv_handle)?;
    cmd.push_u16(char_handle)?;
    cmd.push_u8(val_offset)?;
    cmd.push_u8(value.len() as u8)?;
    cmd.push_bytes(value)?;
    Ok(cmd)
}

/* ------------------------------
   BLE Task
   ------------------------------ */

#[embassy_executor::task]
pub async fn ble_peripheral_task(
    driver: &'static Mutex<ThreadModeRawMutex, embassy_stm32::spi::Spi<'static, Async, SpiMaster>>,
    rst: &'static Mutex<ThreadModeRawMutex, Output<'static>>,
    cs: &'static Mutex<ThreadModeRawMutex, Output<'static>>,
    state: &'static Mutex<ThreadModeRawMutex, State>,
) {
    let bluenrg = BLUENRGM0A::new(driver, rst, cs);
    
    // Reset the module
    // bluenrg.reset().await;
    // Timer::after_millis(500).await; // Wait for module to stabilize
    
    // Test basic communication before attempting BLE initialization
    // match bluenrg.test_communication().await {
    //     Ok(true) => {
    //         info!("Communication test passed, proceeding with BLE initialization");
    //     }
    //     Ok(false) => {
    //         error!("Communication test failed - module not responding correctly");
    //         error!("Cannot proceed with BLE initialization");
    //         return;
    //     }
    //     Err(e) => {
    //         error!("Communication test error: {:?}", e);
    //         error!("Cannot proceed with BLE initialization");
    //         return;
    //     }
    // }

    // loop {
    //     // bluenrg.reset().await;
    //     bluenrg.test_communication().await;
    //     Timer::after_millis(100).await;
    // }
    
    // Initialize BLE stack
    let (service_handle, char_handle) = match bluenrg.init_ble().await {
        Ok(handles) => handles,
        Err(e) => {
            error!("Failed to initialize BLE: {:?}", e);
            return;
        }
    };
    
    info!("BLE peripheral ready! Service handle: {}, Char handle: {}", service_handle, char_handle);
    
    // Main loop: update sensor data periodically
    loop {
        // Read sensor data from state
        let (pressure, temperature) = {
            let state_guard = state.try_lock().unwrap();
            (state_guard.pressure, state_guard.temperature)
        };
        
        // Update characteristic value
        if let Err(e) = bluenrg.update_sensor_data(service_handle, char_handle, pressure, temperature).await {
            warn!("Failed to update sensor data: {:?}", e);
        } else {
            info!("Updated sensor data: P={} Pa, T={} C", pressure, temperature);
        }
        
        // Process any pending events (non-command-complete events)
        // Command Complete events are handled in send_command()
        if let Some(event) = bluenrg.read_event().await {
            if event.len() > 0 {
                let event_code = event[0];
                // Skip Command Complete events (0x0E) and ACI Command Complete (0xFF) as they're handled in send_command
                if event_code != 0x0E && event_code != 0xFF {
                    info!("Received BLE event: code=0x{:02X}, len={}", event_code, event.len());
                    // Handle other events (connection, disconnection, etc.) here
                }
            }
        }
        
        // Update every 2 seconds
        Timer::after_millis(2000).await;
    }
}

/* ------------------------------
   Minimal event parser skeleton
   (you will get vendor specific event packets from the controller;
    parse them in your IRQ task after reading SPI frames)
   ------------------------------ */

/// Small enum with a couple of events to get you started. Extend from UM1865.
pub enum AciEvent<'a> {
    CommandComplete { opcode: u16, data: &'a [u8] },
    GattAttributeModified { connection_handle: u16, attribute_handle: u16, data: &'a [u8] },
    Unknown(&'a [u8]),
}

/// Very small parser: the event payload format varies — this is a placeholder.
/// Implement full event parsing using UM1865 tables when you need to handle events.
pub fn parse_event(evt: &[u8]) -> AciEvent<'_> {
    // simple fallback
    AciEvent::Unknown(evt)
}
