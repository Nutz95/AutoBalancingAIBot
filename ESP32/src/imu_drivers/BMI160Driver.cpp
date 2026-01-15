// BMI160Driver.cpp
#include "../../include/imu_drivers/BMI160Driver.h"
#include "../../include/imu_calibration.h"
#include "../../include/logging.h"
#include <SPI.h>

namespace abbot {

BMI160Driver::BMI160Driver(const BMI160Config &cfg) : cfg_(cfg) {
}

bool BMI160Driver::begin() {
  LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "BMI160: starting initialization (SPI mode)...");

  if (cfg_.use_spi) {
    LOG_PRINTF(abbot::log::CHANNEL_DEFAULT, "BMI160: Initializing SPI (SCK=%d, MOSI=%d, MISO=%d, CS=%d)\n",
               cfg_.spi_sck, cfg_.spi_mosi, cfg_.spi_miso, cfg_.spi_cs);
    
    // Ensure CS is HIGH before starting SPI
    pinMode(cfg_.spi_cs, OUTPUT);
    digitalWrite(cfg_.spi_cs, HIGH);
    
    // Also set SCK/MOSI to known state to avoid SPI mode glitch
    pinMode(cfg_.spi_sck, OUTPUT);
    pinMode(cfg_.spi_mosi, OUTPUT);
    digitalWrite(cfg_.spi_sck, LOW);
    digitalWrite(cfg_.spi_mosi, LOW);
    delay(50);

    // Initialisation du SPI avec les bonnes pins pour l'ESP32-S3
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "BMI160: Initializing SPI bus...");
    SPI.begin(cfg_.spi_sck, cfg_.spi_miso, cfg_.spi_mosi, cfg_.spi_cs);
    SPI.setClockDivider(SPI_CLOCK_DIV32); // ~2.5MHz pour une stabilité maximale sur câbles longs
    
    // Auto-détection du dummy byte (octet factice)
    // On fait plusieurs essais pour stabiliser le bus SPI
    digitalWrite(cfg_.spi_cs, LOW);
    SPI.transfer(0x00 | 0x80); // Read Chip ID
    uint8_t first = SPI.transfer(0x00); 
    uint8_t second = SPI.transfer(0x00);
    digitalWrite(cfg_.spi_cs, HIGH);
    delay(10);

    LOG_PRINTF(abbot::log::CHANNEL_DEFAULT, "BMI160: SPI Init probe - first=0x%02X, second=0x%02X\n", first, second);

    if (second == 0xD1) {
        has_dummy_ = true;
    } else if (first == 0xD1) {
        has_dummy_ = false;
    } else {
        // Retry once after a dummy read
        digitalWrite(cfg_.spi_cs, LOW);
        SPI.transfer(0x00 | 0x80);
        first = SPI.transfer(0x00);
        second = SPI.transfer(0x00);
        digitalWrite(cfg_.spi_cs, HIGH);
        has_dummy_ = (second == 0xD1);
    }
    
    uint8_t chip_id = has_dummy_ ? second : first;
    
    LOG_PRINTF(abbot::log::CHANNEL_DEFAULT, "BMI160: Mode SPI %s (ID=0x%02X)\n", 
               has_dummy_ ? "avec Dummy Byte" : "Direct", chip_id);
    
    if (chip_id != 0xD1) {
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "BMI160: ERROR - Sensor not found on SPI bus.");
      return false;
    }

    // On effectue un soft-reset MANUEL
    auto manual_write = [&](uint8_t reg, uint8_t val) {
        digitalWrite(cfg_.spi_cs, LOW);
        SPI.transfer(reg & 0x7F); // Bit Write = 0
        SPI.transfer(val);
        digitalWrite(cfg_.spi_cs, HIGH);
        delay(2);
    };

    auto manual_read = [&](uint8_t reg) -> uint8_t {
        digitalWrite(cfg_.spi_cs, LOW);
        SPI.transfer(reg | 0x80); // Bit Read = 1
        if (has_dummy_) {
            SPI.transfer(0x00); // Dummy byte
        }
        uint8_t val = SPI.transfer(0x00);
        digitalWrite(cfg_.spi_cs, HIGH);
        return val;
    };
    
    manual_write(0x7E, 0xB6); // Soft Reset CMD
    delay(100);               // Attente de sécurité (datasheet préconise 80ms)
    
    // On réveille le capteur MANUELLEMENT
    // Note: Datasheet préconise d'attendre entre les commandes de mode
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "BMI160: Awakening sensors (Accel+Gyro)...");
    manual_write(0x7E, 0x11); // ACCEL normal mode
    delay(50);                // Transition accel
    manual_write(0x7E, 0x15); // GYRO normal mode
    delay(100);               // Transition gyro (80ms min)
    
    // Attente du statut stable (Accel Normal + Gyro Normal = 0x14)
    uint8_t pmu_status = 0;
    for (int i = 0; i < 20; i++) {
        pmu_status = manual_read(0x03);
        if (pmu_status == 0x14) {
            break;
        }
        // Si ça ne marche pas, on renvoie les commandes juste au cas où
        if (i == 10) {
            manual_write(0x7E, 0x11);
            delay(10);
            manual_write(0x7E, 0x15);
        }
        delay(20);
    }
    LOG_PRINTF(abbot::log::CHANNEL_DEFAULT, "BMI160: PMU_STATUS = 0x%02X (Expected 0x14)\n", pmu_status);

    // Configuration des registres (ODR et Plages)
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "BMI160: Configuring 800Hz ODR...");
    // 0x40 (ACC_CONF): 0x2B = normal filter, ODR 800Hz
    // 0x42 (GYR_CONF): 0x2B = normal filter, ODR 800Hz
    manual_write(0x40, 0x2B); 
    manual_write(0x42, 0x2B);
    manual_write(0x41, 0x03); // ±2g 
    manual_write(0x43, 0x00); // ±2000 deg/s

    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "BMI160: SPI initialization successful.");
    last_read_us_ = micros();
    return true;
  } else {
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "BMI160: I2C mode requested...");
    Wire.setPins(cfg_.sda_pin, cfg_.scl_pin);
    if (!BMI160.begin(BMI160GenClass::I2C_MODE, Wire, cfg_.i2c_addr, -1)) {
        return false;
    }
  }

  // 1. Configure sensor ranges
  BMI160.setAccelerometerRange(2); // ±2g 
  BMI160.setGyroRange(2000);       // ±2000°/s
  
  // 2. Set high frequency ODR (Output Data Rate)
  // BMI160Gen supports setGyroRate and setAccelerometerRate
  // 400Hz matches BMI160_GYRO_RATE_400HZ (0x0A)
  BMI160.setGyroRate(400); 
  BMI160.setAccelerometerRate(400);
  
  LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "BMI160: ranges set to 2g, 2000 deg/s, ODR=400Hz");
  
  last_read_us_ = micros();
  return true;
}

bool BMI160Driver::read(IMUSample &out) {
  unsigned long now_us = micros();
  unsigned long interval_us = cfg_.sampling_interval_us();
  
  if (interval_us > 0 && (now_us - last_read_us_) < interval_us) {
    return false;
  }

  if (!readRaw(out)) {
    return false;
  }

  out.ts_ms = millis();
  out.ts_us = now_us;
  last_read_us_ = now_us;

  // Apply software calibration
  abbot::imu_cal::applyCalibrationToSample(out);
  
  return true;
}

bool BMI160Driver::readRaw(IMUSample &out) {
  // BMI160 SPI Read requires a dummy byte after the register address.
  // We read 12 bytes in burst: Gyro (6 bytes) + Accel (6 bytes), starting from 0x0C.
  // Register 0x0C is DATA_GYRO_X_LSB
  uint8_t raw_data[12];
  
  digitalWrite(cfg_.spi_cs, LOW);
  SPI.transfer(0x0C | 0x80); // Register 0x0C (GYR_X_L) | Read Bit
  
  if (has_dummy_) {
      SPI.transfer(0x00); // Dummy byte
  }

  uint32_t zero_check = 0;
  for (int i = 0; i < 12; i++) {
    raw_data[i] = SPI.transfer(0x00);
    zero_check |= raw_data[i];
  }
  digitalWrite(cfg_.spi_cs, HIGH);

  // If all 12 bytes are zero, something is wrong with the SPI bus or sensor.
  // Return false to trigger the auto-recovery in SystemTasks.
  if (zero_check == 0) {
      return false;
  }

  // Format: LSB, MSB
  // Data alignment verification:
  // 0,1: GX | 2,3: GY | 4,5: GZ | 6,7: AX | 8,9: AY | 10,11: AZ
  int16_t gx_raw = (int16_t)((raw_data[1]  << 8) | raw_data[0]);
  int16_t gy_raw = (int16_t)((raw_data[3]  << 8) | raw_data[2]);
  int16_t gz_raw = (int16_t)((raw_data[5]  << 8) | raw_data[4]);
  int16_t ax_raw = (int16_t)((raw_data[7]  << 8) | raw_data[6]);
  int16_t ay_raw = (int16_t)((raw_data[9]  << 8) | raw_data[8]);
  int16_t az_raw = (int16_t)((raw_data[11] << 8) | raw_data[10]);

  // Convert raw values to SI units
  // ±2g -> 16384 LSB/g. 
  // ±2000 deg/s -> 16.384 LSB/(deg/s).
  const float ACCEL_SCALE = 9.80665f / 16384.0f; 
  const float GYRO_SCALE = (1.0f / 16.384f) * (3.14159265f / 180.0f);

  out.ax = (float)ax_raw * ACCEL_SCALE;
  out.ay = (float)ay_raw * ACCEL_SCALE;
  out.az = (float)az_raw * ACCEL_SCALE;
  out.gx = (float)gx_raw * GYRO_SCALE;
  out.gy = (float)gy_raw * GYRO_SCALE;
  out.gz = (float)gz_raw * GYRO_SCALE;

  // Sign correction from config
  applySigns(out);

  out.temperatureCelsius = 23.0f; // Placeholder
  out.ts_ms = millis();
  out.ts_us = micros();

  return true;
}

void BMI160Driver::applySigns(IMUSample &out) {
  out.ax *= cfg_.accel_sign_x;
  out.ay *= cfg_.accel_sign_y;
  out.az *= cfg_.accel_sign_z;
  out.gx *= cfg_.gyro_sign_x;
  out.gy *= cfg_.gyro_sign_y;
  out.gz *= cfg_.gyro_sign_z;
}

uint16_t BMI160Driver::getSamplingHz() const {
  return cfg_.sampling_hz;
}

const char* BMI160Driver::getDriverName() const {
  return "BMI160";
}

} // namespace abbot
