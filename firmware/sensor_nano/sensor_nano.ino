/*
  BillieBot Sensor Nano -- production-candidate platform-sensor acquisition firmware.

  Target: Arduino Nano V3 / ATmega328P @ 16 MHz, 115200 baud USB serial to the Jetson.

  Wiring (see docs/md/BILLIEBOT_IMU_BATTERY_BENCH_TEST_PLAN.md section 6):
      A4 / SDA -> DFRobot SEN0253 "D"   (I2C data)
      A5 / SCL -> DFRobot SEN0253 "C"   (I2C clock)
      A0       -> battery-divider midpoint
      5V/GND   -> SEN0253 VCC/GND

  This board ACQUIRES ONLY. It contains no mission logic, no SAFE-state logic, no motor
  commands and no motor watchdog -- those belong to the Motor Nano and to the Jetson. It
  likewise contains no divider ratio, no ADC reference voltage and no battery thresholds:
  it emits the raw averaged ADC count and the Jetson bridge owns every conversion. Putting
  a "5.0 V" or a "10.5 V" in this file would hide a calibration constant inside a device
  that cannot be recalibrated without a reflash.

  Scheduling is non-blocking elapsed-micros(). There is no delay() anywhere in loop().

  ---------------------------------------------------------------------------------------
  Coordinate frames and units -- read before changing anything below
  ---------------------------------------------------------------------------------------
  Acceleration is the BNO055 VECTOR_ACCELEROMETER output, which is specific force and
  therefore INCLUDES GRAVITY (a stationary board reads ~9.8 m/s^2). It is deliberately NOT
  VECTOR_LINEARACCEL: the BillieBot robot_localization config sets
  imu0_remove_gravitational_acceleration: true, so sending an already gravity-removed vector
  would remove gravity twice. A stationary magnitude near zero is the signature of that
  mistake (test plan section 20.7).

  No axis remap and no world-frame conversion is applied here. The BNO055 stays at its
  default P1 axis placement and the quaternion is transmitted exactly as the chip fuses it.
  Any frame conversion is an explicit, documented, opt-in transform in the Jetson bridge
  (`orientation_frame_convention`), so the convention is visible in configuration rather
  than buried in a sketch.

  Unit conversions applied here. The IMU path reads the BNO055 data registers directly --
  see bnoRead() for why -- so the scale factors and byte order below are transcribed from the
  installed Adafruit_BNO055.cpp getQuat()/getVector() bodies and datasheet section 3.6.4,
  not assumed:
      QUATERNION (reg 0x20, 8 B)      1/2^14 per LSB, dimensionless -> sent as-is
      GYROSCOPE  (reg 0x14, 6 B)      1 dps = 16 LSB                -> * DEG_TO_RAD -> rad/s
      ACCELEROMETER (reg 0x08, 6 B)   1 m/s^2 = 100 LSB             -> sent as m/s^2
      getVector(VECTOR_MAGNETOMETER)  microtesla (1 uT = 16 LSB)    -> sent as uT
  Those registers are all little-endian two's-complement 16-bit. bnoInt16() does the decode
  and static_assert pins its byte order and sign handling at compile time, so an endianness
  or sign regression fails the build rather than reaching the bench.

  The library's begin() leaves BNO055_UNIT_SEL at its power-on default (the write is
  commented out upstream), which is what makes those divisors correct. If a future library
  version starts writing UNIT_SEL, re-verify the gyro unit before trusting this firmware.

  Magnetometer is transmitted in MICROTESLA, not tesla: tesla would need eight decimals of
  fixed-point text to preserve the chip's 0.0625 uT quantum. The bridge scales to tesla for
  sensor_msgs/MagneticField.
*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>   /* also pulls in utility/imumaths.h */
#include <Adafruit_BMP280.h>

/* ====================================================================================
   Compile-time configuration
   ==================================================================================== */

#define SERIAL_BAUD            115200UL

#define BNO055_I2C_ADDRESS     0x28
#define BMP280_I2C_ADDRESS     0x76   /* SEN0253 default; Adafruit's BMP280_ADDRESS_ALT */

/* NDOF is the 9-DoF absolute-orientation fusion mode. It is the default because the bench
   plan records magnetometer calibration as an informational metric (section 14.8), which
   only exists when the magnetometer is in the fusion loop. Switch to
   OPERATION_MODE_IMUPLUS for gyro+accel-only relative heading that is immune to the motor
   and speaker magnetics of BLK-13 -- the active mode is reported in every status record, so
   changing this can never silently misrepresent a captured run. */
#define BNO055_FUSION_MODE     OPERATION_MODE_NDOF

/* Set to 0 to drop the magnetometer record and reclaim its serial bandwidth. */
#define ENABLE_MAGNETOMETER    1

/* Emission periods in microseconds. IMU is the highest-priority path. */
#define IMU_PERIOD_US          20000UL    /*  50 Hz */
#define MAG_PERIOD_US         100000UL    /*  10 Hz */
#define BATTERY_PERIOD_US     200000UL    /*   5 Hz */
#define BAROMETER_PERIOD_US   500000UL    /*   2 Hz */
#define STATUS_PERIOD_US     1000000UL    /*   1 Hz */

#define BATTERY_ADC_PIN        A0

/* I2C transactions are bounded so a held-low SDA cannot wedge the controller forever.
   reset_with_timeout=true re-initialises the TWI peripheral on timeout, which is what lets
   the bus recover after a transient short or a brown-out on the sensor rail. */
#define I2C_TIMEOUT_US         25000UL
#define I2C_CLOCK_HZ           100000UL

/* A valid BNO055 fusion quaternion is always unit norm. Every BNO055 read on the IMU path is
   now transaction-checked (bnoRead()), so this gate is no longer the only defence against
   zeroed data -- it is defence in depth against the case a transaction check cannot see: a
   chip that reset into CONFIG mode and *successfully* returns real zeros. */
#define QUAT_NORM_MIN          0.90f
#define QUAT_NORM_MAX          1.10f

/* BNO055 fixed-point scale factors, datasheet section 3.6.4. Identical to the divisors in
   the installed Adafruit_BNO055.cpp getVector()/getQuat(); the library's begin() leaves
   UNIT_SEL at its power-on default, which is what makes them correct. */
#define BNO055_QUAT_LSB            16384.0f   /* 2^14 per unit, dimensionless */
#define BNO055_GYRO_LSB_PER_DPS       16.0f
#define BNO055_ACCEL_LSB_PER_MPS2    100.0f

/* Plausibility window for the BMP280. Outside this the sample is dropped and counted
   instead of being transmitted. Matches the test plan's 30-110 kPa acceptance band. */
#define PRESSURE_MIN_PA        30000.0f
#define PRESSURE_MAX_PA        110000.0f
#define TEMPERATURE_MIN_C      (-40.0f)
#define TEMPERATURE_MAX_C      85.0f

/* Bounded recovery: after this many consecutive read failures a peripheral is re-begun, at
   most once per cooldown, at most MAX_REINIT_ATTEMPTS times. begin() blocks for roughly a
   second, which is acceptable only because the peripheral is already dead when we get here;
   the cooldown is what stops that stall from repeating every loop. */
#define FAILURES_BEFORE_REINIT 25
#define REINIT_COOLDOWN_US   5000000UL
#define MAX_REINIT_ATTEMPTS    20

/* Worst-case line is the IMU record: type + uint32 seq + uint32 micros + 4 quaternion +
   3 gyro + 3 accel + 4 calibration fields + CRC suffix. Measured worst case is ~128 bytes;
   160 leaves headroom without meaningfully denting the 2 KB of SRAM. */
#define LINE_BUF_LEN           160

/* Decimals per field, chosen to be lossless against each sensor's own quantum:
   quaternion 1/2^14 = 6.1e-5, gyro 1/16 dps = 1.09e-3 rad/s, accel 0.01 m/s^2,
   magnetometer 0.0625 uT. Spending more digits would only buy serial bandwidth away from
   the 50 Hz IMU path. */
#define QUAT_DECIMALS          5
#define GYRO_DECIMALS          4
#define ACCEL_DECIMALS         3
#define MAG_DECIMALS           3
#define PRESSURE_DECIMALS      1
#define TEMPERATURE_DECIMALS   2
#define ADC_DECIMALS           2

/* ====================================================================================
   State
   ==================================================================================== */

Adafruit_BNO055 g_bno = Adafruit_BNO055(-1, BNO055_I2C_ADDRESS, &Wire);
Adafruit_BMP280 g_bmp(&Wire);

static char g_line[LINE_BUF_LEN];

/* One monotonically increasing sequence counter shared by every record type, so a dropped
   frame of ANY type is detectable as a single global discontinuity on the Jetson. */
static uint32_t g_seq = 0;

static uint32_t g_last_imu_us = 0;
static uint32_t g_last_mag_us = 0;
static uint32_t g_last_battery_us = 0;
static uint32_t g_last_barometer_us = 0;
static uint32_t g_last_status_us = 0;

static uint32_t g_adc_accumulator = 0;
static uint16_t g_adc_samples = 0;

static uint8_t g_bno_ok = 0;
static uint8_t g_bmp_ok = 0;

static uint16_t g_i2c_errors = 0;
static uint16_t g_imu_read_errors = 0;
static uint16_t g_bmp_errors = 0;
static uint16_t g_reinits = 0;
static uint16_t g_imu_records_dropped = 0;

static uint8_t g_bno_consecutive_failures = 0;
static uint8_t g_bmp_consecutive_failures = 0;
static uint32_t g_last_reinit_us = 0;

/* ====================================================================================
   CRC-16/CCITT-FALSE -- poly 0x1021, init 0xFFFF, no reflection, no final XOR.
   Byte-for-byte identical to crc16_ccitt_false() in
   billiebot_sensor_tests/sensor_nano/protocol.py. The shared check value
   crc16("123456789") == 0x29B1 is asserted at boot here and in the Python test suite, so
   the two implementations cannot silently diverge.
   ==================================================================================== */

static uint16_t crc16_update(uint16_t crc, uint8_t byte) {
  crc ^= (uint16_t)byte << 8;
  for (uint8_t bit = 0; bit < 8; bit++) {
    if (crc & 0x8000) {
      crc = (uint16_t)((crc << 1) ^ 0x1021);
    } else {
      crc = (uint16_t)(crc << 1);
    }
  }
  return crc;
}

static uint16_t crc16_ccitt_false(const char *data, uint8_t length) {
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < length; i++) {
    crc = crc16_update(crc, (uint8_t)data[i]);
  }
  return crc;
}

/* ====================================================================================
   Line assembly
   ==================================================================================== */

static uint8_t appendFloat(uint8_t n, double value, uint8_t decimals) {
  if (n + 1 >= LINE_BUF_LEN) {
    return n;
  }
  g_line[n++] = ',';
  /* avr-libc's snprintf has no %f unless the float-printf variant is linked in, so dtostrf
     is the correct tool here rather than an omission. Width 1 = minimum width, left packed. */
  dtostrf(value, 1, decimals, g_line + n);
  while (g_line[n] != '\0' && n < LINE_BUF_LEN - 1) {
    n++;
  }
  return n;
}

static uint8_t appendUInt(uint8_t n, uint32_t value) {
  if (n + 1 >= LINE_BUF_LEN) {
    return n;
  }
  g_line[n++] = ',';
  ultoa(value, g_line + n, 10);
  while (g_line[n] != '\0' && n < LINE_BUF_LEN - 1) {
    n++;
  }
  return n;
}

/* Begin a record: "<TYPE>,<seq>,<t_us>". Consumes one sequence number. */
static uint8_t beginRecord(char type, uint32_t t_us) {
  uint8_t n = 0;
  g_line[n++] = type;
  n = appendUInt(n, g_seq++);
  n = appendUInt(n, t_us);
  return n;
}

/* Close the record with its CRC and emit it as a single write.

   The write is allowed to block. At 115200 baud a 128-byte line occupies 11.1 ms of wire
   time against a 20 ms IMU period, and the aggregate offered load is ~58% of the link, so
   HardwareSerial's 64-byte ring drains faster than the schedule refills it and the blocking
   is bounded and non-cumulative. Dropping records to avoid a bounded, budgeted wait would
   trade guaranteed data loss for nothing. Genuine schedule overruns are detected by the
   catch-up guard in loop() and counted in g_imu_records_dropped. */
static void emitRecord(uint8_t n) {
  uint16_t crc = crc16_ccitt_false(g_line, n);
  if (n + 6 >= LINE_BUF_LEN) {
    return;  /* unreachable with the current record set; refuse to emit a truncated line */
  }
  g_line[n++] = '*';
  static const char hex[] = "0123456789ABCDEF";
  g_line[n++] = hex[(crc >> 12) & 0x0F];
  g_line[n++] = hex[(crc >> 8) & 0x0F];
  g_line[n++] = hex[(crc >> 4) & 0x0F];
  g_line[n++] = hex[crc & 0x0F];
  g_line[n++] = '\n';
  Serial.write((const uint8_t *)g_line, n);
}

/* ====================================================================================
   I2C health
   ==================================================================================== */

/* The AVR Wire library (core 1.8.8) latches a timeout flag we can poll after each
   transaction, which is how an I2C fault is counted rather than inferred. */
static void pollI2cTimeout(void) {
  if (Wire.getWireTimeoutFlag()) {
    if (g_i2c_errors < 0xFFFF) {
      g_i2c_errors++;
    }
    Wire.clearWireTimeoutFlag();
  }
}

/* ====================================================================================
   Checked BNO055 register reads

   Adafruit_BNO055::getQuat() and getVector() memset their buffers to zero, call the private
   readLen(), and then DISCARD the bool it returns (Adafruit_BNO055.cpp 1.6.4, getVector() at
   ~401 and getQuat() at ~466). A NACKed or timed-out I2C transaction is therefore
   indistinguishable from a genuine all-zero reading. read8() -- the one getCalibration()
   uses -- has the same defect and one worse consequence: it reuses a single buffer for the
   register address and the reply, so a failed read of CALIB_STAT decodes 0x35 into a
   fabricated cal_gyr=3/cal_acc=1/cal_mag=1.

   A stationary gyro is legitimately near zero and a failed accelerometer read is exactly
   zero, so no value-domain check can tell them apart. readLen() is private, so the only way
   to see the transaction result is to issue the register read here.

   Transaction semantics mirror Adafruit_I2CDevice::write_then_read(): the address write ends
   with NO stop (repeated start -- write_then_read's `stop` argument defaults to false), the
   read ends with one, and the received length is verified. On a NACK the AVR TWI ISR issues
   its own stop, and Wire.setWireTimeout(..., true) resets the peripheral on timeout, so a
   failed transaction can never leave the bus held despite the suppressed stop.
   ==================================================================================== */

static bool bnoRead(uint8_t reg, uint8_t *buffer, uint8_t length) {
  Wire.beginTransmission(BNO055_I2C_ADDRESS);
  if (Wire.write(reg) != 1) {
    Wire.endTransmission(true);
    pollI2cTimeout();
    return false;
  }
  if (Wire.endTransmission(false) != 0) {
    pollI2cTimeout();
    return false;
  }
  if (Wire.requestFrom((uint8_t)BNO055_I2C_ADDRESS, length, (uint8_t)true) != length) {
    pollI2cTimeout();
    return false;
  }
  pollI2cTimeout();
  for (uint8_t i = 0; i < length; i++) {
    buffer[i] = (uint8_t)Wire.read();
  }
  return true;
}

/* Every BNO055 data register pair is little-endian two's-complement 16-bit. Written as a
   single-return constexpr so the static_asserts below are evaluated by the compiler and cost
   nothing at runtime. */
static constexpr int16_t bnoInt16(uint8_t lsb, uint8_t msb) {
  return (int16_t)((uint16_t)lsb | ((uint16_t)msb << 8));
}

/* Compile-time regression guard for the exact mistakes this decode invites. These run on
   every arduino-cli compile and occupy no flash or SRAM. */
static_assert(bnoInt16(0x00, 0x00) == 0, "bnoInt16: zero");
static_assert(bnoInt16(0xE8, 0x03) == 1000, "bnoInt16: little-endian byte order");
static_assert(bnoInt16(0xFF, 0xFF) == -1, "bnoInt16: two's-complement sign");
static_assert(bnoInt16(0x00, 0x80) == -32768, "bnoInt16: full-scale negative");
static_assert(bnoInt16(0xFF, 0x7F) == 32767, "bnoInt16: full-scale positive");

/* Register addresses are taken from the library's own public enums rather than retyped, so
   there is no hand-copied constant to get wrong. Naming them here is what lets the asserts
   below guard the addresses this file actually reads, not just the library's enum values. */
static constexpr uint8_t BNO055_REG_QUATERNION =
    (uint8_t)Adafruit_BNO055::BNO055_QUATERNION_DATA_W_LSB_ADDR;
static constexpr uint8_t BNO055_REG_GYRO = (uint8_t)Adafruit_BNO055::VECTOR_GYROSCOPE;
static constexpr uint8_t BNO055_REG_ACCEL = (uint8_t)Adafruit_BNO055::VECTOR_ACCELEROMETER;
static constexpr uint8_t BNO055_REG_CALIB_STAT =
    (uint8_t)Adafruit_BNO055::BNO055_CALIB_STAT_ADDR;

/* Addresses from the BNO055 page-0 register map. A library version that renumbers these, or
   an edit that points one of them somewhere else, fails the build. */
static_assert(BNO055_REG_QUATERNION == 0x20, "BNO055 QUA_DATA_W_LSB is 0x20");
static_assert(BNO055_REG_GYRO == 0x14, "BNO055 GYR_DATA_X_LSB is 0x14");
static_assert(BNO055_REG_CALIB_STAT == 0x35, "BNO055 CALIB_STAT is 0x35");

/* The acceleration this firmware sends must be specific force INCLUDING gravity, because the
   BillieBot robot_localization config sets imu0_remove_gravitational_acceleration. Reading
   LIA_DATA_X_LSB (0x28, VECTOR_LINEARACCEL) instead would remove gravity twice, and the
   symptom -- a stationary magnitude near zero -- only shows up on the bench. Retargeting
   BNO055_REG_ACCEL at the linear-acceleration registers fails both of these. */
static_assert(BNO055_REG_ACCEL == 0x08, "accelerometer must read ACC_DATA_X_LSB (0x08)");
static_assert(BNO055_REG_ACCEL != (uint8_t)Adafruit_BNO055::VECTOR_LINEARACCEL,
              "accelerometer source must include gravity, never VECTOR_LINEARACCEL");

/* ====================================================================================
   Sampling
   ==================================================================================== */

/* One failed BNO055 transaction invalidates the whole IMU sample. Called at most once per
   sampleImu() -- the caller returns immediately -- so a single bad sample costs exactly one
   error and one consecutive-failure step, never one per register read. */
static void noteBnoSampleFailure(void) {
  if (g_imu_read_errors < 0xFFFF) {
    g_imu_read_errors++;
  }
  if (g_bno_consecutive_failures < 0xFF) {
    g_bno_consecutive_failures++;
  }
  g_bno_ok = 0;
}

static void sampleImu(uint32_t t_us) {
  /* Shared across the four reads; each value is decoded out before the buffer is reused. */
  uint8_t raw[8];

  if (!bnoRead(BNO055_REG_QUATERNION, raw, 8)) {
    noteBnoSampleFailure();
    return;  /* a failed transaction is a dropped sample, never a zeroed orientation */
  }
  float qw = bnoInt16(raw[0], raw[1]) / BNO055_QUAT_LSB;
  float qx = bnoInt16(raw[2], raw[3]) / BNO055_QUAT_LSB;
  float qy = bnoInt16(raw[4], raw[5]) / BNO055_QUAT_LSB;
  float qz = bnoInt16(raw[6], raw[7]) / BNO055_QUAT_LSB;

  float norm = sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
  if (!(norm >= QUAT_NORM_MIN && norm <= QUAT_NORM_MAX)) {
    /* Also catches NaN, since every comparison against NaN is false. */
    noteBnoSampleFailure();
    return;
  }

  if (!bnoRead(BNO055_REG_GYRO, raw, 6)) {
    noteBnoSampleFailure();
    return;  /* a stationary gyro is legitimately near zero -- zeros here must not ship */
  }
  /* 16 LSB per dps, then deg/s -> rad/s; see the unit table in the file header. */
  float gx = bnoInt16(raw[0], raw[1]) / BNO055_GYRO_LSB_PER_DPS * DEG_TO_RAD;
  float gy = bnoInt16(raw[2], raw[3]) / BNO055_GYRO_LSB_PER_DPS * DEG_TO_RAD;
  float gz = bnoInt16(raw[4], raw[5]) / BNO055_GYRO_LSB_PER_DPS * DEG_TO_RAD;

  if (!bnoRead(BNO055_REG_ACCEL, raw, 6)) {
    noteBnoSampleFailure();
    return;
  }
  /* 100 LSB per m/s^2. Specific force, gravity included. */
  float ax = bnoInt16(raw[0], raw[1]) / BNO055_ACCEL_LSB_PER_MPS2;
  float ay = bnoInt16(raw[2], raw[3]) / BNO055_ACCEL_LSB_PER_MPS2;
  float az = bnoInt16(raw[4], raw[5]) / BNO055_ACCEL_LSB_PER_MPS2;

  if (!bnoRead(BNO055_REG_CALIB_STAT, raw, 1)) {
    noteBnoSampleFailure();
    return;  /* rather than the fabricated levels read8() would have decoded from 0x35 */
  }
  uint8_t cal_sys = (raw[0] >> 6) & 0x03;
  uint8_t cal_gyr = (raw[0] >> 4) & 0x03;
  uint8_t cal_acc = (raw[0] >> 2) & 0x03;
  uint8_t cal_mag = raw[0] & 0x03;

  g_bno_consecutive_failures = 0;
  g_bno_ok = 1;

  uint8_t n = beginRecord('I', t_us);
  n = appendFloat(n, qw, QUAT_DECIMALS);
  n = appendFloat(n, qx, QUAT_DECIMALS);
  n = appendFloat(n, qy, QUAT_DECIMALS);
  n = appendFloat(n, qz, QUAT_DECIMALS);
  n = appendFloat(n, gx, GYRO_DECIMALS);
  n = appendFloat(n, gy, GYRO_DECIMALS);
  n = appendFloat(n, gz, GYRO_DECIMALS);
  n = appendFloat(n, ax, ACCEL_DECIMALS);
  n = appendFloat(n, ay, ACCEL_DECIMALS);
  n = appendFloat(n, az, ACCEL_DECIMALS);
  n = appendUInt(n, cal_sys);
  n = appendUInt(n, cal_gyr);
  n = appendUInt(n, cal_acc);
  n = appendUInt(n, cal_mag);
  emitRecord(n);
}

#if ENABLE_MAGNETOMETER
static void sampleMagnetometer(uint32_t t_us) {
  if (!g_bno_ok) {
    return;
  }
  imu::Vector<3> mag = g_bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  pollI2cTimeout();

  uint8_t n = beginRecord('M', t_us);
  n = appendFloat(n, mag.x(), MAG_DECIMALS);
  n = appendFloat(n, mag.y(), MAG_DECIMALS);
  n = appendFloat(n, mag.z(), MAG_DECIMALS);
  emitRecord(n);
}
#endif

/* One conversion per loop iteration, accumulated. Averaging this way costs no scheduling
   jitter at all, unlike a burst of analogRead()s inside the battery tick: each conversion is
   ~112 us and the loop runs far faster than the 5 Hz battery period, so a battery record
   averages hundreds of samples without ever delaying the IMU. */
static void accumulateBatteryAdc(void) {
  g_adc_accumulator += (uint32_t)analogRead(BATTERY_ADC_PIN);
  if (g_adc_samples < 0xFFFF) {
    g_adc_samples++;
  }
}

static void emitBattery(uint32_t t_us) {
  if (g_adc_samples == 0) {
    return;
  }
  double mean = (double)g_adc_accumulator / (double)g_adc_samples;
  g_adc_accumulator = 0;
  g_adc_samples = 0;

  uint8_t n = beginRecord('B', t_us);
  n = appendFloat(n, mean, ADC_DECIMALS);
  emitRecord(n);
}

static void sampleBarometer(uint32_t t_us) {
  if (!g_bmp_ok) {
    return;
  }
  float pressure = g_bmp.readPressure();
  pollI2cTimeout();
  float temperature = g_bmp.readTemperature();
  pollI2cTimeout();

  bool plausible = (pressure >= PRESSURE_MIN_PA && pressure <= PRESSURE_MAX_PA &&
                    temperature >= TEMPERATURE_MIN_C && temperature <= TEMPERATURE_MAX_C);
  if (!plausible) {
    /* readPressure() returns NAN before begin() and 0 on a calibration divide-by-zero;
       both land here rather than on the wire. */
    if (g_bmp_errors < 0xFFFF) {
      g_bmp_errors++;
    }
    if (g_bmp_consecutive_failures < 0xFF) {
      g_bmp_consecutive_failures++;
    }
    return;
  }

  g_bmp_consecutive_failures = 0;

  uint8_t n = beginRecord('P', t_us);
  n = appendFloat(n, pressure, PRESSURE_DECIMALS);
  n = appendFloat(n, temperature, TEMPERATURE_DECIMALS);
  emitRecord(n);
}

static void emitStatus(uint32_t t_us) {
  /* Read the mode back from the chip rather than echoing the #define, so a run captured
     with a modified build still records what actually executed. */
  uint8_t mode = (uint8_t)g_bno.getMode();
  pollI2cTimeout();

  uint8_t n = beginRecord('S', t_us);
  n = appendUInt(n, g_bno_ok);
  n = appendUInt(n, g_bmp_ok);
  n = appendUInt(n, mode);
  n = appendUInt(n, g_i2c_errors);
  n = appendUInt(n, g_imu_read_errors);
  n = appendUInt(n, g_bmp_errors);
  n = appendUInt(n, g_reinits);
  n = appendUInt(n, g_imu_records_dropped);
  emitRecord(n);
}

/* ====================================================================================
   Bounded peripheral recovery
   ==================================================================================== */

static void maybeReinitialize(uint32_t now_us) {
  if (g_reinits >= MAX_REINIT_ATTEMPTS) {
    return;
  }
  if ((uint32_t)(now_us - g_last_reinit_us) < REINIT_COOLDOWN_US) {
    return;
  }

  bool bno_dead = (g_bno_consecutive_failures >= FAILURES_BEFORE_REINIT);
  bool bmp_dead = (g_bmp_consecutive_failures >= FAILURES_BEFORE_REINIT);
  if (!bno_dead && !bmp_dead) {
    return;
  }

  g_last_reinit_us = now_us;
  g_reinits++;

  if (bno_dead) {
    g_bno_ok = g_bno.begin(BNO055_FUSION_MODE) ? 1 : 0;
    g_bno_consecutive_failures = 0;
  }
  if (bmp_dead) {
    /* A dead BMP280 must never take the BNO055 down with it -- these are independent
       recovery paths on a shared bus for exactly that reason. */
    g_bmp_ok = g_bmp.begin(BMP280_I2C_ADDRESS) ? 1 : 0;
    g_bmp_consecutive_failures = 0;
  }
  pollI2cTimeout();

  /* Re-arm the schedule: begin() blocks for up to about a second, and without this the
     catch-up guard would count the stall as dropped IMU records. */
  uint32_t now = micros();
  g_last_imu_us = now;
  g_last_mag_us = now;
  g_last_barometer_us = now;
}

/* ====================================================================================
   Setup / loop
   ==================================================================================== */

void setup() {
  Serial.begin(SERIAL_BAUD);

  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);
  Wire.setWireTimeout(I2C_TIMEOUT_US, true);

  g_bno_ok = g_bno.begin(BNO055_FUSION_MODE) ? 1 : 0;
  g_bmp_ok = g_bmp.begin(BMP280_I2C_ADDRESS) ? 1 : 0;

  if (g_bmp_ok) {
    /* Explicit oversampling/filter selection rather than relying on the reset defaults, so
       the barometer's own conversion time is known and bounded against its 2 Hz slot. */
    g_bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                      Adafruit_BMP280::SAMPLING_X2,
                      Adafruit_BMP280::SAMPLING_X16,
                      Adafruit_BMP280::FILTER_X16,
                      Adafruit_BMP280::STANDBY_MS_63);
  }
  pollI2cTimeout();

  /* Boot self-check of the CRC implementation against the canonical CCITT-FALSE check
     value. A mismatch means this firmware and the Jetson parser disagree about framing, so
     it is reported as an I2C-independent fault rather than left to corrupt every record. */
  if (crc16_ccitt_false("123456789", 9) != 0x29B1) {
    Serial.println(F("E,crc16_self_check_failed"));
  }

  uint32_t now = micros();
  g_last_imu_us = now;
  g_last_mag_us = now;
  g_last_battery_us = now;
  g_last_barometer_us = now;
  g_last_status_us = now;

  emitStatus(now);
}

void loop() {
  uint32_t now = micros();

  /* Every elapsed test below is unsigned subtraction, which is correct across the ~71.6
     minute micros() rollover without any special-casing. */

  accumulateBatteryAdc();

  if ((uint32_t)(now - g_last_imu_us) >= IMU_PERIOD_US) {
    g_last_imu_us += IMU_PERIOD_US;
    /* If we are more than a full period late the schedule cannot be caught up without
       bunching records, so resynchronise and count the loss instead of emitting a burst. */
    if ((uint32_t)(now - g_last_imu_us) >= IMU_PERIOD_US) {
      uint32_t missed = (uint32_t)(now - g_last_imu_us) / IMU_PERIOD_US;
      if (g_imu_records_dropped + missed < 0xFFFF) {
        g_imu_records_dropped += (uint16_t)missed;
      }
      g_last_imu_us = now;
    }
    sampleImu(now);
  }

#if ENABLE_MAGNETOMETER
  if ((uint32_t)(now - g_last_mag_us) >= MAG_PERIOD_US) {
    g_last_mag_us = now;
    sampleMagnetometer(now);
  }
#endif

  if ((uint32_t)(now - g_last_battery_us) >= BATTERY_PERIOD_US) {
    g_last_battery_us = now;
    emitBattery(now);
  }

  if ((uint32_t)(now - g_last_barometer_us) >= BAROMETER_PERIOD_US) {
    g_last_barometer_us = now;
    sampleBarometer(now);
  }

  if ((uint32_t)(now - g_last_status_us) >= STATUS_PERIOD_US) {
    g_last_status_us = now;
    emitStatus(now);
  }

  maybeReinitialize(now);
}
