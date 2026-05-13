## Output Format

All data is streamed over Serial at **60 Hz** in CSV format:

| Column | Description | Units |
|---|---|---|
| `timestamp_ms` | Time since boot | ms |
| `rpm` | Motor speed | RPM |
| `ax` | Acceleration X | m/s² |
| `ay` | Acceleration Y | m/s² |
| `az` | Acceleration Z | m/s² |
| `gx` | Angular velocity X | rad/s |
| `gy` | Angular velocity Y | rad/s |
| `gz` | Angular velocity Z | rad/s |

## IMU

<p align="center">
  <img src="https://cdn-learn.adafruit.com/assets/assets/000/094/955/medium640/sensors__ism330_pinouts4502-ISM330_top.png?1600792652" width="500">
</p>

Prints acceleration (xyz in m/s²) and angular velocity (xyz in rad/s) at 104 Hz internally, reported at 60 Hz over serial.

### Data Verification

**Accelerations**
- Place the IMU along each axis and verify the recorded acceleration approximates gravity (9.55–9.81 m/s²)

**Gyro Readings**
- Rotate the IMU along each axis and verify that sign changes when shifting direction
- Magnitude should be within reasonable values across all axes

### Wiring

- Connect Arduino Uno's SDA and SCL pins (A4 and A5) to the IMU's SDA and SCL pins
- Connect Uno's 3.3V pin to the IMU's VIN pin
- Connect IMU ground to shared ground with Arduino and RPM sensor

## RPM Sensor

<p align="center">
  <img src="https://img.alicdn.com/imgextra/i3/2208065206382/O1CN01puCDTi1x10ITxiEwT_!!2-item_pic.png_q50.jpg_.webp" width="500">
</p>

Code was tested using the **JMLQD-31NC** photoelectric sensor. If no pulse is detected within **2 seconds**, RPM is set to 0 (timeout).

### Parameters

| Parameter | Value | Description |
|---|---|---|
| Interrupt edge | `FALLING` | Triggers on HIGH → LOW transition |
| Debounce | `100 µs` | Minimum time between valid pulses |
| Timeout | `2,000,000 µs` | Time without pulse before RPM = 0 |
| Gain | `2.3` | Calibration multiplier — adjust per real measurement |
| Max RPM | `8500` | Upper clamp value |

### Wiring

- **Brown wire:** +V (external power supply, minimum 10V)
- **Blue wire:** 0V / Ground (shared with Arduino and IMU)
- **Black wire:** Signal output → Arduino digital pin 2