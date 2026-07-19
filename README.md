# Custom-RF-Kalman-Scanner-4-SDR

A real-time Software Defined Radio (SDR) application for monitoring RF activity using adaptive signal processing techniques.

The project combines power spectral density estimation, adaptive background calibration, rolling statistical analysis, and Kalman filtering to identify significant RF signal events within a configurable frequency window.

---

## Features

- Real-time RF spectrum monitoring
- Adaptive background noise calibration
- Welch Power Spectral Density (PSD) estimation
- Rolling ambient noise estimation
- Kalman-filtered signal tracking
- Detection of transient RF signal events
- Configurable center frequency

---

## How It Works

1. Tune an RTL-SDR to a user-specified center frequency.
2. Capture IQ samples continuously.
3. Estimate the spectrum using Welch PSD.
4. Compute a rolling ambient RF baseline.
5. Measure signal strength relative to the baseline.
6. Smooth measurements using a Kalman filter.
7. Generate events when sustained increases in received signal power exceed configurable thresholds.

The detector evaluates two primary metrics:

- **Delta** — Relative increase in received signal power above the adaptive baseline.
- **Slope** — Rate of change of the filtered signal over time.

Together these metrics help identify significant RF activity while reducing noise-induced false positives.

---

## Example

```bash
python3 custom_scan.py 915.0
```

Example output:

```
SDR Hardware Tuned Wideband to: 915.0 MHz
Baseline calibrated at: -78.2 dB
Scanning RF spectrum...

[ALERT] RF Detected!
Delta: +13.8 dB
Slope: 0.57
```

---

## Requirements

- Python 3
- NumPy
- SciPy
- pyrtlsdr
- RTL-SDR compatible receiver

Install:

```bash
pip install numpy scipy pyrtlsdr
```

---

## Project Structure

```
custom_scan.py
```

Contains:

- SDR acquisition
- Welch spectral analysis
- Adaptive baseline estimation
- Kalman filter implementation
- Real-time event detection

---

## Technical Concepts

- Software Defined Radio (SDR)
- Digital Signal Processing (DSP)
- Welch PSD Estimation
- Kalman Filtering
- Adaptive Thresholding
- Rolling Statistical Analysis
- Real-Time Signal Processing

---

## Limitations

This project detects changes in received RF signal strength.

Because it uses a single passive receiver, it **does not directly estimate physical distance** to a transmitter. Received signal strength is influenced by transmitter power, antenna characteristics, propagation conditions, and environmental effects. The reported detection metrics should therefore be interpreted as indicators of relative RF activity rather than absolute range.

---

## Future Improvements

- Automatic threshold tuning
- Waterfall visualization
- Frequency hopping support
- Multi-band scanning
- GUI dashboard
- Detection history and logging
- Multi-receiver support
