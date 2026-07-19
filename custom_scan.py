import numpy as np
import time
import sys
from rtlsdr import RtlSdr
from scipy.signal import welch

if len(sys.argv) < 2:
    print("Must define center frequency")
    print("Usage: python3 custom_scan.py <center_freq_mhz>")
    print("Example: python3 custom_scan.py 852.25")
    sys.exit(1)

user_freq_input = sys.argv[1]

try:
    user_center_freq_mhz = float(user_freq_input)
    LOW_OFFSET_MHZ = -1.0
    HIGH_OFFSET_MHZ = 1.0

except ValueError as e: 
    print(f"Invalid Frequency: {e}")
    print("Usage: python3 custom_scan.py <center_freq_mhz>")
    sys.exit(1)
except Exception as e:
    print(f"Error occurred: {e}")
    sys.exit(1)



class KalmanFilter:
    def __init__(self, process_variance=1e-2, measurement_variance=1e-1):
        self.dt = 0.25
        self.x = np.array([[0.0], [0.0]])
        self.P = np.eye(2) * 1.0
        self.A = np.array([[1.0, self.dt],
                            [0.0, 1.0]])
        self.H = np.array([[1.0, 0.0]])

        self.Q = np.eye(2) * process_variance
        self.R = np.array([[measurement_variance]])

    def update(self, measured_power):
        self.x = np.dot(self.A, self.x)
        self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q
    
        y = float(measured_power - np.dot(self.H, self.x)[0, 0])  
        S = np.dot(np.dot(self.H, self.P), self.H.T) + self.R
    
        K = np.dot(self.P, self.H.T) / S 
        self.x = self.x + K * y
    
        self.P = (np.eye(2) - K @ self.H) @ self.P
    
        return float(self.x[0, 0]), float(self.x[1, 0])  



def scan_rf(center_freq_mhz, sample_rate=2.4e6):
    tracker = KalmanFilter()
    sdr = None
    try:
       sdr = RtlSdr()
       sdr.sample_rate = sample_rate
       sdr.center_freq = center_freq_mhz * 1e6
       sdr.gain = 40 
       sdr.freq_correlation = 1
       
       print(f"SDR Hardware Tuned Wideband to: {center_freq_mhz} MHz")
       print("Calibrating background noise...")
       
       noise_samples = []
       for _ in range(15):
           samples = sdr.read_samples(128 * 1024)
           f, psd = welch(samples, fs=sdr.sample_rate, nperseg=4096, return_onesided=False)
           f_shifted = np.fft.fftshift(f)
           psd_shifted = np.fft.fftshift(psd)
           freqs = sdr.center_freq + f_shifted
           low_freq = (center_freq_mhz - 1.0) * 1e6
           high_freq = (center_freq_mhz + 1.0) * 1e6

           rf_mask = (freqs >= low_freq) & (freqs <= high_freq)
           if np.any(rf_mask):
               noise_samples.append(10 * np.log10(np.max(psd_shifted[rf_mask]) + 1e-12))
           time.sleep(0.1)
           
       baseline = np.median(noise_samples)
       rolling_history = list(noise_samples)
       print(f"Baseline calibrated at: {baseline:.2f} dB")
       print("Monitoring RF spectrum......")
       
       while True:
           t_start = time.time()
           samples = sdr.read_samples(256 * 1024)
           
           f, psd = welch(samples, fs=sdr.sample_rate, nperseg=4096, return_onesided=False)
           f_shifted = np.fft.fftshift(f)
           psd_shifted = np.fft.fftshift(psd)
           freqs = sdr.center_freq + f_shifted
           low_freq = (center_freq_mhz + LOW_OFFSET_MHZ) * 1e6
           high_freq = (center_freq_mhz + HIGH_OFFSET_MHZ) * 1e6

           rf_mask = (freqs >= low_freq) & (freqs <= high_freq)
           
           if not np.any(rf_mask):
              continue
              
           current_raw_power = 10 * np.log10(np.max(psd_shifted[rf_mask]) + 1e-12)
           
           rolling_history.append(current_raw_power)
           if len(rolling_history) > 40:
              rolling_history.pop(0)
              
           current_ambient = np.median(rolling_history)
           signal_variance = np.std(rolling_history)
           relative_delta_db = current_raw_power - current_ambient
           

           smoothed_delta, tracking_slope = tracker.update(relative_delta_db)
           

           if smoothed_delta > 12.0 and tracking_slope > 0.4: 
              if signal_variance > 2.5:
                 print(f"[ALERT] RF Detected! Delta: +{smoothed_delta:.1f}dB, Slope: {tracking_slope:.2f}")
              else:
                 print(f"[INFO] Approaching Static RF Transmitter. Delta: +{smoothed_delta:.1f}dB, Slope: {tracking_slope:.2f}")
                 
           elapsed = time.time() - t_start
           sleep_time = max(0.01, 0.25 - elapsed)
           time.sleep(sleep_time)
           
    except Exception as e:
       print(f"Error occurred: {e}")
    finally:
       if sdr:
         print("Closing SDR...")
         sdr.close()


if __name__ == '__main__':
    scan_rf(center_freq_mhz=user_center_freq_mhz)

