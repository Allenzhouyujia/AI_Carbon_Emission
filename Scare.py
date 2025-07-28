"""
HX711 Weight Sensor Driver
Professional weight measurement system with fast stabilization algorithm
"""

from machine import Pin
from hx711_gpio import HX711
import time

# Hardware Configuration
PIN_DATA = 8  # HX711 data pin (DOUT)
PIN_CLOCK = 9  # HX711 clock pin (SCK)

# Sensor Configuration
CALIBRATION_FACTOR = 419.0  # Calibration factor (unit: LSB/g)
STABILITY_THRESHOLD = 5.0  # Weight stability threshold (grams)
RAPID_SAMPLE_COUNT = 5  # Number of rapid samples for stabilization
RAPID_SAMPLE_INTERVAL = 0.05  # Interval between rapid samples (seconds)
WARMUP_SAMPLES = 10  # Number of warmup samples during initialization

class WeightSensor:
    """
    Professional HX711 weight sensor interface with fast stabilization
    """
    
    def __init__(self, data_pin=PIN_DATA, clock_pin=PIN_CLOCK):
        """
        Initialize the weight sensor
        
        Args:
            data_pin (int): GPIO pin number for HX711 data line
            clock_pin (int): GPIO pin number for HX711 clock line
        """
        print("Initializing HX711 Weight Sensor...")
        
        try:
            # Configure GPIO pins
            self._pin_data = Pin(data_pin, Pin.IN, pull=Pin.PULL_DOWN)
            self._pin_clock = Pin(clock_pin, Pin.OUT)
            
            # Initialize HX711 driver
            self._hx711 = HX711(self._pin_clock, self._pin_data)
            
            # Sensor state
            self._tare_offset = 0.0
            self._last_stable_weight = 0.0
            self._is_initialized = False
            
            print("✓ HX711 initialization successful")
            
        except Exception as e:
            raise RuntimeError(f"Failed to initialize HX711: {e}")
    
    def warmup(self):
        """
        Warmup the sensor by performing dummy readings
        """
        print("Warming up sensor...")
        for _ in range(WARMUP_SAMPLES):
            self._hx711.get_value()
            time.sleep(0.05)
        print("✓ Sensor warmup complete")
    
    def calibrate_zero(self, samples=20):
        """
        Calibrate the zero point (tare) of the scale
        
        Args:
            samples (int): Number of samples to average for tare value
        """
        print("\nZero Point Calibration")
        print("Please ensure the scale is empty")
        
        # Countdown
        for i in range(3, 0, -1):
            print(f"Starting in {i}...")
            time.sleep(1)
        
        # Warmup before calibration
        self.warmup()
        
        print("Calibrating...")
        
        # Perform HX711 tare operation
        self._hx711.tare()
        
        # Collect samples for accurate zero point
        tare_samples = []
        for i in range(samples):
            reading = self._hx711.get_value()
            tare_samples.append(reading)
            if i % 5 == 0:
                print(f"  Progress: {i}/{samples}")
            time.sleep(0.05)
        
        # Calculate robust average (remove outliers)
        tare_samples.sort()
        trim_count = samples // 5  # Remove top and bottom 20%
        trimmed_samples = tare_samples[trim_count:-trim_count]
        self._tare_offset = sum(trimmed_samples) / len(trimmed_samples)
        
        self._is_initialized = True
        
        print(f"✓ Calibration complete")
        print(f"  Zero offset: {self._tare_offset:.0f} LSB")
    
    def get_raw_value(self):
        """
        Get raw ADC value from HX711
        
        Returns:
            int: Raw ADC reading
        """
        return self._hx711.get_value()
    
    def get_weight_fast(self):
        """
        Get weight measurement with fast stabilization algorithm
        
        Returns:
            tuple: (weight in grams, is_stable boolean)
        """
        if not self._is_initialized:
            raise RuntimeError("Sensor not calibrated. Call calibrate_zero() first.")
        
        # Collect rapid samples
        samples = []
        for _ in range(RAPID_SAMPLE_COUNT):
            raw_value = self.get_raw_value()
            weight = (raw_value - self._tare_offset) / CALIBRATION_FACTOR
            samples.append(weight)
            time.sleep(RAPID_SAMPLE_INTERVAL)
        
        # Analyze sample stability
        sample_range = max(samples) - min(samples)
        
        # If readings are highly variable, weight is changing
        if sample_range > 50.0:  # Large variation threshold
            return samples[-1], False
        
        # Calculate median for robustness
        samples.sort()
        median_weight = samples[RAPID_SAMPLE_COUNT // 2]
        
        # Check stability against last stable reading
        weight_change = abs(median_weight - self._last_stable_weight)
        is_stable = weight_change < STABILITY_THRESHOLD
        
        # Update stable weight if significant change detected
        if not is_stable and sample_range < 10.0:  # New stable weight
            self._last_stable_weight = median_weight
        
        return median_weight if not is_stable else self._last_stable_weight, is_stable
    
    def continuous_measurement(self, interval=1.0, callback=None):
        """
        Perform continuous weight measurements
        
        Args:
            interval (float): Time interval between measurements (seconds)
            callback (function): Optional callback function(weight, is_stable, count)
        """
        print(f"\nContinuous Weight Measurement (Interval: {interval}s)")
        print("Press Ctrl+C to stop")
        print("-" * 50)
        print("Count |   Weight   | Status")
        print("-" * 50)
        
        measurement_count = 0
        
        try:
            while True:
                measurement_count += 1
                start_time = time.time()
                
                # Get weight measurement
                weight, is_stable = self.get_weight_fast()
                
                # Status indicator
                status = "STABLE" if is_stable else "CHANGING"
                
                # Display measurement
                print(f"{measurement_count:5d} | {weight:8.2f} g | {status}")
                
                # Call callback if provided
                if callback:
                    callback(weight, is_stable, measurement_count)
                
                # Maintain interval timing
                elapsed = time.time() - start_time
                sleep_time = max(0, interval - elapsed)
                time.sleep(sleep_time)
                
        except KeyboardInterrupt:
            print("\n" + "-" * 50)
            print(f"Measurement stopped. Total readings: {measurement_count}")
    
    def get_statistics(self, duration=10):
        """
        Collect weight statistics over a specified duration
        
        Args:
            duration (int): Collection duration in seconds
            
        Returns:
            dict: Statistics including mean, std_dev, min, max
        """
        print(f"Collecting statistics for {duration} seconds...")
        
        weights = []
        stable_count = 0
        
        start_time = time.time()
        while time.time() - start_time < duration:
            weight, is_stable = self.get_weight_fast()
            weights.append(weight)
            if is_stable:
                stable_count += 1
            time.sleep(0.2)
        
        # Calculate statistics
        mean_weight = sum(weights) / len(weights)
        variance = sum((w - mean_weight) ** 2 for w in weights) / len(weights)
        std_dev = variance ** 0.5
        
        return {
            'samples': len(weights),
            'mean': mean_weight,
            'std_dev': std_dev,
            'min': min(weights),
            'max': max(weights),
            'stability_rate': stable_count / len(weights) * 100
        }


def main():
    """
    Main application entry point
    """
    try:
        # Initialize sensor
        sensor = WeightSensor(data_pin=PIN_DATA, clock_pin=PIN_CLOCK)
        
        # Calibrate zero point
        sensor.calibrate_zero()
        
        # Start continuous measurement
        sensor.continuous_measurement(interval=1.0)
        
    except Exception as e:
        print(f"Error: {e}")
        raise


if __name__ == "__main__":
    main()
