"""
Simple Food Weight Detection System with ST7789 Display - Hardware Module
HX711 Weight Sensor + ST7789 Display + Serial Communication

Features:
- HX711 weight sensor interface with error handling
- ST7789 240x240 color display for real-time information
- Display AI analysis results, weight, and carbon footprint
- Simple text-based serial communication
- Autonomous weight monitoring and sending
"""

from machine import Pin, SPI
from hx711_gpio import HX711
import time
import gc
import st7789
import vga1_16x32 as font

# =========================================
# HARDWARE CONFIGURATION
# =========================================

# HX711 Weight Sensor
PIN_DATA = 8      # HX711 data pin (DOUT)
PIN_CLOCK = 9     # HX711 clock pin (SCK)

# ST7789 Display Configuration
WIDTH, HEIGHT = 240, 240
SPI_NUM = 0       # SPI bus number
BACKLIGHT_PIN = 0 # Display backlight pin
RST_PIN = 13      # Display reset pin
DC_PIN = 12       # Display data/command pin
CS_PIN = 1        # Display chip select pin
SCK_PIN = 18      # SPI clock pin (SCL)
MOSI_PIN = 19     # SPI data pin (SDA)

# Sensor Configuration
CALIBRATION_FACTOR = 419.0  # Calibration factor (unit: LSB/g)
STABILITY_THRESHOLD = 5.0   # Weight stability threshold (grams)
RAPID_SAMPLE_COUNT = 5      # Number of rapid samples for stabilization
RAPID_SAMPLE_INTERVAL = 0.05  # Interval between rapid samples (seconds)
WARMUP_SAMPLES = 10         # Number of warmup samples during initialization

# Weight sending configuration
MIN_WEIGHT_THRESHOLD = 5.0  # Minimum weight to consider (grams)
WEIGHT_CHANGE_THRESHOLD = 2.0  # Minimum weight change to trigger new send (grams)
TIME_BETWEEN_SENDS = 5.0    # Minimum time between sends (seconds)

# Initialize SPI and Display
spi = SPI(SPI_NUM, baudrate=31250000, sck=Pin(SCK_PIN), mosi=Pin(MOSI_PIN))
tft = st7789.ST7789(
    spi, WIDTH, HEIGHT,
    reset=Pin(RST_PIN, Pin.OUT),
    cs=Pin(CS_PIN, Pin.OUT),
    dc=Pin(DC_PIN, Pin.OUT),
    backlight=Pin(BACKLIGHT_PIN, Pin.OUT),
    rotation=0,
)


class DisplayManager:
    """
    ST7789 Display Manager for showing AI analysis results and system status
    """
    
    def __init__(self, tft_display):
        """Initialize display manager"""
        self.tft = tft_display
        self.font = font
        
        # Display areas layout (240x240 pixels)
        self.header_area = (0, 0, 240, 40)      # System title
        self.weight_area = (0, 45, 240, 80)     # Weight display  
        self.food_area = (0, 90, 240, 130)      # Food name
        self.carbon_area = (0, 135, 240, 175)   # Carbon footprint
        self.status_area = (0, 180, 240, 220)   # System status
        self.footer_area = (0, 225, 240, 240)   # Time/info
        
        # Colors
        self.BLACK = st7789.BLACK
        self.WHITE = st7789.WHITE
        self.RED = st7789.RED
        self.GREEN = st7789.GREEN
        self.BLUE = st7789.BLUE
        self.YELLOW = st7789.YELLOW
        self.CYAN = st7789.CYAN
        self.MAGENTA = st7789.MAGENTA
        
        # Current display state
        self.current_weight = 0.0
        self.current_food = ""
        self.current_carbon = 0.0
        self.current_status = "Initializing..."
        self.ai_result_received = False
        
        # Initialize display
        self.init_display()
    
    def init_display(self):
        """Initialize display with welcome screen"""
        print("Initializing ST7789 Display...")
        
        # Clear screen
        self.tft.fill(self.BLACK)
        
        # Display welcome message
        self.draw_text_centered("Carbon Detection", 10, self.WHITE)
        self.draw_text_centered("System", 45, self.WHITE)
        self.draw_text_centered("Hardware Ready", 100, self.CYAN)
        self.draw_text_centered("Waiting...", 140, self.YELLOW)
        
        print("Display initialized successfully")
    
    def draw_text_centered(self, text, y, color):
        """Draw text centered horizontally at given y position using vga1_16x32 font"""
        try:
            # Calculate text width (16 pixels per character for vga1_16x32)
            text_width = len(text) * 16
            x = max(0, (WIDTH - text_width) // 2)
            self.tft.text(self.font, text, x, y, color, self.BLACK)
        except Exception as e:
            print(f"Text draw error: {e}")
    
    def draw_text_left(self, text, x, y, color):
        """Draw text at specified position"""
        try:
            self.tft.text(self.font, text, x, y, color, self.BLACK)
        except Exception as e:
            print(f"Text draw error: {e}")
    
    def clear_line(self, y):
        """Clear a line at specified y position"""
        self.tft.fill_rect(0, y, WIDTH, 32, self.BLACK)
    
    def update_weight(self, weight, is_stable=False):
        """Update weight display"""
        self.current_weight = weight
        
        # Choose color based on stability
        color = self.GREEN if is_stable else self.YELLOW
        bg_color = self.BLACK
        
        # Format weight text
        if weight > 1000:
            weight_text = f"{weight/1000:.2f}kg"
        else:
            weight_text = f"{weight:.1f}g"
        
        # Update display
        self.draw_text_area(weight_text, self.weight_area, color, bg_color)
        
        # Update status indicator
        status_text = "STABLE" if is_stable else "MEASURING"
        status_color = self.GREEN if is_stable else self.YELLOW
        self.tft.fill_rect(200, 45, 40, 35, status_color)
    
    def display_analysis_result(self, food_name, confidence, weight, co2_grams, impact_level):
        """Display complete AI analysis result on screen"""
        print(f"Displaying: {food_name}, {confidence}%, {weight}g, {co2_grams}g CO2, {impact_level}")
        
        # Clear screen
        self.tft.fill(self.BLACK)
        
        # Title
        self.draw_text_centered("Carbon Analysis", 5, self.WHITE)
        
        # Food info
        food_text = f"Food: {food_name[:10]}"  # Limit to 10 chars
        self.draw_text_left(food_text, 5, 40, self.CYAN)
        
        conf_text = f"Conf: {confidence}%"
        self.draw_text_left(conf_text, 5, 70, self.CYAN)
        
        # Weight info
        weight_text = f"Weight: {weight:.1f}g"
        self.draw_text_left(weight_text, 5, 105, self.YELLOW)
        
        # CO2 info
        if co2_grams > 1000:
            co2_text = f"CO2: {co2_grams/1000:.2f}kg"
        else:
            co2_text = f"CO2: {co2_grams:.2f}g"
        
        # Color based on impact
        if impact_level == "LOW":
            co2_color = self.GREEN
        elif impact_level == "MEDIUM":
            co2_color = self.YELLOW
        elif impact_level == "HIGH":
            co2_color = st7789.color565(255, 165, 0)  # Orange
        else:  # VERY_HIGH
            co2_color = self.RED
            
        self.draw_text_left(co2_text, 5, 140, co2_color)
        
        # Impact level
        impact_text = f"Impact: {impact_level}"
        self.draw_text_left(impact_text, 5, 175, co2_color)
        
        # Status
        self.draw_text_centered("Analysis Complete", 210, self.GREEN)
        
        # Store current state
        self.current_food = food_name
        self.current_carbon = co2_grams
        self.ai_result_received = True
    
    def update_status(self, status, color=None):
        """Update system status display"""
        self.current_status = status
        display_color = color or self.WHITE
        
        # Truncate long status messages
        display_status = status[:18] if len(status) > 18 else status
        
        self.draw_text_area(display_status, self.status_area, display_color, self.BLACK)
    
    def show_ai_analysis(self, food_name, confidence, carbon_footprint):
        """Display complete AI analysis result"""
        print(f"Displaying AI result: {food_name}, confidence: {confidence}, carbon: {carbon_footprint}")
        
        # Clear previous AI result area
        self.tft.fill_rect(0, 90, 240, 85, self.BLACK)
        
        # Update food name
        self.update_food_info(food_name, confidence)
        
        # Update carbon footprint
        self.update_carbon_footprint(carbon_footprint)
        
        # Update status
        self.update_status("AI Analysis Complete", self.GREEN)
        
        self.ai_result_received = True
    
    def show_waiting_for_ai(self):
        """Show waiting for AI analysis message"""
        self.update_status("Waiting for AI...", self.CYAN)
        self.tft.fill_rect(0, 90, 240, 85, self.BLACK)  # Clear food/carbon area
        self.draw_text_centered("Analyzing food...", 110, self.CYAN)
    
    def show_system_ready(self):
        """Show system ready message"""
        self.tft.fill(self.BLACK)
        self.draw_text_centered("System Ready", 60, self.GREEN)
        self.draw_text_centered("Place food on", 100, self.WHITE)
        self.draw_text_centered("scale for analysis", 120, self.WHITE)
        self.update_status("Ready", self.GREEN)
    
    def show_error(self, error_msg):
        """Show error message"""
        self.update_status(f"ERROR: {error_msg}", self.RED)
    
    def refresh_display(self):
        """Refresh the entire display with current values"""
        try:
            # Header
            self.draw_text_centered("Carbon Detection", 5, self.WHITE)
            
            # Current values
            self.update_weight(self.current_weight)
            if self.current_food:
                self.update_food_info(self.current_food)
            if self.current_carbon > 0:
                self.update_carbon_footprint(self.current_carbon)
            self.update_status(self.current_status)
            
        except Exception as e:
            print(f"Display refresh error: {e}")


class ResultReceiver:
    """Receive and parse AI analysis results from PC"""
    
    def __init__(self, display_manager):
        self.display = display_manager
        self.buffer = ""
        
    def process_serial_input(self, line):
        """Process incoming serial data from PC"""
        try:
            line = line.strip()
            print(f"Received from PC: {line}")
            
            # Parse AI result message: AI_RESULT:food:confidence:weight:co2:impact
            if line.startswith("AI_RESULT:"):
                parts = line.split(":")
                if len(parts) >= 6:
                    _, food_name, confidence, weight, co2_grams, impact_level = parts[:6]
                    
                    # Convert to appropriate types
                    confidence = int(confidence)
                    weight = float(weight)
                    co2_grams = float(co2_grams)
                    
                    # Display on screen
                    self.display.display_analysis_result(
                        food_name, confidence, weight, co2_grams, impact_level
                    )
                    
                    print(f"Analysis result displayed: {food_name}")
                    return True
            
            return False
            
        except Exception as e:
            print(f"Error processing serial input: {e}")
            return False


class WeightSensor:
    """
    Simplified HX711 weight sensor interface with simulation fallback
    """
    
    def __init__(self, data_pin=PIN_DATA, clock_pin=PIN_CLOCK):
        """Initialize the weight sensor with retry and fallback"""
        print("Initializing HX711 Weight Sensor...")
        
        self._tare_offset = 0.0
        self._last_stable_weight = 0.0
        self._is_initialized = False
        self._simulation_mode = False
        self._sim_weight = 0.0
        
        # Try to initialize HX711 with retries
        for attempt in range(3):
            try:
                print(f"HX711 initialization attempt {attempt + 1}...")
                
                # Configure GPIO pins
                self._pin_data = Pin(data_pin, Pin.IN, pull=Pin.PULL_DOWN)
                self._pin_clock = Pin(clock_pin, Pin.OUT)
                
                # Initialize HX711 driver
                self._hx711 = HX711(self._pin_clock, self._pin_data)
                
                # Test reading
                test_value = self._hx711.get_value()
                print(f"Test reading: {test_value}")
                
                print("HX711 initialization successful!")
                self._is_initialized = True
                break
                
            except Exception as e:
                print(f"HX711 init attempt {attempt + 1} failed: {e}")
                time.sleep(1)
        
        # If HX711 failed, use simulation mode
        if not self._is_initialized:
            print("WARNING: HX711 initialization failed - using SIMULATION MODE")
            print("System will generate simulated weight data for testing")
            self._simulation_mode = True
            self._is_initialized = True
    
    def warmup(self):
        """Warmup the sensor by performing dummy readings"""
        if self._simulation_mode:
            print("Simulation mode - skipping warmup")
            return
            
        print("Warming up sensor...")
        for _ in range(WARMUP_SAMPLES):
            try:
                self._hx711.get_value()
                time.sleep(0.05)
            except:
                break
        print("Sensor warmup complete")
    
    def calibrate_zero(self, samples=20):
        """Calibrate the zero point (tare) of the scale"""
        print("Zero Point Calibration")
        print("Please ensure the scale is empty")
        
        if self._simulation_mode:
            print("Simulation mode - using default zero offset")
            self._tare_offset = 0.0
            self._is_initialized = True
            print("Calibration complete (simulated)")
            return
        
        # Countdown
        for i in range(3, 0, -1):
            print(f"Starting in {i}...")
            time.sleep(1)
        
        # Warmup before calibration
        self.warmup()
        
        print("Calibrating...")
        
        try:
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
            trimmed_samples = tare_samples[trim_count:-trim_count] if trim_count > 0 else tare_samples
            self._tare_offset = sum(trimmed_samples) / len(trimmed_samples)
            
            print(f"Calibration complete - Zero offset: {self._tare_offset:.0f} LSB")
            
        except Exception as e:
            print(f"Calibration error: {e}")
            print("Using default zero offset")
            self._tare_offset = 0.0
    
    def get_raw_value(self):
        """Get raw ADC value from HX711"""
        try:
            if self._simulation_mode:
                # Return simulated raw value
                return int(self._sim_weight * CALIBRATION_FACTOR + self._tare_offset)
            return self._hx711.get_value()
        except Exception as e:
            print(f"Raw value error: {e}")
            raise e  # Re-raise the exception
    
    def get_weight_fast(self):
        """
        Get weight measurement with fast stabilization algorithm
        
        Returns:
            tuple: (weight in grams, is_stable boolean)
        """
        if not self._is_initialized:
            print("ERROR: Sensor not calibrated")
            return 0.0, False
        
        if self._simulation_mode:
            # Simulate weight changes for testing
            try:
                import time
                cycle_time = int(time.time()) % 30  # 30-second cycle
                
                if cycle_time < 10:
                    self._sim_weight = 0.0  # Empty scale
                elif cycle_time < 20:
                    self._sim_weight = 125.0 + (cycle_time % 3) * 2  # Stable weight with small variation
                else:
                    self._sim_weight = 80.0 + cycle_time  # Changing weight
                
                # Simulate stability based on weight change
                weight_change = abs(self._sim_weight - self._last_stable_weight)
                is_stable = weight_change < STABILITY_THRESHOLD and self._sim_weight > MIN_WEIGHT_THRESHOLD
                
                if is_stable:
                    self._last_stable_weight = self._sim_weight
                
                return self._sim_weight, is_stable
            except Exception as e:
                print(f"Simulation error: {e}")
                return 0.0, False
        
        # Real sensor measurement - initialize ALL variables first
        samples = []
        median_weight = 0.0
        is_stable = False
        sample_range = 0.0
        weight_change = 0.0
        
        try:
            # Collect rapid samples with individual error handling
            for i in range(RAPID_SAMPLE_COUNT):
                try:
                    raw_value = self.get_raw_value()
                    weight = (raw_value - self._tare_offset) / CALIBRATION_FACTOR
                    samples.append(weight)
                except Exception as e:
                    print(f"Sample {i} error: {e}")
                    # Use last known weight if available
                    if samples:
                        samples.append(samples[-1])
                    else:
                        samples.append(0.0)
                
                try:
                    time.sleep(RAPID_SAMPLE_INTERVAL)
                except:
                    pass  # Ignore sleep errors
            
            # Ensure we have samples
            if not samples:
                print("ERROR: No samples collected")
                return 0.0, False
            
            # Analyze sample stability
            try:
                sample_range = max(samples) - min(samples)
            except:
                sample_range = 0.0
            
            # Calculate median for robustness
            try:
                samples.sort()
                median_weight = samples[len(samples) // 2]
            except:
                median_weight = 0.0
            
            # If readings are highly variable, weight is changing
            if sample_range > 50.0:  # Large variation threshold
                return median_weight, False
            
            # Check stability against last stable reading
            try:
                weight_change = abs(median_weight - self._last_stable_weight)
                is_stable = weight_change < STABILITY_THRESHOLD
            except:
                is_stable = False
            
            # Update stable weight if significant change detected
            try:
                if not is_stable and sample_range < 10.0:  # New stable weight
                    self._last_stable_weight = median_weight
            except:
                pass
            
            # Return appropriate weight
            try:
                if is_stable:
                    return self._last_stable_weight, True
                else:
                    return median_weight, False
            except:
                return 0.0, False
            
        except Exception as e:
            print(f"Weight reading error: {e}")
            return 0.0, False


class SimpleWeightSystem:
    """
    Main system controller with display integration
    """
    
    def __init__(self):
        """Initialize the system with display"""
        print("=== Food Carbon Detection System with Display ===")
        print("Hardware Module - Raspberry Pi Pico + ST7789")
        print("Weight sensing + AI analysis display")
        print()
        
        # Initialize display manager
        self.display = DisplayManager(tft)
        
        # Initialize weight sensor
        self.weight_sensor = WeightSensor()
        
        # Initialize result receiver
        self.result_receiver = ResultReceiver(self.display)
        
        # System state for weight monitoring
        self.last_sent_weight = 0.0
        self.last_sent_time = 0
        
        # AI analysis state
        self.current_ai_result = None
        self.waiting_for_ai = False
        
        print("System initialization complete")
    
    def initialize(self):
        """Initialize and calibrate the system"""
        try:
            # Calibrate weight sensor
            self.weight_sensor.calibrate_zero()
            
            print("System ready for operation")
            print("Starting autonomous weight monitoring...")
            
        except Exception as e:
            print(f"Initialization error: {e}")
    
    def send_weight_message(self, weight, is_stable):
        """Send weight data using simple text protocol"""
        try:
            # Format: WEIGHT:123.5:STABLE or WEIGHT:123.5:CHANGING
            stability_text = "STABLE" if is_stable else "CHANGING"
            message = f"WEIGHT:{weight:.1f}:{stability_text}"
            
            # Send via USB serial (print function)
            print(message)
            
            return True
            
        except Exception as e:
            print(f"Send error: {e}")
            return False
    
    def send_status_message(self):
        """Send system status message"""
        try:
            current_weight, is_stable = self.weight_sensor.get_weight_fast()
            mode = "SIMULATION" if self.weight_sensor._simulation_mode else "REAL"
            message = f"STATUS:READY:MODE:{mode}:WEIGHT:{current_weight:.1f}"
            print(message)
        except Exception as e:
            print(f"Status error: {e}")
    
    def check_pc_input(self):
        """Check for incoming data from PC"""
        try:
            import sys
            import select
            
            # Check if there's input available (non-blocking)
            if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
                line = sys.stdin.readline()
                if line:
                    self.result_receiver.process_serial_input(line)
        except:
            # For MicroPython without select, use simpler method
            try:
                import sys
                if hasattr(sys.stdin, 'readline'):
                    # Try to read without blocking (may not work on all platforms)
                    line = sys.stdin.readline()
                    if line:
                        self.result_receiver.process_serial_input(line)
            except:
                pass  # No input available or not supported
    
    def run(self):
        """Main system loop - continuous weight monitoring"""
        print("Starting autonomous weight monitoring system...")
        print("Weight will be sent automatically when detected!")
        print()
        
        # Weight monitoring state
        stable_count = 0
        required_stable = 3  # Need 3 consecutive stable readings
        loop_counter = 0
        
        # Send initial status
        self.send_status_message()
        
        try:
            while True:
                loop_counter += 1
                
                # Show loop activity every 500 iterations (about every 100 seconds)
                if loop_counter % 500 == 0:
                    print(f"MSG:LOOP:{loop_counter}:MONITORING")
                
                try:
                    # Check for incoming PC data
                    self.check_pc_input()
                    
                    # Get current weight
                    weight, is_stable = self.weight_sensor.get_weight_fast()
                    current_time = time.time()
                    
                    # Check if weight is significant
                    if weight > MIN_WEIGHT_THRESHOLD:
                        if is_stable:
                            stable_count += 1
                            
                            # Check if we have enough stable readings
                            if stable_count >= required_stable:
                                # Check if this is a new weight worth sending
                                weight_diff = abs(weight - self.last_sent_weight)
                                time_diff = current_time - self.last_sent_time
                                
                                # Send if weight changed significantly or enough time passed
                                should_send = (
                                    weight_diff > WEIGHT_CHANGE_THRESHOLD or 
                                    self.last_sent_time == 0 or 
                                    time_diff > TIME_BETWEEN_SENDS
                                )
                                
                                if should_send:
                                    success = self.send_weight_message(weight, True)
                                    if success:
                                        self.last_sent_weight = weight
                                        self.last_sent_time = current_time
                                        print(f"MSG:SENT:WEIGHT:{weight:.1f}g")
                                
                                stable_count = 0  # Reset counter
                        else:
                            stable_count = 0
                            
                        # Show current reading every 50 loops when weight detected
                        if loop_counter % 50 == 0:
                            status = "STABLE" if is_stable else "CHANGING"
                            print(f"MSG:CURRENT:{weight:.1f}g:{status}")
                            
                    else:
                        # No significant weight
                        stable_count = 0
                    
                    # Memory management
                    if loop_counter % 100 == 0:
                        gc.collect()
                    
                    # Small delay
                    time.sleep(0.2)  # 5 readings per second
                    
                except Exception as e:
                    print(f"ERROR:WEIGHT_MONITORING:{e}")
                    time.sleep(1)
                
        except KeyboardInterrupt:
            print("MSG:SYSTEM:STOPPED_BY_USER")
        except Exception as e:
            print(f"ERROR:SYSTEM:{e}")


def main():
    """Main application entry point"""
    try:
        # Create and initialize system
        system = SimpleWeightSystem()
        system.initialize()
        
        # Run main loop
        system.run()
        
    except Exception as e:
        print(f"ERROR:FATAL:{e}")


if __name__ == "__main__":
    main()

