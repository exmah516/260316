import pyads
import time
import numpy as np

# Configuration
PLC_PORT = 851
VAR_NAME = "G.fn_value"
GRAVITY = 9.80665  # m/s^2

# Standard AMS Net ID for local loopback. 
# If this fails, we will try the one found in C++ code: '169.254.119.135.1.1'
LOCAL_NET_ID = '127.0.0.1.1.1' 
FALLBACK_NET_ID = '169.254.119.135.1.1'

def connect_to_plc():
    """Attempt to connect to the PLC."""
    print(f"Connecting to PLC (Port {PLC_PORT})...")
    
    # Try local loopback first
    try:
        plc = pyads.Connection(LOCAL_NET_ID, PLC_PORT)
        plc.open()
        # Read status to verify connection
        plc.read_state()
        print(f"Connected using NetID: {LOCAL_NET_ID}")
        return plc
    except Exception as e:
        print(f"Failed to connect to {LOCAL_NET_ID}: {e}")
    
    # Try fallback NetID
    try:
        print(f"Trying fallback NetID: {FALLBACK_NET_ID}...")
        plc = pyads.Connection(FALLBACK_NET_ID, PLC_PORT)
        plc.open()
        plc.read_state()
        print(f"Connected using NetID: {FALLBACK_NET_ID}")
        return plc
    except Exception as e:
        print(f"Failed to connect to {FALLBACK_NET_ID}: {e}")
    
    return None

def main():
    # 1. Connect to PLC
    plc = connect_to_plc()
    if not plc:
        print("Could not connect to PLC. Please check TwinCAT is running and routes are configured.")
        return

    # 2. Define Calibration Points
    # Added 0g as a baseline
    weights_g = [0, 1, 2, 5, 10, 20, 50, 100, 200]
    
    sensor_data = []  # Stores average sensor values
    force_data = []   # Stores calculated force (N)

    print("\n" + "="*50)
    print("Force Sensor Calibration Assistant")
    print(f"Variable to calibrate: {VAR_NAME}")
    print("="*50 + "\n")

    try:
        # 3. Measurement Loop
        for weight in weights_g:
            input(f">> Please place {weight}g weight (or 0 for empty) and press ENTER to measure...")
            
            print(f"   Measuring {weight}g...", end="", flush=True)
            
            # Collect data for ~2 seconds
            samples = []
            start_time = time.time()
            duration = 2.0 # seconds
            
            while time.time() - start_time < duration:
                try:
                    # Read INT value from PLC
                    val = plc.read_by_name(VAR_NAME, pyads.PLCTYPE_INT)
                    samples.append(val)
                except pyads.ADSError as e:
                    print(f"\n   Error reading {VAR_NAME}: {e}")
                time.sleep(0.05) # 20Hz sampling
            
            if not samples:
                print("\n   No data collected! Skipping this point.")
                continue
                
            avg_val = np.mean(samples)
            std_val = np.std(samples)
            
            # Calculate Force: F = m * g
            force_n = (weight / 1000.0) * GRAVITY
            
            sensor_data.append(avg_val)
            force_data.append(force_n)
            
            print(f" Done. Avg: {avg_val:.2f} (Std: {std_val:.2f}), Force: {force_n:.4f} N")
            
    except KeyboardInterrupt:
        print("\n\nCalibration interrupted by user.")
    finally:
        plc.close()

    # 4. Data Analysis & Fitting
    if len(sensor_data) < 2:
        print("\nNot enough data points for calibration.")
        return

    print("\n" + "-"*50)
    print("Calculating calibration parameters...")
    
    x = np.array(sensor_data) # Sensor Value
    y = np.array(force_data)  # Force (N)
    
    # Linear Fit: Force = k * Sensor + b
    # deg=1 means linear
    slope, intercept = np.polyfit(x, y, 1)
    
    print("-" * 50)
    print("CALIBRATION RESULT:")
    print(f"Slope (k):     {slope:.8f}  (N / unit)")
    print(f"Intercept (b): {intercept:.8f}  (N)")
    print("-" * 50)
    print(f"Formula: Force (N) = {slope:.6e} * {VAR_NAME} + ({intercept:.6e})")
    print("-" * 50)
    
    # 5. Verification Table
    print("\nVerification:")
    print(f"{'Weight(g)':<10} | {'Sensor':<10} | {'Actual(N)':<10} | {'Pred(N)':<10} | {'Error(N)':<10}")
    print("-" * 60)
    
    for i, w in enumerate(weights_g):
        if i < len(sensor_data): # Handle if loop was interrupted
            s_val = sensor_data[i]
            act_f = force_data[i]
            pred_f = slope * s_val + intercept
            error = pred_f - act_f
            print(f"{w:<10} | {s_val:<10.2f} | {act_f:<10.4f} | {pred_f:<10.4f} | {error:<10.4f}")

    print("\nTo use this in your code:")
    print(f"force_newton = {slope:.8f} * {VAR_NAME} + {intercept:.8f}")

if __name__ == "__main__":
    main()
