import serial
import time
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline

# Update your serial port as needed
SERIAL_PORT = 'COM3'
BAUD_RATE = 9600
TIMEOUT = 2

def wait_for_prompt(ser, prompt="Select Mode"):
    """Wait for the Arduino to print the mode selection prompt."""
    while True:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if line:
            print(line)
        if prompt in line:
            break

def send_mode_and_get_data(ser, mode):
    """Send the selected mode and read absorbance data if in spectrometer mode."""
    ser.write(f"{mode}\n".encode())
    time.sleep(0.5)  # Give Arduino time to process

    # Wait for confirmation (optional, adjust as per your Arduino code)
    while True:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if line:
            print(line)
        if "Mode Selected" in line or "Spectrometer Mode Selected" in line:
            break

    if mode != 2:
        print("Selected mode is not spectrometer mode. Exiting.")
        return None, None

    # Read absorbance values
    y_values = []
    max_abs = None
    while True:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if line:
            print(line)
        if line.startswith("Absorbance") and "Max" not in line:
            try:
                val = float(line.split(": ")[1])
                y_values.append(val)
            except (IndexError, ValueError):
                continue
        elif "Max Absorbance" in line:
            try:
                max_abs = float(line.split(": ")[1])
            except (IndexError, ValueError):
                max_abs = None
            break
        if len(y_values) >= 6 and max_abs is not None:
            break
    return np.array(y_values), max_abs

def lagrange_interp(x, x_points, y_points):
    total = 0.0
    n = len(x_points)
    for i in range(n):
        term = y_points[i]
        for j in range(n):
            if i != j:
                term *= (x - x_points[j]) / (x_points[i] - x_points[j])
        total += term
    return total

def plot_interpolations(x_data, y_data, max_abs):
    x_fine = np.linspace(x_data.min(), x_data.max(), 300)
    y_lagrange = np.array([lagrange_interp(xi, x_data, y_data) for xi in x_fine])
    y_linear = np.interp(x_fine, x_data, y_data)
    cs = CubicSpline(x_data, y_data, bc_type='natural')
    y_cubic = cs(x_fine)

    plt.figure(figsize=(12, 6))
    plt.plot(x_data, y_data, 'ko', label='Original Data', markersize=8)
    plt.plot(x_fine, y_lagrange, '--', label='Lagrange', linewidth=1.5)
    plt.plot(x_fine, y_linear, '-.', label='Linear', linewidth=1.5)
    plt.plot(x_fine, y_cubic, '-', label='Cubic Spline', linewidth=2)

    if max_abs is not None:
        plt.title(f'Spectral Data (Max Absorbance: {max_abs:.2f}%)', fontsize=14)
    else:
        plt.title('Spectral Data (Max Absorbance: N/A)', fontsize=14)

    plt.xlabel('Wavelength Position', fontsize=12)
    plt.ylabel('Absorbance (%)', fontsize=12)
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.show()

def main():
    x_data = np.array([87, 128, 180, 190, 196, 211])  # Arduino wavelength positions
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT)
        print("Serial port opened.")
        wait_for_prompt(ser)
        # Show menu and ask user for mode
        mode = int(input("Enter the mode number to select (e.g., 2 for Spectrometer Mode): ").strip())
        y_data, max_abs = send_mode_and_get_data(ser, mode)
        ser.close()
        if y_data is not None and max_abs is not None:
            print("Absorbance values:", y_data)
            print("Max Absorbance:", max_abs)
            plot_interpolations(x_data, y_data, max_abs)
        else:
            print("No absorbance data to plot.")
    except Exception as e:
        print(f"Serial error: {e}")

if __name__ == "_main_":
    main()