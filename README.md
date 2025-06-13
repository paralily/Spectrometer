The spectrometer full code comprises of the following -
Photometer Mode: Measures absorbance at a specific wavelength selected by the user.

Spectrometer Mode: Measures absorbance at multiple predefined wavelengths and reports the maximum value.

Interpolation Modes:

Lagrange Interpolation: Fits a polynomial through all data points.

Linear Interpolation: Connects data points with straight lines.

Cubic Spline Interpolation: Connects data points with smooth, piecewise cubic curves.

Motor Control: Automated stepper motor control for wavelength selection and system calibration.

User Interaction: Simple serial interface for mode selection and data input.
Hardware requirements :
Sensor: Adafruit AS7341 spectral sensor

Stepper Motor Driver: A4988 

Stepper Motor: NEMA 17 

Usage:
Photometer Mode:

Select a wavelength index (1-6) to measure absorbance at that wavelength.

The system reports the calculated absorbance.

Spectrometer Mode:

The system measures absorbance at all available wavelengths.

Displays the absorbance for each wavelength and the maximum value.

Interpolation Modes:

Lagrange, Linear, or Cubic Spline Interpolation: Available after running Spectrometer mode.

The system outputs interpolated values for each data point.

Motor Control:

The system automatically moves the motor by 60 degrees each loop .😊


