import numpy as np
import matplotlib.pyplot as plt
import math

# Calculate AP polynomial for thermistors
# https://www.amazon.co.uk/Thermistor-Temperature-Sensor-Cylinder-Probe/dp/B01FO3JUXA

# degC → Kelvin offset
deg2kel = 273.15

R0 = 10000.0
T0 = 25.0
B = 3435.0

# Temperature range (-30 to 110 inclusive)
T = np.arange(-30, 111, 1)

# Thermistor resistance
R = R0 * np.exp(B * ((1.0 / (T + deg2kel)) - (1.0 / (T0 + deg2kel))))

# Plot Resistance vs Temperature
plt.figure()
plt.plot(T, R)
plt.scatter([T0], [R0])
plt.xlabel("Temperature (degree celsius)")
plt.ylabel("Resistance (ohms)")
plt.grid(True)

# Voltage divider
VCC = 3.3
# VCC = 5.0
# RFixed = 10000.0
RFixed = 9500.0

# V = (VCC * R) / (R + RFixed)
V = (VCC * RFixed) / (R + RFixed)



# Plot Voltage vs Temperature
plt.figure()
plt.plot(T, V, label="Full Curve")
plt.xlabel("Temperature (degree celsius)")
plt.ylabel("Voltage (volts)")
plt.grid(True)

# Polynomial fit (Voltage → Temperature)
# polyfit returns highest power first (same as MATLAB)
p = np.polyfit(V, T, 5)

# Flip to match MATLAB's flipped coefficient order
p = np.flip(p)

# Evaluate polynomial manually (same structure as MATLAB loop)
temp = np.zeros_like(V)
poly = np.ones_like(V)

for coeff in p:
    temp += coeff * poly
    poly *= V

# Plot fitted curve
plt.plot(temp, V, label="Best Fit")
plt.legend()

# Error plot
plt.figure()
plt.plot(T, temp - T)
plt.xlabel("Temperature (degree celsius)")
plt.ylabel("Error (degree celsius)")
plt.grid(True)

# Print coefficients (same format as MATLAB)
# for j in range(1, 3):
j = 1
for i, coeff in enumerate(p):
    print(f"TEMP{j}_A{i},{coeff:.6f}")

plt.show()
