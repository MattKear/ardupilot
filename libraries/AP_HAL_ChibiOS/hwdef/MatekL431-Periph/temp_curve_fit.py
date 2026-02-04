# ======================================================================
# 
#   Fitting voltage curve from temperature calibration experiment data
#
# ======================================================================

import numpy as np
import matplotlib.pyplot as plt
import math


# ADC Voltage measured by Periph node
V = [0.86,0.98,1.1,1.25,1.38,1.53,1.69,1.8,1.94,2.06,2.18,2.31,2.5,2.63,2.73]
T = [15,20,25,30,35,40,45,50,55,60,65,70,80,90,100]

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
