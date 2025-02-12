import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

max_radius = 7 # (m) Maximum permissible radius as issued by the MAV_CMD_DO_CONTAINMENT_MANAGER mission item
max_pos = 10 # (m) Maximum radius to vary the copters position by
rad_pos = np.linspace(0, max_pos, 200) # (m) Copters actual radial position from mission leg
dead_zone_pct = 0.1
min_time_scaler = 0.1

def calc_time_scaler(pos):
    norm_error = pos / max_radius

    # Scale and shift the norm error so that the inner dead zone of the radius moves with full speed (i.e. time scaler = 1)
    # Also set a minimum time scaler that we cannot scale beyond
    scaler = 1.0 + dead_zone_pct - norm_error
    scaler = min(max(scaler, min_time_scaler), 1.0)
    return scaler

# Plot the time scaler
time_scaler = []
for pos in rad_pos:
    time_scaler.append(calc_time_scaler(pos))

fig, ax = plt.subplots()
ax.plot(rad_pos, time_scaler)
ax.set_xlabel('Copters Radial Position (m)')
ax.set_ylabel('Time Scaler (No Units)')
ax.set_xlim([0, max_pos])
ax.set_ylim([0, 1.5])
plt.grid()


# Draw an example vertical velocity profile
rad_pos = np.linspace(0, max_pos, 200)

# Define the grid for North and East position (x and y in NEU frame)
x = np.linspace(-max_pos, max_pos, 200)  # East position
y = np.linspace(-max_pos, max_pos, 200)  # North position
X, Y = np.meshgrid(x, y)

# Define the velocity profile
V0 = -2.5  # Base descent velocity at the center (x=0, y=0)
R = np.sqrt(X**2 + Y**2)  # Radius from the center
t_scaler = np.vectorize(calc_time_scaler)(R)  # Compute scaler using the function element-wise
Z = V0 * t_scaler # scaled velocity profile

# Plot the 3D surface
fig = plt.figure(figsize=(10, 7))
ax = fig.add_subplot(111, projection='3d')

# Surface plot
ax.plot_surface(X, Y, Z, cmap='viridis', edgecolor='none')

# Labels
ax.set_xlabel("East Position (m)")
ax.set_ylabel("North Position (m)")
ax.set_zlabel("Velocity in Z (m/s)")
ax.set_title("3D Surface of Velocity Profile in NEU Frame")

# Show the plots
plt.show()


