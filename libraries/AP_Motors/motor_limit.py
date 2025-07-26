import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.ticker import MaxNLocator

def cyclic_scaling():
    # Define inputs
    input1 = np.linspace(-1.0, 1.0, 35)
    input2 = input1

    # Create meshgrid
    X, Y = np.meshgrid(input1, input2)

    # Compute Z as the sum of X and Y
    Z = X + Y # Swash1
    Z2 = np.copy(Z) # Swash2

    Z_Scaled = np.copy(Z)
    Z2_Scaled = np.copy(Z)

    # Apply output constraints for every combination
    for i in range(Z.shape[0]):
        for j in range(Z.shape[1]):

            scale = 1.0
            if abs(Z[i, j]) > 1.0:
                scale = 1.0 / abs(Z[i, j])
                # This is where we would set the limit flag
            if abs(Z2[i, j]) > 1.0:
                scale = min(scale, 1.0 / abs(Z2[i, j]))
                # This is where we would set the limit flag

            Z_Scaled[i, j] *= scale
            Z2_Scaled[i, j] *= scale

    # Create figure with 2 subplots side by side
    fig = plt.figure(figsize=(14, 6))  # Adjust size as needed

    z_ticks = [-2.0, -1.5, -1.0, -0.5, 0, 0.5, 1.0, 1.5, 2.0]

    # First 3D plot
    ax1 = fig.add_subplot(1, 2, 1, projection='3d')
    surf1 = ax1.plot_surface(X, Y, Z, cmap='viridis', edgecolor='none', vmin=min(z_ticks), vmax=max(z_ticks))
    surf1_scaled = ax1.plot_wireframe(X, Y, Z_Scaled, color='black', linewidth=0.5)
    ax1.set_title('Swash 1')
    ax1.set_xlabel('Roll')
    ax1.set_ylabel('Yaw')
    ax1.set_zlabel('Swash1 Roll Output')
    ax1.view_init(elev=21, azim=-61)
    fig.colorbar(surf1, ax=ax1, shrink=0.5, aspect=10)
    ax1.xaxis.set_major_locator(MaxNLocator(nbins=5))
    ax1.yaxis.set_major_locator(MaxNLocator(nbins=5))
    ax1.set_zticks(z_ticks)

    # Second 3D plot
    ax2 = fig.add_subplot(1, 2, 2, projection='3d')
    surf2 = ax2.plot_surface(X, Y, Z2, cmap='plasma', edgecolor='none', vmin=min(z_ticks), vmax=max(z_ticks))
    surf2_scaled = ax2.plot_wireframe(X, Y, Z2_Scaled, color='black', linewidth=0.5)
    ax2.set_title('Swash 2')
    ax2.set_xlabel('Roll')
    ax2.set_ylabel('Yaw')
    ax2.set_zlabel('Swash2 Roll Output')
    ax2.view_init(elev=21, azim=132)
    fig.colorbar(surf2, ax=ax2, shrink=0.5, aspect=10, ticks=z_ticks)
    ax2.xaxis.set_major_locator(MaxNLocator(nbins=5))
    ax2.yaxis.set_major_locator(MaxNLocator(nbins=5))
    ax2.set_zticks(z_ticks)

    plt.suptitle('Tandem Mix Cyclic Example', fontsize=16, y=0.98)



def collective_scaling():
    # Define inputs
    rpy_input = np.linspace(-1.0, 1.0, 20)
    collective = np.linspace(0.0, 1.0, 20)

    # Create meshgrid
    RPY, COL = np.meshgrid(rpy_input, collective)

    Z = np.zeros_like(RPY) # Swash1
    Z2 = np.zeros_like(RPY) # Swash2
    Z_Scaled = np.zeros_like(RPY)
    Z2_Scaled = np.zeros_like(RPY)

    # Apply output constraints for every combination
    for i in range(RPY.shape[0]):
        for j in range(COL.shape[1]):
            for k in range(RPY.shape[0]):
                for m in range(COL.shape[1]):

                    # Calculate the unscaled mixed value
                    Z[i,j] = RPY[i,j] + COL[i,j]
                    Z2[k,m] = RPY[k,m] + COL[k,m]

                    COL_HIGH = 1.0
                    COL_LOW = 0.0

                    scale = 1.0
                    if Z[i, j] > COL_HIGH:
                        # We only want to scale pitch in this case. We are giving priority to collective
                        delta = Z[i, j] - COL_HIGH
                        scale = min(scale, delta / RPY[i,j])
                        # This is where we would set the limit flag

                    if Z[i, j] < COL_LOW:
                        delta = Z[i, j] - COL_LOW
                        scale = min(scale, delta / RPY[i,j])
                        # This is where we would set the limit flag

                    if Z2[k,m] > COL_HIGH:
                        # We only want to scale pitch in this case. We are giving priority to collective
                        delta = Z2[k,m] - COL_HIGH
                        scale = min(scale, delta / RPY[k,m])
                        # This is where we would set the limit flag

                    if Z2[k,m] < COL_LOW:
                        delta = Z2[k,m] - COL_LOW
                        scale = min(scale, delta / RPY[k,m])
                        # This is where we would set the limit flag

                    Z_Scaled[i, j] = RPY[i,j]*scale + COL[i,j]
                    Z2_Scaled[i, j] = RPY[i,j]*scale + COL[i,j]


    # Create figure with 2 subplots side by side
    fig = plt.figure(figsize=(14, 6))  # Adjust size as needed

    z_ticks = [-2.0, -1.5, -1.0, -0.5, 0, 0.5, 1.0, 1.5, 2.0]

    # First 3D plot
    ax1 = fig.add_subplot(1, 2, 1, projection='3d')
    surf1 = ax1.plot_surface(RPY, COL, Z, cmap='viridis', edgecolor='none', vmin=min(z_ticks), vmax=max(z_ticks))
    surf1_scaled = ax1.plot_wireframe(RPY, COL, Z_Scaled, color='black', linewidth=0.5)
    ax1.set_title('Swash 1')
    ax1.set_xlabel('Pitch')
    ax1.set_ylabel('Collective')
    ax1.set_zlabel('Swash1 Roll Output')
    ax1.view_init(elev=21, azim=-61)
    fig.colorbar(surf1, ax=ax1, shrink=0.5, aspect=10)
    ax1.xaxis.set_major_locator(MaxNLocator(nbins=5))
    ax1.yaxis.set_major_locator(MaxNLocator(nbins=5))
    ax1.set_zticks(z_ticks)

    # Second 3D plot
    ax2 = fig.add_subplot(1, 2, 2, projection='3d')
    surf2 = ax2.plot_surface(RPY, COL, Z2, cmap='plasma', edgecolor='none', vmin=min(z_ticks), vmax=max(z_ticks))
    surf2_scaled = ax2.plot_wireframe(RPY, COL, Z2_Scaled, color='black', linewidth=0.5)
    ax2.set_title('Swash 2')
    ax2.set_xlabel('Pitch')
    ax2.set_ylabel('Collective')
    ax2.set_zlabel('Swash2 Roll Output')
    ax2.view_init(elev=21, azim=132)
    fig.colorbar(surf2, ax=ax2, shrink=0.5, aspect=10, ticks=z_ticks)
    ax2.xaxis.set_major_locator(MaxNLocator(nbins=5))
    ax2.yaxis.set_major_locator(MaxNLocator(nbins=5))
    ax2.set_zticks(z_ticks)

    plt.suptitle('Tandem Mix Collective Example', fontsize=16, y=0.98)


if __name__ == "__main__":
    # cyclic_scaling()
    collective_scaling()

# Show the figure
plt.tight_layout()
plt.show()
