import numpy as np
import matplotlib.pyplot as plt

# Assuming CSV has columns x, y
data = np.genfromtxt('world_points.csv', delimiter=',', skip_header=1)  # skip header
x = data[:, 0]
y = data[:, 1]

mask = ~np.isnan(x) & ~np.isnan(y)
x_clean = x[mask]
y_clean = y[mask]

plt.figure(figsize=(8, 8))
plt.scatter(x_clean, y_clean, s=1)  # s=1 for small points
plt.xlabel('X [m]')
plt.ylabel('Y [m]')
plt.title('Fused LiDAR + IMU points')
plt.axis('equal')  # keep aspect ratio
plt.show()

