import requests
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# === URLs ===
#AP_t = "http://192.168.4.1/get_depth"
AP_URL = "http://192.168.4.1/get_xyz_ap"
CLIENT_URL = "http://192.168.4.2/get_xyz_client"

def filter_close_points(X, Y, Z, min_dist=5.0):
    points = np.vstack((X, Y, Z)).T
    keep_indices = []
    
    for i, p in enumerate(points):
        too_close = False
        for idx in keep_indices:
            if np.linalg.norm(p - points[idx]) < min_dist:
                too_close = True
                break
        if not too_close:
            keep_indices.append(i)
    
    return X[keep_indices], Y[keep_indices], Z[keep_indices]


#ap_datat = requests.get(AP_t).json()
# === Get data from AP ===
ap_data = requests.get(AP_URL).json()
X_ap = np.array(ap_data["X"])
Y_ap = np.array(ap_data["Y"])
Z_ap = np.array(ap_data["Z"], dtype=float)

# === Get data from Client ===
client_data = requests.get(CLIENT_URL).json()
X_client = np.array(client_data["X"])
Y_client = np.array(client_data["Y"])
Z_client = np.array(client_data["Z"], dtype=float)

# === Combine both halves ===
X = np.concatenate((X_ap, X_client))
Y = np.concatenate((Y_ap, Y_client))
Z = np.concatenate((Z_ap, Z_client))

# Remove invalid points (Z > 0)
valid = Z > 0
X, Y, Z = X[valid], Y[valid], Z[valid]

# Remove points too close to each other
X, Y, Z = filter_close_points(X, Y, Z, min_dist=5.0)

# === Save to CSV ===
df = pd.DataFrame({'X': X, 'Y': Y, 'Z': Z})
df.to_csv("depth_data_full.csv", index=False)
print("Saved combined depth data to depth_data_full.csv")

# === Plot ===
fig = plt.figure(figsize=(10, 7))
ax = fig.add_subplot(111, projection='3d')

sc = ax.scatter(X, Y, Z, c=Z, cmap='viridis', s=5)

ax.set_xlim(0, 255)
ax.set_ylim(0, 255)
ax.set_zlim(0, 255)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z (Depth)')

plt.colorbar(sc, ax=ax, shrink=0.6, label="Depth (Z value)")
plt.title("ESP32-CAM Combined Depth Map")

plt.show()
