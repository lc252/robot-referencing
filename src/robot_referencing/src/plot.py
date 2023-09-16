import matplotlib.pyplot as plt
import pandas as pd
from scipy.spatial.transform import Rotation as R
import numpy as np
from tf import transformations


df = pd.read_csv("/home/lachl/measurements/icp.csv")


print(df.head())
df["roll"] = np.zeros(len(df))
df["pitch"] = np.zeros(len(df))
df["yaw"] = np.zeros(len(df))

# convert quaternions to euler angles
for i in range(len(df)):
    # r = R.from_quat([df["qx"][i], df["qy"][i], df["qz"][i], df["qw"][i]])
    # roll, pitch, yaw = r.as_euler("xyz", degrees=True)
    roll, pitch, yaw = transformations.euler_from_quaternion([df["qx"][i], df["qy"][i], df["qz"][i], df["qw"][i]], axes="sxyz")
    df["roll"][i] = roll
    df["pitch"][i] = pitch
    df["yaw"][i] = yaw

# convert xyz to mm
df["x"] = df["x"] * 1000
df["y"] = df["y"] * 1000
df["z"] = df["z"] * 1000

# convert rpy to degrees
df["roll"] = df["roll"] * (180 / np.pi)
df["pitch"] = df["pitch"] * (180 / np.pi)
df["yaw"] = df["yaw"] * (180 / np.pi)

# remove values outside of 3 standard deviations
df = df[(np.abs(df["x"] - df["x"].mean()) <= (3 * df["x"].std()))]
df = df[(np.abs(df["y"] - df["y"].mean()) <= (3 * df["y"].std()))]
df = df[(np.abs(df["z"] - df["z"].mean()) <= (3 * df["z"].std()))]
df = df[(np.abs(df["roll"] - df["roll"].mean()) <= (3 * df["roll"].std()))]
df = df[(np.abs(df["pitch"] - df["pitch"].mean()) <= (3 * df["pitch"].std()))]
df = df[(np.abs(df["yaw"] - df["yaw"].mean()) <= (3 * df["yaw"].std()))]

# shift mean to zero and print range
for k in df.keys():
    df[k] = df[k] - df[k].mean()
    print("range", k, "=", max(df[k]) - min(df[k]))

# plot histograms of x,y,z and r,p,y and qx,qy,qz,qw
# fig, axs = plt.subplots(3, 4)
fig, axs = plt.subplots(2, 3)
# plot xyz axes in blue
axs[0, 0].hist(df["x"], bins=50, color="tab:blue")
axs[0, 0].set_title("X Distribution")
axs[0, 0].set_xlabel("X Error (mm)")
axs[0, 1].hist(df["y"], bins=50, color="tab:blue")
axs[0, 1].set_title("Y Distribution")
axs[0, 1].set_xlabel("Y Error (mm)")
axs[0, 2].hist(df["z"], bins=50, color="tab:blue")
axs[0, 2].set_title("Z Distribution")
axs[0, 2].set_xlabel("Z Error (mm)")

# plot rpy axes in orange
axs[1, 0].hist(df["roll"], bins=50, color="tab:orange")
axs[1, 0].set_title("Roll Distribution")
axs[1, 0].set_xlabel("Roll Error (deg)")
axs[1, 1].hist(df["pitch"], bins=50, color="tab:orange")
axs[1, 1].set_title("Pitch Distribution")
axs[1, 1].set_xlabel("Pitch Error (deg)")
axs[1, 2].hist(df["yaw"], bins=50, color="tab:orange")
axs[1, 2].set_title("Yaw Distribution")
axs[1, 2].set_xlabel("Yaw Error (deg)")

# set padding
plt.tight_layout(pad=1.0)

# plot quaternions in green
# axs[2, 0].hist(df["qx"], bins=50, color="tab:green")
# axs[2, 0].set_title("qx")
# axs[2, 1].hist(df["qy"], bins=50, color="tab:green")
# axs[2, 1].set_title("qy")
# axs[2, 2].hist(df["qz"], bins=50, color="tab:green")
# axs[2, 2].set_title("qz")
# axs[2, 3].hist(df["qw"], bins=50, color="tab:green")
# axs[2, 3].set_title("qw")

print(df.describe())


plt.show()
