import matplotlib.pyplot as plt
import pandas as pd
from scipy.spatial.transform import Rotation as R
import numpy as np
from tf import transformations


df = pd.read_csv("/home/lachl/measurements/multi_aruco_tf_euler.csv")
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


for k in df.keys():
    df[k] = df[k] - df[k].mean()
    print("range", k, "=", max(df[k]) - min(df[k]))

# plot histograms of x,y,z and r,p,y and qx,qy,qz,qw
fig, axs = plt.subplots(3, 4)
axs[0, 0].hist(df["x"], bins=100)
axs[0, 0].set_title("x")
axs[0, 1].hist(df["y"], bins=100)
axs[0, 1].set_title("y")
axs[0, 2].hist(df["z"], bins=100)
axs[0, 2].set_title("z")
axs[1, 0].hist(df["roll"], bins=100)
axs[1, 0].set_title("roll")
axs[1, 1].hist(df["pitch"], bins=100)
axs[1, 1].set_title("pitch")
axs[1, 2].hist(df["yaw"], bins=100)
axs[1, 2].set_title("yaw")
# axs[2, 0].hist(df["qx"], bins=100)
# axs[2, 0].set_title("qx")
# axs[2, 1].hist(df["qy"], bins=100)
# axs[2, 1].set_title("qy")
# axs[2, 2].hist(df["qz"], bins=100)
# axs[2, 2].set_title("qz")
# axs[2, 3].hist(df["qw"], bins=100)
# axs[2, 3].set_title("qw")

print(df.describe())


plt.show()
