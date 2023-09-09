import matplotlib.pyplot as plt
import pandas as pd
from scipy.spatial.transform import Rotation as R
import numpy as np


df = pd.read_csv("/home/lachl/measurements/data.csv")
print(df.head())
df["roll"] = np.zeros(len(df))
df["pitch"] = np.zeros(len(df))
df["yaw"] = np.zeros(len(df))

# convert quaternions to euler angles
for i in range(len(df)):
    r = R.from_quat([df["qx"][i], df["qy"][i], df["qz"][i], df["qw"][i]])
    roll, pitch, yaw = r.as_euler("xyz", degrees=True)
    df["roll"][i] = roll
    df["pitch"][i] = pitch
    df["yaw"][i] = yaw


for k in df.keys():
    df[k] = df[k] - df[k].mean()
    print("range", k, "=", max(df[k]) - min(df[k]))

# plot histograms of x,y,z and qx,qy,qz,qw
fig, axs = plt.subplots(2, 3)
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

print(df.describe())


plt.show()
