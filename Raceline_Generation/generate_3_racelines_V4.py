from __future__ import print_function
import numpy as np
import os

# ================= CONFIG =================
MAP_NAME = "FTMHalle_ws25"
input_csv = "inputs/tracks/{}.csv".format(MAP_NAME)
output_dir = "inputs/tracks/racelines"

if not os.path.exists(output_dir):
    os.makedirs(output_dir)

alpha = np.array([0.75, 0.5, 0.25])  # racelines left -> right
CONST_SPEED = 3.0                   # Pure Pursuit speed

# ================= LOAD TRACK =================
# columns: x, y, w_right, w_left
track = np.loadtxt(input_csv, delimiter=",", skiprows=1)

x = track[:, 0]
y = track[:, 1]
w_right = track[:, 2]
w_left = track[:, 3]
track_width = w_left + w_right

# ================= HELPERS =================
def smooth_periodic(arr, window_len=21):
    """Moving average smoothing for closed curves"""
    pad = window_len // 2
    arr_ext = np.concatenate((arr[-pad:], arr, arr[:pad]))
    kernel = np.ones(window_len) / window_len
    smoothed = np.convolve(arr_ext, kernel, mode="same")
    return smoothed[pad:-pad]

# ================= CENTERLINE NORMAL =================
dx = np.gradient(x)
dy = np.gradient(y)
norm = np.sqrt(dx**2 + dy**2)

qx = -dy / norm
qy = dx / norm

# ================= SAVE FUNCTION =================
def save_line(name, x, y):
    """Save Pure-Pursuit compatible CSV: x, y, v"""
    # close loop
    x = np.append(x, x[0])
    y = np.append(y, y[0])

    # constant velocity column
    v = np.full_like(x, CONST_SPEED)

    out = np.column_stack((x, y, v))
    np.savetxt(
        os.path.join(output_dir, name),
        out,
        delimiter=",",
        fmt="%.4f"
    )

# ================= LEFT / RIGHT BOUNDARIES =================
shift = track_width * 0.5
lx = smooth_periodic(x + qx * shift, 31)
ly = smooth_periodic(y + qy * shift, 31)
save_line("track_left.csv", lx, ly)
print("track_left.csv saved")

shift = -track_width * 0.5
rx = smooth_periodic(x + qx * shift, 31)
ry = smooth_periodic(y + qy * shift, 31)
save_line("track_right.csv", rx, ry)
print("track_right.csv saved")

# ================= RACELINES =================
for i, a in enumerate(alpha):
    shift = track_width * (a - 0.5)
    rx = x + qx * shift
    ry = y + qy * shift

    rx = smooth_periodic(rx, 15)
    ry = smooth_periodic(ry, 15)

    save_line(f"raceline_{i + 1}.csv", rx, ry)
    print(f"raceline_{i + 1}.csv saved")

print("All racelines and boundaries generated — PURE PURSUIT SAFE.")
