# %%
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D


# %%

df = np.loadtxt(open("results.txt").readlines()[:-1], delimiter="\t", skiprows=3)
s = df[:, 1:]
t = df[:, 0]

# %%
fig, ax = plt.subplots(1, 1, sharex=True, figsize=(16, 9))

for i in range(s.shape[1]):
    ax.plot(t, s[:, i])
# ax.legend(["$\phi$", "$\\theta$", "$\psi$"])
# ax.set_ylabel("degrees")
plt.show()
