# %%
import numpy as np
import matplotlib.pyplot as plt

# %%


def ffunc(t):
    _p = np.pi / 6 + 0 * t
    _q = np.cos(t * 6 / np.pi)
    _r = 3 * np.sin(t * 30 / np.pi)
    return _p, _q, _r


def dfunc(x, dcm, v_body, t):
    # For readability, pull out Euler angles
    _phi = x[0]
    _theta = x[1]
    _psi = x[2]

    # In this case, p,q & r are given by an analytic formula
    _p, _q, _r = ffunc(t)

    # Gimbal equation
    _qdot = np.array(
        [
            [1, np.tan(_theta) * np.sin(_phi), np.tan(_theta) * np.cos(_phi)],
            [0, np.cos(_phi), -np.sin(_phi)],
            [0, np.sin(_phi) / np.cos(_theta), np.cos(_phi) / np.cos(_theta)],
        ]
    ).dot(np.array([_p, _q, _r]))

    # Strapdown Equation
    _Cdot = dcm.dot(np.array([[0, -_r, _q], [_r, 0, -_p], [-_q, _p, 0]]))

    # Get Velocity in NED
    # this is position_dot for position measured in NED
    _V_NED = dcm.dot(v_body)

    # Assemble the vector of state derivatives -- putting in zeros for the
    # derivative of velocity since it is a constant here
    _xdot = np.hstack((_qdot, [0, 0, 0], _V_NED))

    return _xdot, _Cdot


# %%
dt = 0.01
x0 = [0, 0, 0]  # radians
V_body = [60 * 6076 / 3600, 0, 0]  # in ft/sec

x = [np.hstack((x0, V_body, [0, 0, 0]))]
xd = []
t = [0]
tmax = 20
func_calls = 0

# For readability, pull out Euler angles
phi = x[0][0]
theta = x[0][1]
psi = x[0][2]

DCM = np.eye(3)
DCM = np.array(
    [[1, 0, 0], [0, np.cos(phi), -np.sin(phi)], [0, np.sin(phi), np.cos(phi)]]
).dot(DCM)

DCM = np.array(
    [[np.cos(theta), 0, np.sin(theta)], [0, 1, 0], [-np.sin(theta), 0, np.cos(theta)]]
).dot(DCM)

DCM = np.array(
    [[np.cos(psi), -np.sin(psi), 0], [np.sin(psi), np.cos(psi), 0], [0, 0, 1]]
).dot(DCM)


for idx in range(int(tmax / dt)):
    time = idx * dt
    xn = np.array(x[idx])

    xdot1, Cdot1 = dfunc(xn, DCM, V_body, time)
    xdot2, Cdot2 = dfunc(
        xn + xdot1 * dt / 2, DCM + Cdot1 * dt / 2, V_body, time + dt / 2
    )
    xdot3, Cdot3 = dfunc(
        xn + xdot2 * dt / 2, DCM + Cdot2 * dt / 2, V_body, time + dt / 2
    )
    xdot4, Cdot4 = dfunc(xn + xdot3 * dt, DCM + Cdot3 * dt, V_body, time + dt)

    xdot = (xdot1 + 2 * xdot2 + 2 * xdot3 + xdot4) / 6
    Cdot = (Cdot1 + 2 * Cdot2 + 2 * Cdot3 + Cdot4) / 6

    DCM = DCM + Cdot * dt
    x.append(xn + xdot * dt)
    xd.append(xdot)
    t.append(time + dt)
    func_calls += 4
x = np.array(x)
xd = np.array(xd)
t = np.array(t)
print(func_calls)


# %%

fig, axes = plt.subplots(5, 1, sharex=True, figsize=(16, 9))

RAD2DEG = 180 / np.pi

ps, qs, rs = ffunc(t)
axes[0].plot(t, RAD2DEG * ps, t, RAD2DEG * qs, t, RAD2DEG * rs)
axes[0].legend(["$p$", "$q$", "$r$"])
axes[0].set_ylabel("degrees/sec")

axes[1].plot(
    t[1:], RAD2DEG * xd[:, 0], t[1:], RAD2DEG * xd[:, 1], t[1:], RAD2DEG * xd[:, 2]
)
axes[1].legend(["$\dot{\phi}$", "$\dot{\\theta}$", "$\dot{\psi}$"])
axes[1].set_ylabel("degrees/sec")

phis = (x[:, 0] + np.pi) % (2 * np.pi) - np.pi
thetas = (x[:, 1] + np.pi) % (2 * np.pi) - np.pi
psis = (x[:, 2] + np.pi) % (2 * np.pi) - np.pi
axes[2].plot(t, RAD2DEG * phis, t, RAD2DEG * thetas, t, RAD2DEG * psis)
axes[2].legend(["$\phi$", "$\\theta$", "$\psi$"])
axes[2].set_ylabel("degrees")

vt = np.sqrt(np.sum(xd[:, 6:] ** 2, axis=1))
axes[3].plot(t[1:], xd[:, 6], t[1:], xd[:, 7], t[1:], xd[:, 8], t[1:], vt)
axes[3].legend(["$V_N$", "$V_E$", "$V_D$", "$V$"])
axes[3].set_ylabel("velocity (fps)")

pt = np.sqrt(np.sum(x[:, 6:] ** 2, axis=1))
axes[4].plot(t, x[:, 6], t, x[:, 7], t, x[:, 8], t, pt)
axes[4].legend(["$P_N$", "$P_E$", "$P_D$", "$P$"])
axes[4].set_ylabel("position (ft)")

plt.xlabel("time (sec)")
plt.tight_layout()
# plt.savefig('runge-kutta.png', format='png', dpi=1200)
plt.show()

# %%
fig, ax = plt.subplots(1, 1, sharex=True, figsize=(16, 9))

ax.plot(t, RAD2DEG * phis, t, RAD2DEG * thetas, t, RAD2DEG * psis)
ax.legend(["$\phi$", "$\\theta$", "$\psi$"])
ax.set_ylabel("degrees")
plt.show()
# %%
# plt.switch_backend('TkAgg')
fig = plt.figure(figsize=(16, 9))
ax = fig.add_subplot(111, projection="3d")
ax.plot(x[:, 6], x[:, 7], x[:, 8])

plt.show()
# get_ipython().run_line_magic('matplotlib', 'inline')


# %%
