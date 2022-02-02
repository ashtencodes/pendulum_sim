import numpy as np
from scipy import integrate
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt

# Input constants
m = 1 # mass (kg)
L = 1 # length (m)
b = 0.5# damping value (kg/m^2-s)
g = 9.81 # gravity (m/s^2)
delta_t = 0.02 # time step size (seconds)
t_max = 10 # max sim time (seconds)
theta1_0 = np.pi/2 # initial angle (radians)
theta2_0 = 0 # initial angular velocity (rad/s)
theta_init = (theta1_0, theta2_0)
# Get timesteps
t = np.linspace(0, t_max, 500)

def int_pendulum_sim(theta_init, t, L=1, m=1, b=0.5, g=9.81):
    theta_dot_1 = theta_init[1]
    theta_dot_2 = -b/m*theta_init[1] - g/L*np.sin(theta_init[0])
    return theta_dot_1, theta_dot_2

theta_vals_int = integrate.odeint(int_pendulum_sim, theta_init, t)

energyArray = []
for interval in theta_vals_int:
    totEnergy = m*g*L*(1-np.cos(interval[0])) + 0.5*m*(interval[1])**2
    energyArray.append(totEnergy)

energyArray = np.array(energyArray)

plt.title("Angular Position & Velocity Vs Time")
line1, = plt.plot(t, theta_vals_int[:,0], 'b-', label='Angular Position')
line2, = plt.plot(t, theta_vals_int[:,1], 'r--', label='Angular Velocity')
plt.xlabel("Time")
plt.legend(handles=[line1, line2])
plt.show()

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot3D(theta_vals_int[:,0], theta_vals_int[:,1], energyArray)
ax.set_xlabel('Position')
ax.set_ylabel('Velocity')
ax.set_zlabel('Energy')
plt.show()

plt.plot(theta_vals_int[:,0], theta_vals_int[:,1])
plt.xlabel("Angular Position")
plt.ylabel("Angular Velocity")
plt.show()