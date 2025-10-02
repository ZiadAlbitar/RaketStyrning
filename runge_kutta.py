import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import fsolve
from common import ode_rhs, goal_cords

h = 0.1
t0 = 0
t1 = 15
t_span = (t0, t1)
tt = np.arange(t0, t1, h)

# x,y,vx,vy
v0 = np.array([0,0,0,0])

def g(t_n, v_n, h):
    t_next = t_n + h
    k1 = ode_rhs(t_n, v_n)
    k2 = ode_rhs(t_n + h/2, v_n + h/2 * k1)
    k3 = ode_rhs(t_n + h/2, v_n + h/2 * k2)
    k4 = ode_rhs(t_next, v_n + h * k3)
    return v_n + h/6 * (k1 + 2*k2 + 2*k3 + k4)
    
y = np.zeros((len(tt), len(v0)))
y[0, :] = v0

for i in range(len(tt)-1):
    y[i + 1] = g(tt[i], y[i], h)

plt.plot(y[: , 0], y[: , 1], label="Trajectory")
plt.scatter(goal_cords[0], goal_cords[1], color='red', label='Goal')
plt.xlabel("x")
plt.ylabel('y')
plt.title('Raket')
plt.legend()
plt.grid()
plt.show()