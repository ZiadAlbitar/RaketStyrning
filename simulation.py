import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from common import ode_rhs, goal_cords

h = 0.01
t0 = 0
t1 = 15
t_span = (t0, t1)
tt = np.arange(t0, t1, h)
# x,y,vx,vy
v0 = np.array([0,0,0,0])

sol = solve_ivp(ode_rhs, t_span, v0, t_eval = tt)

plt.figure(figsize=(10,5))

plt.subplot(1,2,1)
plt.plot(sol.y[0], sol.y[1], label='Trajectory')
plt.scatter(goal_cords[0],goal_cords[1], color='red', label='Goal')
plt.xlabel('x')
plt.ylabel('y')
plt.title('Raket')
plt.legend()
plt.grid()

plt.subplot(1,2,2)
plt.plot(sol.t, sol.y[2], label='v_x')
plt.plot(sol.t, sol.y[3], label='v_y')
plt.xlabel('Tid')
plt.ylabel('Hastighet')
plt.title('Raket hastigheter')
plt.legend()
plt.grid()

plt.tight_layout()
plt.show()