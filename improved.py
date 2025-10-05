import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from common import u_x, u_y, m_prim, F_vector, m, goal_cords
import time

# Mät körtiden
start_time = time.time()
h = 0.01
t0 = 0
t1 = 10
t_span = (t0, t1)
tt = np.arange(t0, t1, h)

# x, y, vx, vy
v0 = np.array([0,0,0,0])
    
# Det vi kommer styra med
def angle(pos, ang):
    if pos[1] >= 20:
        return ang
    else:
        return -np.pi / 2  

# Den funktionen behövde omdefineras för att använda överskridna angle(),
# finns snyggare sätt att hantera detta men blev enklast så.
def u(pos, ang):
    ang = angle(pos, ang)
    return np.array([u_x(ang), u_y(ang)])


def ode_rhs(t, v, ang):
    x, y, vx, vy = v
    pos = np.array([x, y])
    distance = np.sqrt((goal_cords[0] - pos[0])**2 + (goal_cords[1] - pos[1])**2)
    global closest_dist
    global best_angle
    if (distance < closest_dist):
        closest_dist = distance
        best_angle = ang
    v_tot = np.array([vx, vy])
    a = (F_vector(t, v_tot) + m_prim(t) * u(pos, ang))/m(t)
    return np.array([vx, vy, a[0], a[1]])

# Intiala värden
angle0 = -np.pi / 2
angle1 = - np.pi
best_angle = -np.pi / 2
closest_dist = np.sqrt((goal_cords[0]**2) + goal_cords[1]**2)
for ang in np.linspace(angle0, angle1, 1800):
    solve_ivp(ode_rhs, t_span, v0, args=(ang,), t_eval = tt)

sol = solve_ivp(ode_rhs, t_span, v0, args=(best_angle,), t_eval = tt)
run_time = str(time.time() - start_time)
print("Run time: " + run_time)

plt.figure(figsize=(10,5))
plt.subplot(1,2,1)
plt.plot(sol.y[0], sol.y[1], label='Trajectory')
plt.scatter(goal_cords[0],goal_cords[1], color='red', label='Goal')
plt.xlabel('x')
plt.ylabel('y')
plt.title('Angel= ' + str(best_angle))
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
