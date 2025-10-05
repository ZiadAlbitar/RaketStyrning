import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from common import goal_cords, F_vector, m, m_prim, u_x, u_y
import time

# Mät körtiden
start_time = time.time()
c = 0.05 # kg/m
k_m = 700 # m/s
g = np.array([0, -9.82]) # Vektor gravitation

# x,y,vx,vy
v0 = np.array([0,0,0,0])

h = 0.01
t0 = 0
t1 = 10
t_span = (t0, t1)
tt = np.arange(t0, t1, h)

#konstanter för alla delar i PID
Kp = 1.0
Ki = 0.1
Kd = 0.05

prev_error = np.array([0.0,0.0])
#förra error vi får av att integrera
integral_error = np.array([0.0,0.0])

def angle(pos, v):
    global prev_error, integral_error

    dt = 0.01
    #Riktning mot målet
    dx = goal_cords[0] - pos[0]
    dy = goal_cords[1] - pos[1]
    theta = np.arctan2(dy, dx)
    desired_v = np.array([np.cos(theta), np.sin(theta)])

    if(np.linalg.norm(v) < 1.e-6):
        current_v = v
    else:
        current_v = v / np.linalg.norm(v)

    # Error vector
    error = desired_v - current_v

    # PID termer
    P_out = Kp * error
    integral_error += error * dt
    I_out = Ki * integral_error
    D_out = Kd * (error - prev_error) / dt

    prev_error = error

    out = P_out + I_out + D_out
    if pos[1] >= 20:
        return np.arctan2(out[1], out[0])+np.pi

    else:
        return -np.pi / 2  

        
# Denna funktion behövde omdefineras för att använda överskridna angle(),
# finns snyggare sätt att hantera detta men blev enklast så.
def u(pos, v):
    ang = angle(pos, v)
    return np.array([u_x(ang), u_y(ang)])

def ode_rhs(t, v):
    x, y, vx, vy = v
    pos = np.array([x, y])
    v_tot = np.array([vx, vy])
    a = (F_vector(t, v_tot) + m_prim(t) * u(pos, v_tot))/m(t)
    # Returnera en array med hastigheten och accelration i x- och y-led
    return np.array([vx, vy, a[0], a[1]])

sol = solve_ivp(ode_rhs, t_span, v0, t_eval = tt)
run_time = str(time.time() - start_time)
print("Run time: " + run_time)

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