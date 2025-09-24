import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

# m(t) ̄a(t) =  ̄F + m′(t) ̄u(t)
# ̄a(t) =  ̄(F + m′(t) ̄u(t))/m(t)

# ̄v'(t) =  ̄(F + m′(t) ̄u(t))/m(t)
# ̄v'(t) =  (m(t) ̄g − c|| ̄v(t)|| ̄v(t) + m′(t) ̄u(t))/m(t)
# v(0) = 0

h = 0.01
t0 = 0
t1 = 15
t_span = (t0, t1)
tt = np.arange(t0, t1, h)

# x,y,vx,vy
v0 = np.array([0,0,0,0])
c = 0.05 # kg/m
k_m = 700 # m/s
goal_cords = np.array([80, 60]) # mål koordinator
t = 0
t_max = 10
g = np.array([0, -9.82]) # Vektor gravitation

# massfunktion
def m(t):
    if t < 10:
        return 8 - 0.4 * t
    else:
        return 4
    
def m_prim(t):
    if t < 10:
        return -0.4
    else:
        return 0
    
# Det vi kommer styra med
def angle(t, pos):
    if pos[1] >= 20:
        dx = pos[0] - goal_cords[0]
        dy = goal_cords[1] - pos[1]
        return -np.arctan2(dy, dx)
    else:
        return -np.pi / 2  
     

# Funktioner för hastighetsvektorn
def u_x(t,ang):
    return k_m * np.cos(ang)

def u_y(t,ang):
    return k_m * np.sin(ang)

#hastighetsvektor för bränslet
def u(t,pos):
    ang = angle(t,pos)
    return np.array([u_x(t,ang), u_y(t,ang)])

# Alla externa krafter som verkar på raketen
def F_vector(t, v):
    return m(t) * g - c * np.linalg.norm(v) * v
# f(t, v) = (F + m′(t) ̄u(t))/m(t)

# ̄v'(t) =  (F + m′(t) * u(t))/m(t)
# v(0) = 0
def ode_rhs(t, v):
    #return (m(t) ̄g − c|| ̄v(t)|| ̄v(t) + m′(t) ̄u(t))/m(t)
    x, y, vx, vy = v
    pos = np.array([x, y])
    v_tot = np.array([vx, vy])
    a = (F_vector(t, v_tot) + m_prim(t) * u(t,pos))/m(t)
    return np.array([vx,vy, a[0], a[1]])

sol = solve_ivp(ode_rhs, t_span, v0, t_eval = tt)

#y_pos = np.zeros(int(t1 / h))
#x_pos = np.zeros(int(t1 / h))
#for i in range(len(tt) - 1):
#    x_pos[i + 1] = x_pos[i] + sol.y[2][i] * 0.1
#    y_pos[i + 1] = y_pos[i] + sol.y[3][i] * 0.1

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