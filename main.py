import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp


# m(t) ̄a(t) =  ̄F + m′(t) ̄u(t)
# ̄a(t) =  ̄(F + m′(t) ̄u(t))/m(t)

# ̄v'(t) =  ̄(F + m′(t) ̄u(t))/m(t)
# ̄v'(t) =  (m(t) ̄g − c|| ̄v(t)|| ̄v(t) + m′(t) ̄u(t))/m(t)
# v(0) = 0


h = 0.1
t_span = (0,15)
tt = np.arange(0, 15, h)
v0 = np.array([0,0])
c = 0.05 # kg/m
k_m = 700 # m/s
position_0 = np.array([0, 0]) # x och y
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
def angle(t):
    return np.pi / 2
     

# Funktioner för hastighetsvektorn
def u_x(t):
    return k_m * np.cos(angle(t))

def u_y(t):
    return k_m * np.sin(angle(t))


#hastighetsvektor för bränslet
def u(t):
    return np.array([u_x(t), u_y(t)])

# Alla externa krafter som verkar på raketen
def F_vector(t, v):
    # v1, v2 = v * np.absolute(v)
    # test = m(t)*g - c * np.absolute(v) * v
    # return np.array([test[0], test[1]])
    return m(t)*g - c *np.multiply(np.absolute(v), v)
    #return m(t) * g - (c * np.absolute() * v)
# f(t, v) = (F + m′(t) ̄u(t))/m(t)


# ̄v'(t) =  (F + m′(t) * u(t))/m(t)
# v(0) = 0
def ode_rhs(t, v):
    #return (m(t) ̄g − c|| ̄v(t)|| ̄v(t) + m′(t) ̄u(t))/m(t)
    F1, F2 = F_vector(t,v)

    u1, u2 = u(t)
 
    return np.array([((F1 + m_prim(t)) * u1)/ m(t), (F2 + m_prim(t) * u2) / m(t)])
    #return (F_vector(t) + m_prim(t) * u(t)) / m(t)


sol = solve_ivp(ode_rhs, t_span, v0, t_eval = tt)

plt.plot(sol.t, sol.y[0], label='x')
plt.plot(sol.t, sol.y[1], label='y')
plt.title('Uppgift 2')
plt.xlabel('tid, t')
plt.ylabel('v')
plt.legend()
plt.show()
