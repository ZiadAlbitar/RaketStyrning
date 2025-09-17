import numpy as np

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
    
# Det vi kommer styra med
def angle(t):
    return np.pi / 2

# Funktioner för hastighetsvektorn
def u_x(t):
    return k_m * np.cos(angle(t))

def u_y(t):
    return k_m * np.sin(angle(t))

def u(t):
    return np.array([u_x(t), u_y(t)])

def v(t):
    return 

def F_vector(t):
    return m(t) * g - (c * np.absolute(v(t)) * v(t))