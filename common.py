import numpy as np

c = 0.05 # kg/m
k_m = 700 # m/s
goal_cords = np.array([80, 60])
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
def angle(pos):
    if pos[1] >= 20:
        dx = pos[0] - goal_cords[0]
        dy = goal_cords[1] - pos[1]
        return -np.arctan2(dy, dx)
    else:
        return -np.pi / 2  
        
# Funktioner för hastighetsvektorn
def u_x(t, ang):
    return k_m * np.cos(ang)

def u_y(t, ang):
    return k_m * np.sin(ang)

# Hastighetsvektor för bränslet
def u(t, pos):
    ang = angle(t, pos)
    return np.array([u_x(t, ang), u_y(t, ang)])

# Alla externa krafter som verkar på raketen
def F_vector(t, v):
    return m(t) * g - c * np.linalg.norm(v) * v

def ode_rhs(t, v):
    x, y, vx, vy = v
    pos = np.array([x, y])
    v_tot = np.array([vx, vy])
    a = (F_vector(t, v_tot) + m_prim(t) * u(t,pos))/m(t)
    # Returnera en array med hastigheten och accelaration i x- och y-led
    return np.array([vx, vy, a[0], a[1]])