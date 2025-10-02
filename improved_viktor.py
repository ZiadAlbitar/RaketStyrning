import numpy as np

c = 0.05 # kg/m
k_m = 700 # m/s
goal_cords = np.array([60, 80])
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
    # Direction toward goal
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

        
# Funktioner för hastighetsvektorn
def u_x(ang):
    return k_m * np.cos(ang)

def u_y(ang):
    return k_m * np.sin(ang)

# Hastighetsvektor för bränslet
def u(pos, v):
    ang = angle(pos, v)
    return np.array([u_x(ang), u_y(ang)])

# Alla externa krafter som verkar på raketen
def F_vector(t, v):
    return m(t) * g - c * np.linalg.norm(v) * v

def ode_rhs(t, v):
    x, y, vx, vy = v
    pos = np.array([x, y])
    v_tot = np.array([vx, vy])
    a = (F_vector(t, v_tot) + m_prim(t) * u(pos, v_tot))/m(t)
    # Returnera en array med hastigheten och accelration i x- och y-led
    return np.array([vx, vy, a[0], a[1]])