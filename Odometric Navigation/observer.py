from math import cos,sin

WHEEL_RADIUS = 0.0
WHEEL_AXIS   = 0.0
X_GLOBAL_POS = 0.0
Y_GLOBAL_POS = 0.0
TH_GLOBAL_POS = 0.0

WL0 = 0.0
WR0 = 0.0

def init_pose (robot_pars):
    """
    Initialize the observer, using the given robot's parameters
    Return True if the inizialization was successful
    """
    global WHEEL_RADIUS, WHEEL_AXIS, X_GLOBAL_POS, Y_GLOBAL_POS, TH_GLOBAL_POS, WL0, WR0

    WHEEL_RADIUS = robot_pars['wheel_radius']
    WHEEL_AXIS = robot_pars['wheel_axis']
    X_GLOBAL_POS = robot_pars['x_init_pos']
    Y_GLOBAL_POS = robot_pars['y_init_pos']
    TH_GLOBAL_POS = robot_pars['th_init_pos']

    WL0 = robot_pars['wl_init_pos']
    WR0 = robot_pars['wr_init_pos']

    return True

def update_pose (wl, wr):
    """
    Incremental position estimation: update the current pose (x, y, th) of the robot
    taking into account the newly read positions (rotations) of the left and right wheels
    Returns the new robot's pose, as a triple (x, y, th)
    """
    global WHEEL_RADIUS, WHEEL_AXIS, X_GLOBAL_POS, Y_GLOBAL_POS, TH_GLOBAL_POS, WL0, WR0

    # Convert wheel rotations to linear displacements
    d_left = (wl - WL0) * WHEEL_RADIUS
    d_right = (wr - WR0) * WHEEL_RADIUS

    WL0 = wl
    WR0 = wr

    # Calculate linear displacement and angular displacement w.r.t. the robot reference system
    d_linear = (d_right + d_left) / 2.0
    d_angular = (d_right - d_left) / WHEEL_AXIS

    # Calculate linera displacement w.r.t. the world reference system
    dx = d_linear * cos(d_angular/2) 
    dy = d_linear * sin(d_angular/2)

    # Update robot's pose based on odometry
    x = (dx * cos(TH_GLOBAL_POS)) - (dy * sin(TH_GLOBAL_POS))
    y = (dx * sin(TH_GLOBAL_POS)) + (dy * cos(TH_GLOBAL_POS))
    th = d_angular

    X_GLOBAL_POS = X_GLOBAL_POS + x
    Y_GLOBAL_POS = Y_GLOBAL_POS + y
    TH_GLOBAL_POS = TH_GLOBAL_POS + th

    return (X_GLOBAL_POS, Y_GLOBAL_POS, TH_GLOBAL_POS)
