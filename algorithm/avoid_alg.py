"""
Function to return a control signal for RC car A to slow to a desired velocity
required to avoid collision with RC car B.
"""
import math
# import PID
# import time
# import numpy
# import matplotlib.pyplot

def correct_speed(A_pos, A_poscol, B_poscol, A_angcol, B_angcol, B_velcol, t_col):
    """
    Finds back edge of car to be avoided (car B) and front edge of automated
    car (car A) that will collide in frame A. Finds minimum space car B needs
    to move forward to avoid collision with car A's front. Reduces car A's
    speed accordingly.
    """
    
    # Car dimensions
    w = 12.65 # Width in cm
    l = 17.50 # Length in cm
    
    # Find direction of approach towards B and select back corner to calibrate
    # with
    B_angNpos = (B_angcol - A_angcol) % (2 * math.pi) # Positive B angle in A frame
    B_angNneg = (A_angcol - B_angcol) % (2 * math.pi) # Negative B angle in A frame
    x_eBN = -l / 2
    #x_eAN = l / 2
    if abs(B_angNpos) < abs(B_angNneg):
        y_eBN = w / 2
        y_eAN = w / 2
    else:
        y_eBN = -w / 2
        y_eAN = -w / 2
        
    # Back corner of car B and front corner of car A at time of collision
    #x_eBA = B_poscol[0] - A_poscol[0] + x_eBN * math.cos(B_angcol - A_angcol) - y_eBN * math.sin(B_angcol - A_angcol) # Back corner x in A frame
    y_eBA = B_poscol[1] - A_poscol[1] + x_eBN * math.sin(B_angcol - A_angcol) + y_eBN * math.cos(B_angcol - A_angcol) # Back corner y in A frame
    
    # Calculate speed decrease
    t_extra = (y_eAN - y_eBA) / B_velcol
    d_col = math.sqrt((A_poscol[0] - A_pos[0]) ** 2 + (A_poscol[1] - A_pos[1]) ** 2)
    t_avoid = t_col + t_extra
    A_velavoid = d_col / t_avoid
    print(A_velavoid)
    return A_velavoid