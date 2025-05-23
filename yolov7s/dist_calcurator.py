import numpy as np
 
def angle_convert(x, y, z):
    X_angle = np.rad2deg(np.arctan2(x, z))
    Y_angle = np.rad2deg(np.arctan2(y, z))
    return np.round(X_angle, decimals=2), np.round(Y_angle, decimals=2)
    
    
def real_cordinate(cx, cy, x, y, fpx, z):
    real_x = (x-cx)*z / fpx
    real_y = (y-cy)*z / fpx
    print("real_x, (abs(x-cx)*z), real_y, (abs(y-cy)*z)", real_x, (abs(x-cx)*z), real_y, (abs(y-cy)*z))
    return real_x, real_y
    
def distance_formula(disparity, fpx, hyp):
    B = hyp['CAM_BASELINE'] # [mm]
    dist = (fpx*B)/disparity # [mm]
    print('f, B, dist', B, fpx, dist)
    return dist
    
def prams_calcurator(hyp, disparity, width, cx, cy, x, y):
    focal_px = (hyp['FOCAL_LENTH'] * width)/hyp['W_PIXEL_LENTH'] # [px]
    #print('fpxxxxxx, ', focal_px, width)
    distance = distance_formula(disparity, focal_px, hyp)
    real_x, real_y = real_cordinate(cx, cy, x, y, focal_px, distance)
    return disparity, np.round(distance, decimals=2), np.round(real_x, decimals=2), np.round(real_y, decimals=2)
 

