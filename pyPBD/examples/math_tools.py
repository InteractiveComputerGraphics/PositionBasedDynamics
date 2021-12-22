import math
import numpy as np


######################################################
# compute rotation matrix
######################################################
def rotation_matrix(angle, axis):
    x = axis[0]
    y = axis[1]
    z = axis[2]
    d = math.sqrt(x*x + y*y + z*z)
    if d < 1.0e-6:
        print ("Vector of rotation matrix is zero!")
        return
    x = x/d;
    y = y/d;
    z = z/d;

    x2 = x*x;
    y2 = y*y;
    z2 = z*z;
    s = math.sin(angle);
    c = math.cos(angle);
    c1 = 1.0-c;
    xyc = x*y*c1;
    xzc = x*z*c1;
    yzc = y*z*c1;
    xs=x*s;
    ys=y*s;
    zs=z*s;

    return np.array(((c + x2*c1, xyc-zs, xzc+ys),
			(xyc+zs, c+y2*c1, yzc-xs),
            (xzc-ys, yzc+xs, c+z2*c1)))
            
            