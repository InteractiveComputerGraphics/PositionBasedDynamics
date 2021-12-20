import math

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

    return [[c + x2*c1, xyc-zs, xzc+ys],
			[xyc+zs, c+y2*c1, yzc-xs],
            [xzc-ys, yzc+xs, c+z2*c1]]
            
######################################################
# compute matrix vector product
######################################################
def matrix_vec_product(A, v):
    res = [0,0,0]
    for i in range(0,3):
        for j in range(0,3):
            res[i] += A[i][j] * v[j];
    return res

######################################################
# compute cross product
######################################################
def cross_product(a, b):
    res = [0,0,0]
    res[0] = a[1]*b[2] - a[2]*b[1];
    res[1] = a[2]*b[0] - a[0]*b[2];
    res[2] = a[0]*b[1] - a[1]*b[0];
    return res

######################################################
# scale vector
######################################################
def scale_vector(v, s):
    res = [0,0,0]
    res[0] = s*v[0];
    res[1] = s*v[1];
    res[2] = s*v[2];
    return res
	
######################################################
# add vector
######################################################
def add_vector(v1, v2):
    res = [0,0,0]
    res[0] = v1[0] + v2[0];
    res[1] = v1[1] + v2[1];
    res[2] = v1[2] + v2[2];
    return res	            