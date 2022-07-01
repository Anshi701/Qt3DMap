import math
import numpy as np
from numba import njit
import random
from GLOBALS import *

class Plane():
    def __init__(self, n, d) -> None:
        self.normal_vector = n
        self.d = d
    def prnt(self):
        print(f"[{self.normal_vector}|{self.d}]")
    def DotV(self, v):
        return self.normal_vector[0] * v[0] + self.normal_vector[1] * v[1] + self.normal_vector[2] * v[2]
    def DotP(self, p):
        l = self.normal_vector[0] * p[0] + self.normal_vector[1] * p[1] + self.normal_vector[2] * p[2] + self.d
        if math.fabs(l) < 0.000000001:
            l = 0.0
        return l

def generateFieldValues():
    polar_p = np.array( [[random.gauss(0.7, 0.1), fi] for fi in np.linspace(0.01, 2*math.pi, 12, endpoint=True)]  )
    polar_p[-1, 0] = polar_p[0, 0]
    X = polar_p[:, 1]
    Y = polar_p[:, 0]
    return X, Y

@njit
def projectVectorToTangent(vector, x, y, z):
    d = math.sqrt(x**2 + y**2 + z**2)
    n = np.array((x, y, z)) / d
    dtp = (n[0] * vector[0] + n[1] * vector[1] + n[2] * vector[2]) 
    proj_vector = vector - n * dtp
    return proj_vector / normOfVector(proj_vector) * normOfVector(vector)

@njit
def polarInterpolation(x, Xvalues, Yvalues):
    N = len(Xvalues)
    x = x % (2 * math.pi)
    for i in range(N):
        __i = (i-1)%N
        i_1 = (i+1)%N
        i_2 = (i+2)%N

        x0  = Xvalues[__i]
        x1  = Xvalues[i  ]
        x2  = Xvalues[i_1]
        x3  = Xvalues[i_2]

        y0 = Yvalues[__i]
        y1 = Yvalues[i]
        y2 = Yvalues[i_1]
        y3 = Yvalues[i_2]

        if x1 - x0 < 0:
            x0 = x0 - 2 * math.pi
        if x2 - x1 < 0:
            if x < x1:
                x  += 2 * math.pi
            x2 += 2 * math.pi
            x3 += 2 * math.pi
        elif x3 - x2 < 0: 
            x3 += 2 * math.pi

        if x == Xvalues[i]:
            return Yvalues[i]

        if x > x1 and x < x2:
            ##print(f"x: {x} \n p0: {x0}, {y0} \n p1: {x1}, {y1} \n p2: {x2}, {y2} \n p3: {x3}, {y3} \n" )
            F1 = y1
            F2 = y2
            f1 = (y2 - y0)/(2)
            f2 = (y3 - y1)/(2)

            q11 = F1
            q12 = F2
            q13 = f1
            q14 = f2

            a11 = x1**3 
            a12 = x1**2
            a13 = x1
            a14 = 1

            a21 = x2**3 
            a22 = x2**2
            a23 = x2
            a24 = 1

            a31 = 3 * x1 ** 2
            a32 = 2 * x1 
            a33 = 1
            a34 = 0

            a41 = 3 * x2 ** 2
            a42 = 2 * x2 
            a43 = 1
            a44 = 0

            # a11 a12 a13 a14 | q11
            # a21 a22 a23 a24 | q12
            # a31 a32 a33 a34 | q13
            # a41 a42 a43 a44 | q14

            b11 = a22 - a21 * a12/a11
            b12 = a23 - a21 * a13/a11
            b13 = a24 - a21 * a14/a11

            b21 = a32 - a31 * a12/a11
            b22 = a33 - a31 * a13/a11
            b23 = a34 - a31 * a14/a11

            b31 = a42 - a41 * a12/a11
            b32 = a43 - a41 * a13/a11
            b33 = a44 - a41 * a14/a11

            q21 = q12 - a21 * q11/a11
            q22 = q13 - a31 * q11/a11
            q23 = q14 - a41 * q11/a11

            # b11 b12 b13 | q21
            # b21 b22 b23 | q22
            # b31 b32 b33 | q23

            c11 = b22 - b21 * b12/b11
            c12 = b23 - b21 * b13/b11

            c21 = b32 - b31 * b12/b11
            c22 = b33 - b31 * b13/b11

            q31 = q22 - b21 * q21/b11 
            q32 = q23 - b31 * q21/b11 

            # c11 c12 | q31
            # c21 c22 | q32

            #  D = (q32 - c21 * q31 / c11) / (c22 - c21 * c12 / c11)

            D = (q32 - c21 * q31 / c11) / (c22 - c21 * c12 / c11)
            C = q31/c11 - c12/c11 * D
            B = q21/b11 - b12/b11 * C - b13/b11 * D
            A = q11/a11 - a12/a11 * B - a13/a11 * C - a14/a11 * D
            
            return A * (x  ** 3) + B * (x  ** 2) + C * (x) + D

def sphericalInterpolation(point1, point2):
    v = point2 - point1
    ln = math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2 ) # normOfVector(v)
    n = int(ln * INTERPOLATION_ACCURACY + 1)
    z = np.array((0.0, 0.0, 0.0))
    interp_points = np.linspace(z, v, n, endpoint=True)
    for i in range(n):
        point = interp_points[i] + point1
        d = math.sqrt(point[0] ** 2 + point[1] ** 2 + point[2] ** 2 )
        interp_points[i] = point * CIRCLE_RADIUS / d # projectPointOnSphere(interp_points[:, i])
    return interp_points # np.array(points)

def flatAngle(vector):
    return math.atan2(vector[1], vector[0])

def isPointInsideSector(point, figure_points):
    odd = False
    check_plane = definePlane([point, [0, 0, 0], [1, 0, 0]])
    l = len(figure_points)
    for k in range(1, l+1):
        p1 = np.asarray(figure_points[(k-1  )%(l)])
        p2 = np.asarray(figure_points[(k    )%(l)])
        dt1 = check_plane.DotP(p1)
        dt2 = check_plane.DotP(p2)
        if dt1*dt2 <= 0:
            b, q = PlaneIntersectLinePoint(check_plane, [p2 - p1, p1])
            if not b : 
                print("wtf?!")
            odd = not odd
    return odd

def PlaneIntersectLinePoint(f: Plane, line):
    v = line[0]
    p = line[1]
    fv = f.DotV(line[v])
    if math.fabs(fv) > 0:
        q = p - v * (f.DotP(p)/fv)
        return True, q
    return False, None

def randomPointOnSphere(acc = 1000):
    z = random.randint(-int(CIRCLE_RADIUS * acc), int(CIRCLE_RADIUS * acc)) / acc
    f = random.randint(0, int(math.pi*2 * acc)) / acc
    r = math.sqrt(CIRCLE_RADIUS**2 - z**2)
    x = r * math.cos(f)
    y = r * math.sin(f)
    return x, y, z

def distanceToPlane(pt, p0, p1, p2):
    a11 = pt[0] - p0[0];a12 = p1[0] - p0[0];a13 = p2[0] - p0[0]
    a21 = pt[1] - p0[1];a22 = p1[1] - p0[1];a23 = p2[1] - p0[1]
    a31 = pt[2] - p0[2];a32 = p1[2] - p0[2];a33 = p2[2] - p0[2]
    dt =    a11 * a22 * a33 - \
            a11 * a23 * a32 - \
            a12 * a21 * a33 + \
            a12 * a23 * a31 + \
            a13 * a21 * a32 - \
            a13 * a22 * a31 
    return dt

def definePlane(points, check_point=[], id=0):
    a11 = points[0][0];a12 = points[1][0] - points[0][0];a13 = points[2][0] - points[0][0]
    a21 = points[0][1];a22 = points[1][1] - points[0][1];a23 = points[2][1] - points[0][1]
    a31 = points[0][2];a32 = points[1][2] - points[0][2];a33 = points[2][2] - points[0][2]
    normal = myCrossProd((a12, a22, a32), (a13, a23, a33))
    z = normOfVector(normal)
    d = (a11 * normal[0] + a21 * normal[1] + a31 * normal[2])
    
    plane = Plane(normal/z, d/z)
    if len(check_point) > 0:
        k = plane.DotP(check_point)
        if k < 0: 
            plane.normal_vector = plane.normal_vector * -1 
    return plane

def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / normOfVector(vector)# np.linalg.norm(vector)

def angle_between(v1, v2):
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

def angleBetwVectors(v1, v2):
    md1 = normOfVector(v1)
    md2 = normOfVector(v2)
    angle = math.acos((v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2])/(md1 * md2))
    return angle

def projectPointOnSphere(point):
    D = math.sqrt(point[0]**2 + point[1]**2 + point[2]**2)
    new_point = (
        point[0] * CIRCLE_RADIUS/D,
        point[1] * CIRCLE_RADIUS/D,
        point[2] * CIRCLE_RADIUS/D
    )
    return np.asarray(new_point)

def distBetwPoints(p1, p2):
    v = [
        p1[0] - p2[0],
        p1[1] - p2[1],
        p1[2] - p2[2],
    ]
    return normOfVector(v)

@njit
def vectorAngle(vctr, axisX, axisY, axisZ=None):
    if axisZ == None:
        axisZ = myCrossProd(axisX, axisY)
    vector = pointInNewBasis(vctr, axisX, axisY, axisZ) # funcs.myCrossProd(axisX, axisY)
    if vector[1] == 0 and vector[0] == 0:
        return 0
    angle = math.atan2(vector[1], vector[0])
    if angle < 0:
        angle = 2 * math.pi - angle
    return angle

@njit
def findPerpendiculars(vector):
    x = vector[0]
    y = vector[1]
    z = vector[2]
    n = np.array((0.0, 0.0, 0.0))
    if   z != 0:
        C = (x + y)/z
        v = np.array((-1, -1, C))
    elif y != 0: 
        C = (x + z)/y
        v = np.array((-1, C, -1))
    elif x != 0:
        C = (y + z)/x
        v = np.array((C, -1, -1))
    else:
        return np.array((1.0, 0.0, 0.0)), np.array((0.0, 1.0, 0.0))
    
    v = v / math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)
    n = np.array((y * v[2] - z * v[1], 
                  z * v[0] - x * v[2], 
                  x * v[1] - y * v[0]),
                  dtype=np.float64
    )
    d = math.sqrt(n[0] ** 2 + n[1] ** 2 + n[2] ** 2)
    if d == 0: d = 1
    n = n / d
    return v, n

@njit
def normOfVector(v1):
    return math.sqrt(v1[0] * v1[0] + v1[1] * v1[1] + v1[2] * v1[2])

@njit
def allPointsRotationOffset(points, rotation_x, rotation_y, rotation_z):
    rotated_axisX = np.array((0, 0, 0))
    rotated_axisY = np.array((0, 0, 0))
    rotated_axisZ = np.array((0, 0, 0))

    rotated_axisX = kvaterRotation(AXIS_X,        AXIS_Z,  -rotation_z)
    rotated_axisY = kvaterRotation(AXIS_Y,        AXIS_Z,  -rotation_z)

    rotated_axisZ = kvaterRotation(AXIS_Z       , rotated_axisY,  -rotation_y)
    rotated_axisX = kvaterRotation(rotated_axisX, rotated_axisY,  -rotation_y)

    rotated_axisY = kvaterRotation(rotated_axisY, rotated_axisX,  -rotation_x)
    rotated_axisZ = kvaterRotation(rotated_axisZ, rotated_axisX,  -rotation_x)

    new_points = np.zeros((len(points), 3))
    for i in range(len(points)):
        new_points[i] = pointInNewBasis(points[i], rotated_axisX, rotated_axisY, rotated_axisZ)

    return new_points

@njit
def pointInNewBasis(point, axis_x, axis_y, axis_z):
    x = projectVectorOnVector(point, axis_x)
    y = projectVectorOnVector(point, axis_y)
    z = projectVectorOnVector(point, axis_z)
    return np.array((x, y, z))

@njit
def projectVectorOnVector(vectorA, vectorB):
    dot = vectorA[0] * vectorB[0] + vectorA[1] * vectorB[1] + vectorA[2] * vectorB[2]
    return dot / (vectorB[0] * vectorB[0] + vectorB[1] * vectorB[1] + vectorB[2] * vectorB[2])

@njit
def Dot(vectorA, vectorB):
    return vectorA[0] * vectorB[0] + vectorA[1] * vectorB[1] + vectorA[2] * vectorB[2] 

@njit
def kvaterRotation(v: np.ndarray, u: np.ndarray, f):
    qp0 = u[0] * v[0] + u[1] * v[1] + u[2] * v[2]
    return v * math.cos(f) + myCrossProd(u, v) *  math.sin(f) + u * qp0 * (1 - math.cos(f))

def kvaterRotationArray(p, u, f):
    sin  = np.sin(f/2)
    Sa   = np.cos(f/2)
    q    = sin * u
    qp0  = np.matmul(p, -1 * q)
    ABC  = np.cross(q, p) + Sa * p 
    qqp  = np.outer(qp0, q) 
    prm  = -1 * np.cross(ABC, q) - qqp
    trr  = Sa * ABC
    ans2 = prm + trr 
    return ans2

def scalarProd(vectorA, vectorB):
    return vectorA[0] * vectorB[0] + vectorA[1] * vectorB[1] + vectorA[2] * vectorB[2]

@njit
def myCrossProd(vector, vector2):
    return np.array([vector2[1] * vector[2] - vector2[2] * vector[1], 
                     vector2[2] * vector[0] - vector2[0] * vector[2], 
                     vector2[0] * vector[1] - vector2[1] * vector[0]]
    )

@njit
def myCrossProdNormal(vector, vector2):
    vctr = myCrossProd(vector, vector2)
    return vctr / (math.sqrt(vctr[0] ** 2 + vctr[1] ** 2 + vctr[2] ** 2))

@njit
def sphericToDecart(point):
    xyz = np.array([
        math.sin(point[1]) * math.cos(point[2]), 
        math.sin(point[1]) * math.sin(point[2]), 
        math.cos(point[1])
    ]) * point[0]
    return xyz

@njit
def decartToSpheric(point):
    x = point[0]
    y = point[1]
    z = point[2]
    r = math.sqrt(x*x + y*y + z*z)
    g = math.atan2(math.sqrt(x*x + y*y), z)
    if    g < 0:  g = 2 * math.pi + g
    f = math.atan2(y, x)
    if    f < 0:  f = 2 * math.pi + f
    return np.array((r, g, f))
