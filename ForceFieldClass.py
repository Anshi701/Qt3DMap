import GLOBALS
import random
import math
import numpy as np
from PIL import Image
from PyQt5.QtGui import QImage, QPixmap
from numba import njit
import functions as funcs
import time

@njit
def angleSystem(x, y, z):
    return funcs.findPerpendiculars((x, y, z)) #v, n

@njit
def applyForce(field, x, y, z, angle_vector, fieldFunction):
    vector = np.array((
        x - field[0], 
        y - field[1],
        z - field[2]
    ))
    d = math.sqrt(vector[0] ** 2 + vector[1] ** 2 + vector[2] ** 2)
    c = 1
    cf = 1
    if d != 0: 
        angle = funcs.vectorAngle(vector, angle_vector[0], angle_vector[1])
        angle_coef = funcs.polarInterpolation(angle, fieldFunction[0], fieldFunction[1])
        f = field[3] #* angle_coef
        if f == 0: 
            c = 0
        else:   
            dn = d / 360   
            c = angle_coef/((dn**2) + 1)
        cf = c * f / d
    return vector * cf, c

@njit
def generatePixelArray(radius, field, field_functions, direction_vectors, front=True, map_accuracy=17):
    h = 2*radius
    pixels1 = np.zeros((h, h, 2), dtype = np.float64) # dtype=np.uint8
    pixels2 = np.zeros((h, h, 2), dtype = np.float64)
    Rsqr = radius**2
    forces_count = len(field)
    rng = range(-radius, radius)
    mi_coeff1 = 10000000#forces_count
    maximum_coeff1 = 0
    mi_coeff2 = 10000000#forces_count
    maximum_coeff2 = 0
    sumVector = np.array((0, 0, 0), dtype=np.float64)
    for y in     rng:
        for x in rng:
            j = radius - y - 1
            i = x + radius - 1
            if x**2 + y**2 > Rsqr:
                continue
            z = math.sqrt(Rsqr - (x) ** 2 - (y) ** 2)
            if not front: z = -z
            resForceCoeff = 0
            sumVector[0] = 0
            sumVector[1] = 0
            sumVector[2] = 0
            for f in range(forces_count):
                vector, coef = applyForce(field[f], x, y, z, direction_vectors[f], field_functions[f])
                sumVector += vector
                resForceCoeff += coef #/ forces_count
            if resForceCoeff > maximum_coeff1:  maximum_coeff1 = resForceCoeff
            if resForceCoeff < mi_coeff1:        mi_coeff1 = resForceCoeff
            pixels1[j, i, 0] = resForceCoeff
            D = math.sqrt(sumVector[0]**2 + sumVector[1]**2 + sumVector[2]**2)
            if D > maximum_coeff2:  maximum_coeff2 = D
            if D < mi_coeff2:       mi_coeff2 = D
            pixels1[j, i, 1] = 255
            pixels2[j, i, 0] = D
            pixels2[j, i, 1] = 255
    pixels1[:, :, 0] = (pixels1[:, :, 0]-mi_coeff1) / (maximum_coeff1 -mi_coeff1) *255 #- pixels % map_accuracy
    pixels1[:, :, 0] -= pixels1[:, :, 0] % map_accuracy
    pixels2[:, :, 0] = (pixels2[:, :, 0]-mi_coeff2) / (maximum_coeff2 -mi_coeff2) *255 #- pixels % map_accuracy
    pixels2[:, :, 0] -= pixels2[:, :, 0] % map_accuracy
    return pixels1.astype(np.uint8),pixels2.astype(np.uint8), #, vectors

def initCcode():
    f = (1, 1, 1, 1)
    fieldFunctions = []
    direction_vectors = []
    for i in range(len(f)):
        X, Y = funcs.generateFieldValues()
        fieldFunctions.append([X, Y])

        direction_vectors.append(angleSystem(1, 1, 1))
    fieldFunctions = np.array(   fieldFunctions   )
    direction_vectors = np.array(   direction_vectors   )
    l = generatePixelArray (10,  np.array((f, f, f)), fieldFunctions, direction_vectors)
initCcode()

@njit
def calcMovementVectors(points, field, field_functions, direction_vectors):
    vectors = np.zeros((len(points), 3), dtype=np.float64)
    for p in range(len(points)):
        x = points[p][0]
        y = points[p][1]
        z = points[p][2]
        for f in range(len(field)):
            vector, coef = applyForce(field[f], x, y, z, direction_vectors[f], field_functions[f])
            vectors[p] += vector
        vectors[p] = funcs.projectVectorToTangent(vectors[p], x, y, z)
    return vectors


class ForceField():
    def __init__(self) -> None:
        self.field              = []
        self.direction_vectors  = []
        self.field_functions    = []
        self.current_seed = GLOBALS.GENERATION_SEED
        random.seed(self.current_seed)
        self.updatedImages = False
        self.temp_pixmap_front  = QPixmap(int(GLOBALS.CIRCLE_RADIUS*2), int(GLOBALS.CIRCLE_RADIUS*2))
        self.temp_pixmap_back   = QPixmap(int(GLOBALS.CIRCLE_RADIUS*2), int(GLOBALS.CIRCLE_RADIUS*2))
        self.speed_pixmap_front = QPixmap(int(GLOBALS.CIRCLE_RADIUS*2), int(GLOBALS.CIRCLE_RADIUS*2))
        self.speed_pixmap_back  = QPixmap(int(GLOBALS.CIRCLE_RADIUS*2), int(GLOBALS.CIRCLE_RADIUS*2))
        self.createField()
        # self.create2DImage()

    def getMovementVectors(self, points):
        return calcMovementVectors(points, self.field, self.field_functions, self.direction_vectors)

    def createField(self):
        forces = []
        direction_vectors = []
        fieldFunctions = []
        sm_force = 0
        acc = 1000
        if self.current_seed != GLOBALS.GENERATION_SEED:
            self.current_seed = GLOBALS.GENERATION_SEED
            random.seed(self.current_seed)
        cf = 1
        for _ in range(GLOBALS.NUMBER_OF_FIELDS):
            x, y, z = funcs.randomPointOnSphere(acc)
            force = random.randint(int(GLOBALS.MIN_FORCE * cf), int(GLOBALS.MAX_FORCE * cf))
            sm_force += force
            forces.append([x, y, z, force])
            direction_vectors.append(angleSystem(x, y, z))
            X, Y = self.generateFieldValues()
            fieldFunctions.append([X, Y])
        forces = np.array(forces)
        self.field              = forces
        self.direction_vectors  = np.array(direction_vectors)
        self.field_functions    = np.array(fieldFunctions)
        self.updatedImages = False
        print("Field created")

    def create2DImage(self):
        print("Creating images...")
        start = time.time()
        pixels1,pixels2 = generatePixelArray(int(GLOBALS.CIRCLE_RADIUS*2)//2, self.field, self.field_functions, self.direction_vectors, front=True)
        new_image1 = Image.fromarray(pixels1, mode="LA")
        new_image2 = Image.fromarray(pixels2, mode="LA")
        self.temp_pixmap_front = self.pil2pixmap(new_image1)
        self.speed_pixmap_front = self.pil2pixmap(new_image2)
        pixels1,pixels2 = generatePixelArray(int(GLOBALS.CIRCLE_RADIUS*2)//2, self.field, self.field_functions, self.direction_vectors, front=False)
        new_image1 = Image.fromarray(pixels1, mode="LA")
        new_image2 = Image.fromarray(pixels2, mode="LA")
        self.temp_pixmap_back = self.pil2pixmap(new_image1)
        self.speed_pixmap_back = self.pil2pixmap(new_image2)
        self.updatedImages = True
        print(f"Images created ({time.time() - start}).")

    def pil2pixmap(self, im):
        if im.mode == "RGB":
            r, g, b = im.split()
            im = Image.merge("RGB", (b, g, r))
        elif  im.mode == "RGBA":
            r, g, b, a = im.split()
            im = Image.merge("RGBA", (b, g, r, a))
        elif im.mode == "L":
            im = im.convert("RGBA")
        elif im.mode == "LA":
            im = im.convert("RGBA")
        # Bild in RGBA konvertieren, falls nicht bereits passiert
        im2 = im.convert("RGBA")
        data = im2.tobytes("raw", "RGBA")
        qim = QImage(data, im.size[0], im.size[1], QImage.Format_ARGB32)
        pixmap = QPixmap.fromImage(qim)
        return pixmap

    def generateFieldValues(self):
        polar_p = np.array( [[random.gauss(0.7, 0.1), fi] for fi in np.linspace(0.01, 2*math.pi, 12, endpoint=True)]  )
        polar_p[-1, 0] = polar_p[0, 0]
        X = polar_p[:, 1]
        Y = polar_p[:, 0]
        return X, Y