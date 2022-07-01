import math
import functions as funcs
import GLOBALS
import numpy as np
from Continent import *
from ForceFieldClass import ForceField

class Sphere():
    def __init__(self) -> None:
        self.continents = []
        self.superContinents = []
        self.field = ForceField()
        self.initContinents()
        self.procent_of_continental_area = 0

    def evolutionStep(self):
        self.updateContinentsInfo()
        self.moveContinents()
        self.areaControl()

    def areaControl(self):
        k = len(self.continents)
        total_area = 0
        for i in range(k):
            cont: Continent = self.continents[i]
            ar = cont.area
            total_area += ar
        self.procent_of_continental_area = (total_area/GLOBALS.SPHERE_AREA) * 100
        procent_difference = GLOBALS.CONTINENTS_AREA_PROCENT - self.procent_of_continental_area
        area_changing_probability = procent_difference/GLOBALS.CONTINENTS_AREA_PROCENT
        if area_changing_probability < -1:
            area_changing_probability = -1
        if area_changing_probability > 1:
            area_changing_probability = 1
        area_changing_probability = 0.7
        for i in range(k):
            cont: Continent = self.continents[i]
            cont.changeContinentArea(area_changing_probability)
        create_cont_prob = random.randint(-100, 100) / 100
        if area_changing_probability <= create_cont_prob and len(self.continents) < GLOBALS.MAXIMUM_OF_CONTINENTS:
            self.createNewContinent()

    def createNewContinent(self):
        rand_point1 = np.array(funcs.randomPointOnSphere())
        rad = GLOBALS.CIRCLE_RADIUS * GLOBALS.NEW_CONTINENT_RADIUS_PROCENT
        k = len(self.continents)
        for i in range(k):
            cont: Continent = self.continents[i]
            distance = funcs.normOfVector(rand_point1 - cont.center_mass)
            if distance <= (cont.continent_radius + rad):
                return
        rand_point2 = np.array(funcs.randomPointOnSphere()) / GLOBALS.CIRCLE_RADIUS * rad
        rand_point2 = rand_point1 + rand_point2

        rand_point2 = rand_point2 / funcs.normOfVector(rand_point2) * GLOBALS.CIRCLE_RADIUS
        axis = funcs.unit_vector(rand_point1)
        rand_point3 = funcs.kvaterRotation(rand_point2, axis, math.pi/4)
        points = np.array([
            rand_point1,
            rand_point2,
            rand_point3
        ])
        self.continents.append(Continent(points , createNew=True ))

    def setClimateOfContinents(self):
        for cont in self.continents:
            cont.setClimate()

    def updateContinentsInfo(self):
        i = 0
        while i < len(self.continents):
            cont1: Continent = self.continents[i]
            if cont1.toDelete: 
                self.continents.remove(cont1)
            else:
                cont1.updateInfo()
            i += 1

    def moveContinents(self): 
        k = len(self.continents)
        if k == 0: return
        if k > 1:
            for i in range(k):
                cont1: Continent = self.continents[i]
                if i+1 != k: # Последний континент проверять с собой нет смысла
                    for j in range(i+1, k):
                        cont2: Continent = self.continents[j]
                        self.rangeCheck(cont1, cont2)
                cont1.MoveEvent(self.field)
        else:
            cont1: Continent = self.continents[0]
            cont1.MoveEvent(self.field)
        mk = len(self.superContinents)
        for i in range(mk):
            superContinent: ContinentConnection = self.superContinents[i]
            superContinent.MoveEvent(self.field)
        self.superContinentsCheck()

    def applyTransformation(self, connection: ContinentConnection):
        self.continents.remove(connection.connected_continents[0])
        self.continents.remove(connection.connected_continents[1])
        self.continents.append(connection.newContinent)

    def superContinentsCheck(self):
        i = 0
        while i < len(self.superContinents):
            superContinent: SuperContinent = self.superContinents[i]
            if len(superContinent.connections) == 0:
                self.superContinents.remove(superContinent)
            superContinent.checkConnections()
            i += 1

    def rangeCheck(self, continent1: Continent, continent2: Continent):
        if continent1.superContinent != None and (continent1.superContinent == continent2.superContinent):
            return
        mass_distance = funcs.normOfVector(continent2.center_mass - continent1.center_mass)
        radius_distance = mass_distance - continent1.continent_radius - continent2.continent_radius
        intersected_edges = [[], []]
        if radius_distance <= 0:
            for i in range(len(continent1.border_edges)):
                for j in range(len(continent2.border_edges)):
                    edge1: ContinentEdge = continent1.border_edges[i]
                    edge2: ContinentEdge = continent2.border_edges[j]
                    res = edge1.edgeIntersection(edge2)
                    if res:
                        intersected_edges[0].append(edge1)
                        intersected_edges[1].append(edge2)
        if intersected_edges[0] != []:
            ########### поглощение ###########
            if continent1.superContinent == None:
                area = continent1.continentArea()
                # print(f"1: {area} < {GLOBALS.SPHERE_AREA * GLOBALS.MAXIMUM_AREA_PROCENT_OF_CONTINENT_TRIANGLE}")
                if area < GLOBALS.SPHERE_AREA * GLOBALS.MAXIMUM_AREA_PROCENT_OF_CONTINENT_TRIANGLE:
                    continent1.toDelete = True # self.continents.remove(continent1)
                    for v in continent2.vertices:
                        v: ContinentVertex = v
                        if len(v.border_edges) > 0:
                            v.changePosition(GLOBALS.CHANGE_POSITION_COEF * (area / GLOBALS.SPHERE_AREA))
                    return
            if continent2.superContinent == None:
                area = continent2.continentArea()
                # print(f"2: {area} < {GLOBALS.SPHERE_AREA * GLOBALS.MAXIMUM_AREA_PROCENT_OF_CONTINENT_TRIANGLE}")
                if area < GLOBALS.SPHERE_AREA * GLOBALS.MAXIMUM_AREA_PROCENT_OF_CONTINENT_TRIANGLE:
                    continent2.toDelete = True # self.continents.remove(continent2)
                    for v in continent1.vertices:
                        v: ContinentVertex = v
                        if len(v.border_edges) > 0:
                            v.changePosition(GLOBALS.CHANGE_POSITION_COEF * (area / GLOBALS.SPHERE_AREA))
                    return
            #################################
            new_conn = ContinentConnection(self, continent1, continent2, intersected_edges)
            if continent1.superContinent != None:
                continent1.superContinent.addConnection(new_conn)
                return
            if continent2.superContinent != None:
                continent2.superContinent.addConnection(new_conn)
                return
            new_superContinent = SuperContinent(self, new_conn)
            self.superContinents.append(new_superContinent)
            # print("Created super continent")
            return
            
    def initContinents(self):
        self.continents = []
        self.superContinents = []
        handmade1 = np.array([
            [GLOBALS.CIRCLE_RADIUS, math.pi/2 - 0.36 - math.pi/10, -0.59],
            [GLOBALS.CIRCLE_RADIUS, math.pi/2 - 0.61 - math.pi/10, -0.39],
            [GLOBALS.CIRCLE_RADIUS, math.pi/2 - 0.92 - math.pi/10, -0.93],
            [GLOBALS.CIRCLE_RADIUS, math.pi/2 - 0.86 - math.pi/10, -0.51],
            [GLOBALS.CIRCLE_RADIUS, math.pi/2 - 1.24 - math.pi/10, -0.13],
            [GLOBALS.CIRCLE_RADIUS, math.pi/2 - 0.99 - math.pi/10,  0.02],
            [GLOBALS.CIRCLE_RADIUS, math.pi/2 - 0.64 - math.pi/10,  0.06],
            [GLOBALS.CIRCLE_RADIUS, math.pi/2 - 1.03 - math.pi/10,  0.64],
            [GLOBALS.CIRCLE_RADIUS, math.pi/2 -  0.4 - math.pi/10,  0.54],
            [GLOBALS.CIRCLE_RADIUS, math.pi/2 - 0.29 - math.pi/10, -0.27],
        ])
        # handmade2 = np.array([
        #     [GLOBALS.CIRCLE_RADIUS, math.pi/2 - 1.2 , 2.44],
        #     [GLOBALS.CIRCLE_RADIUS, math.pi/2 - 1.01, 2.62],
        #     [GLOBALS.CIRCLE_RADIUS, math.pi/2 - 1.03, 2.36],
        #     [GLOBALS.CIRCLE_RADIUS, math.pi/2 - 0.91, 2.4 ],
        #     [GLOBALS.CIRCLE_RADIUS, math.pi/2 - 0.92, 2.16],
        #     [GLOBALS.CIRCLE_RADIUS, math.pi/2 - 1.02, 2.14],
        #     [GLOBALS.CIRCLE_RADIUS, math.pi/2 - 1.12, 2.16],
        #     [GLOBALS.CIRCLE_RADIUS, math.pi/2 - 1.17, 1.93],
        self.continents.append(Continent(  np.array(np.apply_along_axis(funcs.sphericToDecart, 1, handmade1), dtype=np.float64) , createNew=True     ))
        # self.continents.append(Continent(  np.array(np.apply_along_axis(funcs.sphericToDecart, 1, handmade2), dtype=np.float64) , createNew=True     ))