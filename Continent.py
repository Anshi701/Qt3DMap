from PyQt5.QtGui import QPainterPath, QColor
import math
import numpy as np
from scipy.spatial import Delaunay
import time
import random
import copy
import functions as funcs
import GLOBALS
import climate
from TriangleNet import TRIANGLE_POINTS, TRIANGLE_OBJECTS, Triangle

class ContinentVertex():
    def __init__(self, point_id, continent) -> None:
        self.continent: Continent = continent
        self.continent.vertices.append(self)
        self.point_id = point_id
        self.edges = [] # list of ContinentEdge
        self.border_edges = []
        self.fixed = False
        self.climatedata = None

    def __repr__(self) -> str:
        return f"\nVertex({self.value()}, edges: {len(self.edges)}({len(self.border_edges)}))"

    def setClimateInfo(self):
        spherical_point = funcs.decartToSpheric(self.value())
        latitude = int(math.degrees(spherical_point[1] - math.pi/2))
        i = np.where(climate.CLIMATE_DATA[:, 0] == latitude)[0][0]
        self.climatedata = climate.CLIMATE_DATA[i]

    def changePosition(self, crat):
        if self.fixed:
            return
        edge1: ContinentEdge = self.border_edges[0]
        vertex1 = edge1.getOtherVertex(self)
        check_vertex1 = None
        for v in edge1.continental_triangles[0].vertices:
            if v != vertex1 and v != self:
                check_vertex1 = v
        crs1 = funcs.myCrossProd(self.value() - vertex1.value(), check_vertex1.value() - vertex1.value())
        znak1 = math.copysign(1, funcs.scalarProd(crs1, vertex1.value()))
        edge2: ContinentEdge = self.border_edges[1]
        vertex2 = edge2.getOtherVertex(self)
        v = znak1 * (vertex1.value() - vertex2.value())
        move_axis = (v) / funcs.normOfVector(v)
        if self.continent.continent_radius == None:
            self.continent.updateInfo()
        new_value = funcs.kvaterRotation(self.value(), move_axis, -crat * GLOBALS.SPEED * GLOBALS.SPEED_CONVERT_COEF * 1/self.continent.continent_radius)
        self.continent.points[self.point_id] = new_value
    
    def initBorderEdges(self):
        self.border_edges = []
        for edge in self.edges:
            if edge.border:
                self.border_edges.append(edge)
    
    def distanceToEdge(self, edge):
        return edge.plane.DotP(self.value())
    
    def findEdge(self, vertex):
        for e in self.edges:
            if e.vertex_1 == vertex or e.vertex_2 == vertex:
                return e
        return None
    
    def getNextBorderEdge(self, edge):
        if edge == self.border_edges[0]:
            return self.border_edges[1]
        if edge == self.border_edges[1]:
            return self.border_edges[0]
        else: 
            return None
    
    def value(self, rotated_points = []):
        if rotated_points == []:
            return self.continent.points[self.point_id]
        else:
            return rotated_points[self.point_id]


class ContinentEdge():
    
    def __init__(self, vertex_1, vertex_2, continental_triangle=None) -> None:
        self.vertex_1: ContinentVertex = vertex_1
        self.vertex_2: ContinentVertex = vertex_2
        self.vertex_1.edges.append(self)
        self.vertex_2.edges.append(self)
        self.continental_triangles = []
        self.addContinentalTriangle(continental_triangle)
        self.recalcPlane()
        self.interpolatedPoints = []
        self.intersected = False
        self.border = False
    
    def __repr__(self) -> str:
        return f"\nEdge({self.vertex_1.value().astype(np.int32)}; {self.vertex_2.value().astype(np.int32)}, border: {self.border}, triangles: {len(self.continental_triangles)})"
    
    def deleteEdge(self):
        if self in self.vertex_1.edges: self.vertex_1.edges.remove(self)
        if self in self.vertex_2.edges: self.vertex_2.edges.remove(self)
        if self in self.vertex_1.continent.edges:        self.vertex_1.continent.edges.remove(self)
        if self in self.vertex_1.continent.border_edges: self.vertex_1.continent.border_edges.remove(self)
        if self in self.vertex_1.border_edges:  self.vertex_1.border_edges.remove(self)
        if self in self.vertex_2.border_edges:  self.vertex_2.border_edges.remove(self)
    
    def divideEdge(self, newVertex=None):
        if newVertex == None: 
            newPoint = 0.5 * (self.vertex_1.value() + self.vertex_2.value())
            newPoint = newPoint * GLOBALS.CIRCLE_RADIUS / funcs.normOfVector(newPoint)
            self.vertex_1.continent.points = np.concatenate([self.vertex_1.continent.points, [newPoint]])
            newVertex = ContinentVertex(len(self.vertex_1.continent.points)-1, self.vertex_1.continent)
        oposVerts = []
        tris = copy.copy(self.continental_triangles)
        for tri in tris:
            oposVerts.append(tri.getOpossideVertex(self))
            tri.deleteTriangle()
        for i in range(len(oposVerts)):
            ContinentTriangle(self.vertex_1.continent, [self.vertex_1, newVertex, oposVerts[i]])
            ContinentTriangle(self.vertex_1.continent, [self.vertex_2, newVertex, oposVerts[i]])
        self.vertex_1.continent.initBorders()
        print("divide ended")
    
    def setAsBorder(self):
        self.border = True
    
    def setAsNotBorder(self):
        self.border = False
    
    def lenght(self):
        return funcs.normOfVector(self.vertex_1.value() - self.vertex_2.value())
    
    def edgeIntersection(self, otherEdge):
        self.recalcPlane()
        otherEdge.recalcPlane()
        dt1e = self.vertex_1.distanceToEdge(otherEdge) # otherEdge.plane.DotP(self.vertex_1.value())
        dt2e = self.vertex_2.distanceToEdge(otherEdge) # otherEdge.plane.DotP(self.vertex_2.value())
        cross_edge1 = dt1e * dt2e < 0 
        dt1s = otherEdge.vertex_1.distanceToEdge(self) # self.plane.DotP(otherEdge.vertex_1.value())
        dt2s = otherEdge.vertex_2.distanceToEdge(self) # self.plane.DotP(otherEdge.vertex_2.value())
        cross_edge2 = dt1s * dt2s < 0 
        if cross_edge1 and cross_edge2: return True# , values
        else: return False# , values
    
    def addContinentalTriangle(self, triangle): 
        if triangle != None:
            if triangle not in self.continental_triangles:
                self.continental_triangles.append(triangle)
    
    def getOtherVertex(self, vertex):
        if   vertex == self.vertex_1: return self.vertex_2
        elif vertex == self.vertex_2: return self.vertex_1
        print("Error! No Vertex found!")
        return None
    
    def recalcPlane(self):
        self.plane = funcs.definePlane([[0, 0, 0], self.vertex_1.value(), self.vertex_2.value()])
    
    def interpolatePoints(self, rotated_points = []):
        p1 = self.vertex_1.value(rotated_points)
        p2 = self.vertex_2.value(rotated_points)
        self.interpolatedPoints = funcs.sphericalInterpolation(p1, p2)


class ContinentTriangle():
    def __init__(self, continent, vertices: ContinentVertex) -> None:
        self.continent: Continent = continent
        self.vertices: ContinentVertex = vertices
        self.continent.triangles.append(self)
        self.edges = []
        self.initEdges()
        self.color = QColor(48, 141, 30, 255)
        self.plane = self.getPlane()
        self.area = self.calcArea()
        self.climateType = None
    
    def __repr__(self) -> str:
        return f"\nTriangle({self.vertices[0].value()}; \n         {self.vertices[1].value()}; \n         {self.vertices[2].value()})"
    
    def getPlane(self):
        self.plane = funcs.definePlane([v.value() for v in self.vertices])
    
    def calcArea(self):
        self.getPlane()
        v1 = self.vertices[0].value() - self.vertices[2].value()
        v2 = self.vertices[1].value() - self.vertices[2].value()
        triangle_area = 0.5 * funcs.normOfVector(funcs.myCrossProd(v1, v2))
        a = funcs.normOfVector(self.vertices[0].value() - self.vertices[1].value())
        b = funcs.normOfVector(self.vertices[1].value() - self.vertices[2].value())
        c = funcs.normOfVector(self.vertices[2].value() - self.vertices[0].value())
        R = (a * b * c)/ (4 * triangle_area)
        circle_area = math.pi * R**2
        flat_area_coef = triangle_area / circle_area
        sphere_segment_area = 2 * math.pi * GLOBALS.CIRCLE_RADIUS * (GLOBALS.CIRCLE_RADIUS - math.fabs(self.plane.d))
        spherical_triangle_area = sphere_segment_area * flat_area_coef
        return spherical_triangle_area
    
    def initEdges(self):
        for i in range(3):
            v1: ContinentVertex = self.vertices[i]
            v2: ContinentVertex = self.vertices[(i+1)%3]
            edge:ContinentEdge = v1.findEdge(v2)
            if edge == None:
                edge = ContinentEdge(v1, v2, self)
                self.edges.append(edge)
                self.continent.edges.append(edge)
            else:
                self.edges.append(edge)
                edge.addContinentalTriangle(self)
    
    def divideTriangle(self):
        center = sum([v.value() for v in self.vertices])/3
        center = center * GLOBALS.CIRCLE_RADIUS / funcs.normOfVector(center)
        self.continent.points = np.concatenate([self.continent.points, [center]])
        newVertex = ContinentVertex(len(self.continent.points)-1, self.continent)
        self.deleteTriangle()
        for i in range(3):
            v1 = self.vertices[i]
            v2 = self.vertices[(i+1)%3]
            ContinentTriangle(self.continent, [v1, v2, newVertex])
        for i in range(3):
            v1 = self.vertices[i]
            v2 = self.vertices[(i+1)%3]
            edge = v1.findEdge(v2)
            edge.divideEdge()       
    
    def getClimateType(self):
        triangle_data = np.zeros((25))
        for vert in self.vertices:
            vert: ContinentVertex = vert
            triangle_data += vert.climatedata / 3
            self.climateType = climate.climateType(triangle_data)
            self.color = climate.climate_types[self.climateType]
    
    def getOpossideVertex(self, edge):
        for v in self.vertices:
            if v != edge.vertex_1 and v != edge.vertex_2:
                return v
    
    def getEdge(self, vertex1, vertex2):
        reversed_direction = False
        for edge in self.edges:
            if (edge.vertex_1 == vertex1 and edge.vertex_2 == vertex2):
                return edge, reversed_direction
            elif (edge.vertex_1 == vertex2 and edge.vertex_2 == vertex1):
                reversed_direction = True
                return edge, reversed_direction
        return None, reversed_direction        
    
    def deleteTriangle(self):
        self.continent.triangles.remove(self)
        for edge in self.edges:
            edge: ContinentEdge = edge
            edge.continental_triangles.remove(self)
            if edge.border:
                edge.deleteEdge()
        self.continent.initBorders()

class ContinentObject():
    def MovePoints(self, field, points, center_mass):
        beta, vector_forward, phi, shaft2 = self.forcesOfPoints(field, points, center_mass)
        shaft1 = funcs.myCrossProdNormal(vector_forward, center_mass)
        new_points = funcs.kvaterRotationArray(points, shaft1, beta)
        new_points = funcs.kvaterRotationArray(new_points, shaft2, phi)
        return new_points
    
    def forcesOfPoints(self, field, points, center_mass):
        components = self.getMovementComponents(field, points, center_mass)
        vector_forward = funcs.projectVectorToTangent(components[0:3], center_mass[0], center_mass[1], center_mass[2])
        fotate_force = components[3]
        self.movementVector = vector_forward
        # Поступательное движение
        beta = self.convertMoveScalToRads(funcs.normOfVector(vector_forward))
        # Вращательное движение
        phi = self.convertMoveScalToRads(fotate_force)
        shaft2 = center_mass / funcs.normOfVector(center_mass)
        return beta, vector_forward, phi, shaft2
    
    def convertMoveScalToRads(self, u):
        return u / (2 * math.pi * GLOBALS.CIRCLE_RADIUS) * GLOBALS.SPEED * GLOBALS.SPEED_CONVERT_COEF / 4
    
    def getMovementComponents(self, field, continent_points, center_mass):
        components = np.array(    (0, 0, 0, 0) , dtype=np.float64   )
        move_vectors = field.getMovementVectors(continent_points)
        AxisX, AxisY = funcs.findPerpendiculars(center_mass)
        AxisZ = -center_mass / funcs.normOfVector(center_mass)
        for i in range(len(continent_points)):
            point = continent_points[i]
            new_p = funcs.pointInNewBasis(point, AxisX, AxisY, AxisZ)
            new_p[2] = 0
            D1 = funcs.normOfVector(new_p)
            N1 = funcs.normOfVector(point - center_mass)
            point =  new_p * N1/D1
            v_point = continent_points[i] + move_vectors[i]
            new_v = funcs.pointInNewBasis(v_point, AxisX, AxisY, AxisZ)
            new_v[2] = 0
            D2 = funcs.normOfVector(new_v)
            N2 = funcs.normOfVector(v_point - center_mass)
            v_point = new_v * N2/D2
            vector = v_point - point
            vector = vector / funcs.normOfVector(vector) * funcs.normOfVector(move_vectors[i])
            angle = math.atan2(point[1], point[0])
            x = vector[0] * math.cos(-angle) - vector[1] * math.sin(-angle)
            y = vector[0] * math.sin(-angle) + vector[1] * math.cos(-angle)
            components[0:3] +=  point * x / N1
            components[3] += y # * N1
        components[0:4] /= len(continent_points)
        components[0:3] = center_mass +    (components[0] * AxisX + components[1] * AxisY + components[2] * AxisZ)
        return components
    
    def calcCenterMass(self, points):
        center_mass = np.array((0.0, 0.0, 0.0))
        for point in points:
            center_mass += point
        center_mass = center_mass / len(points)
        center_mass = center_mass * GLOBALS.CIRCLE_RADIUS / funcs.normOfVector(center_mass)
        return center_mass
    
    def calcMaxPointRadius(self, points, center_mass):
        radiuses = points - center_mass
        return np.amax(np.sqrt(np.sum(np.power(radiuses, 2), axis=1)))

class Continent(ContinentObject):
    def __init__(self, points=[], createNew=False) -> None:
        self.points = np.array(points, dtype=np.float64)
        self.center_mass       = None
        self.continent_radius  = None
        if points != []:
            self.center_mass      = self.calcCenterMass(self.points)
            self.continent_radius = self.calcMaxPointRadius(self.points, self.center_mass)
        self.vertices = [] 
        self.edges = []
        self.border_edges = []
        self.triangles = []
        self.area = 0
        self.superContinent = None
        self.stop = False
        self.toDelete = False
        if createNew: self.createNewContinent()
        self.movementVector = np.array((0, 0, 0))
    
    def __repr__(self) -> str:
        return f"Continent({len(self.vertices)} verticies; {len(self.edges)} edges ({len(self.border_edges)} borders); {len(self.triangles)} triangles)"
    
    def setClimate(self):
        for v in self.vertices:
            v.setClimateInfo()
        for tri in self.triangles:
            tri: ContinentTriangle = tri
            tri.getClimateType()
    
    def updateInfo(self):
        self.center_mass = self.calcCenterMass(self.points)
        self.continent_radius = self.calcMaxPointRadius(self.points, self.center_mass)
        self.area = 0
        for tri in self.triangles:
            tri.area = tri.calcArea()
            self.area +=tri.area
        self.divideAll()
    
    def divideAll(self):
        i = 0
        while i < len(self.triangles):
            tri: ContinentTriangle = self.triangles[i]
            if tri.area > GLOBALS.SPHERE_AREA * GLOBALS.MAXIMUM_AREA_PROCENT_OF_CONTINENT_TRIANGLE:
                tri.divideTriangle()
            else:
                i += 1
    
    def continentArea(self):
        self.area = 0
        for triangle in self.triangles:
            triangle: ContinentTriangle = triangle
            self.area += triangle.calcArea()
        return self.area
    
    def changeContinentArea(self, change_prob):
        cont_prob = random.randint(-100, 100) / 100
        edge = random.choice(self.border_edges)
        vertex = random.choice([edge.vertex_1, edge.vertex_2])
        if cont_prob <= change_prob:
            vertex.changePosition(1)
        else:
            vertex.changePosition(-1)
        self.center_mass = self.calcCenterMass(self.points)
        self.continent_radius = self.calcMaxPointRadius(self.points, self.center_mass)
    
    def MoveEvent(self, field, forced_move = False):
        if not forced_move:
            if self.stop: return
        self.center_mass = self.calcCenterMass(self.points)
        self.points = self.MovePoints(field, self.points, self.center_mass)
    
    def DelaunayTriang(self, points, center_mass):
        AxisX, AxisY = funcs.findPerpendiculars(center_mass)
        AxisZ = -center_mass / funcs.normOfVector(center_mass)
        projected_points = np.zeros(    points.shape , dtype=np.float64   )
        for i in range(len(points)):
            point = points[i]
            new_p = funcs.pointInNewBasis(point, AxisX, AxisY, AxisZ)
            new_p[2] = 0
            D1 = funcs.normOfVector(new_p)
            N1 = funcs.normOfVector(point - center_mass)
            projected_points[i] =  new_p * N1/D1
        return Delaunay(projected_points[:, 0:2], qhull_options="Qbb Qc Qz Q12").simplices
    
    def createNewContinent(self):
        for point_id in range(len(self.points)):
            ContinentVertex(point_id, self)
        if len(self.vertices) < 3:
            print("Слишком мало точек для триангуляции")
            return
        if len(self.vertices) == 3:
            ContinentTriangle(self, [self.vertices[0], self.vertices[1], self.vertices[2]])
        else:
            triangle_indexes = self.DelaunayTriang(self.points, self.center_mass)
            for i in range(triangle_indexes.shape[0]):
                ContinentTriangle(self, [self.vertices[triangle_indexes[i, 0]], 
                                        self.vertices[triangle_indexes[i, 1]], 
                                        self.vertices[triangle_indexes[i, 2]]])
        self.initBorders()
        self.divideAll()
        self.setClimate()
        
    def initBorders(self):
        self.border_edges = []
        for edge in self.edges:
            edge: ContinentEdge = edge
            edge.setAsNotBorder()
            if len(edge.continental_triangles) == 1:
                self.border_edges.append(edge)
                edge.setAsBorder()
        for vert in self.vertices:
            vert: ContinentVertex = vert
            vert.initBorderEdges()

class ContinentConnection(ContinentObject):
    def __init__(self, sphere, continent1, continent2, intersected_edges) -> None:
        self.sphere = sphere
        self.connected_continents = [continent1, continent2]
        self.newContinent = None
        self.points = np.concatenate([cont.points for cont in self.connected_continents])
        self.intersected_edges = intersected_edges
        self.edgeToSplit, self.intersectedEdgesVertices = self.initMainInfo()
        self.fixVerticies()
        self.timer, self.force_limit = self.initTimerAndLimit()
        self.movementVector = np.array((0, 0, 0))
    
    def initMainInfo(self):
        k = 0 # индекс материка, который пересекается двумя ребрами
        m = 1
        if self.intersected_edges[0][0] == self.intersected_edges[0][1]:
            k = 1
            m = 0
        point1 = self.intersected_edges[k][0].vertex_1
        point2 = self.intersected_edges[k][0].vertex_2
        point3 = self.intersected_edges[k][1].vertex_1
        point4 = self.intersected_edges[k][1].vertex_2
        verts = []
        if point1 == point3:
            verts = [point1, point2, point4]
        if point1 == point4:
            verts = [point1, point2, point3]
        if point2 == point3:
            verts = [point2, point1, point4]
        if point2 == point4:
            verts = [point2, point1, point3]
        edgeToSplit: ContinentEdge = self.intersected_edges[m][0]
        return edgeToSplit, verts
    
    def initTimerAndLimit(self):
        vert0: ContinentVertex = self.intersectedEdgesVertices[0]
        vert1: ContinentVertex = self.intersectedEdgesVertices[1]
        vert2: ContinentVertex = self.intersectedEdgesVertices[2]
        angle1 = vert1.distanceToEdge(self.edgeToSplit) / vert1.findEdge(vert0).lenght() # [0, 1]
        angle2 = vert2.distanceToEdge(self.edgeToSplit) / vert2.findEdge(vert0).lenght() # [0, 1]
        av_angle = 0.5 * (angle1 + angle2)
        timer_coeff = 1000
        force_limit_coeff = 100
        timer = av_angle * timer_coeff
        force_limit = self.convertMoveScalToRads(av_angle * force_limit_coeff)
        return timer, force_limit
    
    def checkState(self, field):
        self.transformToContinent()
    
    def checkIntersection(self):
        res = False
        for i in range(len(self.intersected_edges[0])):
            edge1: ContinentEdge = self.intersected_edges[0][i]
            edge2: ContinentEdge = self.intersected_edges[1][i]
            res = res or edge1.edgeIntersection(edge2)
        return res
    
    def fixVerticies(self):
        for v in self.intersectedEdgesVertices:
            v.fixed = True
        self.edgeToSplit.vertex_1.fixed = True
        self.edgeToSplit.vertex_2.fixed = True
    
    def unfixVerticies(self):
        for v in self.intersectedEdgesVertices:
            v.fixed = False
        self.edgeToSplit.vertex_1.fixed = False
        self.edgeToSplit.vertex_2.fixed = False
    
    def removeConnection(self):
        self.unfixVerticies()
        connects = self.connected_continents[0].superContinent.connections
        for c in connects:
            if c != self:
                for i in range(len(c.connected_continents)):
                    cnt: Continent = c.connected_continents[i]
                    cnt.stop = True
                    cnt.superContinent = None
        self.connected_continents[0].superContinent.connections = []
    
    def transformToContinent(self):
        self.unfixVerticies()
        newContinent = Continent()
        newContinent.points = self.points
        M = 0
        for i in range(len(self.connected_continents)):
            cont: Continent = self.connected_continents[i]
            for v in cont.vertices:
                v: ContinentVertex = v
                v.point_id += M
                v.continent = newContinent
            for tri in cont.triangles:
                tri: ContinentTriangle = tri
                tri.continent = newContinent
            newContinent.vertices   += cont.vertices
            newContinent.edges      += cont.edges
            newContinent.triangles  += cont.triangles
            M += len(cont.points)
        self.newContinent = newContinent
        self.sphere.applyTransformation(self)        
        connects = self.connected_continents[0].superContinent.connections
        for c in connects:
            if c != self:
                for i in range(len(c.connected_continents)):
                    cnt = c.connected_continents[i]
                    if cnt == self.connected_continents[0] or cnt == self.connected_continents[1]:
                        c.connected_continents[i] = self.newContinent
        connects.remove(self)
        newVertex = self.intersectedEdgesVertices[0]
        self.edgeToSplit.divideEdge(newVertex)
        check = [1, 1, 1, 1]
        connecting_edges = [
            ContinentEdge(self.intersectedEdgesVertices[1], self.edgeToSplit.vertex_1),
            ContinentEdge(self.intersectedEdgesVertices[1], self.edgeToSplit.vertex_2),
            ContinentEdge(self.intersectedEdgesVertices[2], self.edgeToSplit.vertex_1),
            ContinentEdge(self.intersectedEdgesVertices[2], self.edgeToSplit.vertex_2)
        ]
        b = 0
        for border in newContinent.border_edges:
            for i in range(4):
                edge: ContinentEdge = connecting_edges[i]
                if edge != border:
                    intersecting = edge.edgeIntersection(border)
                    if intersecting:
                        check[i] = 0
            b += 1
        print(self.newContinent)
        for i in range(4):
            edge: ContinentEdge = connecting_edges[i]
            edge.deleteEdge()
            if check[i] == 1:
                triangle = ContinentTriangle(self.newContinent, [edge.vertex_1, edge.vertex_2, newVertex])
        self.newContinent.initBorders()
    
    def resistanceReduction(self, field):
        forces = []
        for cont in self.connected_continents:
            cont.center_mass = self.calcCenterMass(cont.points)
            forces.append( self.forcesOfPoints(field, cont.points, cont.center_mass))
        cm1 = self.connected_continents[0].center_mass
        v1 = self.intersectedEdgesVertices[0].value() - cm1
        v1_pr = funcs.projectVectorToTangent(v1, cm1[0], cm1[1], cm1[2])
        angle1 = funcs.angleBetwVectors(forces[0][1], v1_pr)
        angle_coeff1 = math.cos(angle1)
        cm2 = self.connected_continents[1].center_mass
        v2 = self.intersectedEdgesVertices[0].value() - cm2
        v2_pr = funcs.projectVectorToTangent(v2, cm2[0], cm2[1], cm2[2])
        angle2 = funcs.angleBetwVectors(forces[1][1], v2_pr)
        angle_coeff2 = math.cos(angle2)
        direction_coeff = forces[0][0] * angle_coeff1 + forces[1][0] * angle_coeff2
        self.force_limit += direction_coeff

class SuperContinent(ContinentObject):
    def __init__(self, sphere, connection) -> None:
        self.sphere = sphere
        self.connections = [connection]
        self.continents = connection.connected_continents
        connection.connected_continents[0].superContinent = self
        connection.connected_continents[1].superContinent = self
        self.points = []
        self.updatePoints()
        self.center_mass = self.calcCenterMass(self.points)
    
    def addConnection(self, connection: ContinentConnection):
        other_superCont = None
        sc0 = connection.connected_continents[0].superContinent
        sc1 = connection.connected_continents[1].superContinent
        if sc0 != self and sc0 != None:
            other_superCont = sc0
        if sc1 != self and sc1 != None:
            other_superCont = sc1
        if other_superCont != None:
            for cont in other_superCont.continents:
                self.continents.append(cont)
                cont.superContinent = self
            for conn in other_superCont.connections:
                self.connections.append(conn)
            self.sphere.superContinents.remove(other_superCont)
        else:
            for cont in connection.connected_continents:
                if cont not in self.continents:
                    self.continents.append(cont)
        self.connections.append(connection)
        connection.connected_continents[0].superContinent = self
        connection.connected_continents[1].superContinent = self
        self.updatePoints()
    
    def checkConnections(self):
        for connection in self.connections:
            connection.checkState(self.sphere.field)      
    
    def updatePoints(self):
        self.points = np.concatenate([cont.points for cont in self.continents])
    
    def MoveEvent(self, field):
        if self.points == []:
            return
        self.center_mass = self.calcCenterMass(self.points)
        self.points = self.MovePoints(field, self.points, self.center_mass)
        M = 0
        for i in range(len(self.continents)):
            cont: Continent = self.continents[i]
            k = len(cont.points)
            cont.points = self.points[M:M+k, :]
            M += k