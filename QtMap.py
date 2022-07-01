import sys
import time
import numpy as np
import math
from PyQt5.QtWidgets import QWidget, QApplication, QVBoxLayout
from PyQt5.QtGui import QBrush, QPainter, QColor, QPainterPath, QPen, QRadialGradient
from PyQt5.QtCore import Qt, QTimer, QPoint
import functions as funcs
import GLOBALS 
from Sphere import Sphere
from Continent import ContinentTriangle, ContinentVertex, ContinentEdge, Continent
from UIObjects import *

class Map(QWidget):
    def __init__(self, size):
        super().__init__()
        self.size = size
        self.W = self.size.width()
        self.H = self.size.height()
        self.qp = QPainter()
        self.pointsAcc = 30
        self.interface_start_point = [50, 50]
        self.circle_center_x = self.interface_start_point[0] + 500
        self.circle_center_y = self.interface_start_point[1] + 500
        self.scrcoord     = np.array([self.circle_center_x, 
                                      self.circle_center_y, 
                                      0])
        self.scrcoordcoef = np.array([1, -1, 1])
        self.center_p = np.array([self.circle_center_x, self.circle_center_y])
        self.rotation_x = 0
        self.rotation_y = 0
        self.rotation_z = 0
        self.sphere = Sphere()
        self.start_time = 0
        self.timestat = 0
        self.timestatglb = 0
        self.timestatnum = 0
        self.initUI()
        np.set_printoptions(15)
        timer = QTimer(self, timeout=self.update_img, interval=15)
        timer.start()
        timer2 = QTimer(self, timeout=self.printstat, interval=1000)
        timer2.start()
        self.show()

    def initUI(self):   
        self.showMaximized()
        self.setStyleSheet("""
        color: white;
        background-color: black;
        font-family: "Comic Sans MS";
        """)
        self.scrollw = QScrollArea()
        self.scrollw.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOn)
        self.scrollw.setStyleSheet("""
        border: 0px solid black;
        """)
        vlay = QVBoxLayout()
        vlay.setContentsMargins(10, 10, 30, 10)
        self.mainBtns = MainBottons(self.restart, self.sphere.setClimateOfContinents)
        self.triangleNetCheckBox = MapCheckBox("Треугольная сетка")
        self.angleSliders = MapHSlider("Поворот угла обзора (X, Y, Z)", [[-360, 360], [-360, 360], [-360, 360]], [0, 0, 0], scale=1)
        self.fieldSettings = FieldSettings(self.sphere.field)
        self.speedSlidere = MapHSlider("Коефф. скорости", [[0, 10000]], [GLOBALS.SPEED], scale=1, globalVarSetter=GLOBALS.speedSetter)
        vlay.addWidget(self.mainBtns)
        vlay.addWidget(self.angleSliders)
        vlay.addWidget(self.fieldSettings)
        vlay.addWidget(self.speedSlidere)
        vlay.addStretch(0)
        groupBox = QGroupBox()
        groupBox.setLayout(vlay)
        self.scrollw.setWidget(groupBox)
        centralLayout = QHBoxLayout()
        centralLayout.setAlignment(Qt.AlignLeft)
        centralLayout.addStretch(0)
        centralLayout.addStretch(0)
        centralLayout.addWidget(self.scrollw)
        self.setLayout(centralLayout)
        self.setWindowTitle('2DMap')
    
    def update_img(self):
        if self.mainBtns.evol:
            self.sphere.evolutionStep()
        self.updateViewAngles()
        self.update()

    def restart(self):
        self.sphere.initContinents()

    def updateViewAngles(self):
        a, b, c = self.angleSliders.values()
        self.rotation_x = 2*math.pi * a/360 
        self.rotation_y = 2*math.pi * b/360 
        self.rotation_z = 2*math.pi * c/360 

    def stat1(self):
        self.start_time = time.time()

    def stat2(self):
        self.timestat += (time.time() - self.start_time)
        self.timestatnum += 1

    def printstat(self):
        if self.timestatnum == 0: return
        print(f"Timestat: {self.timestat/self.timestatnum}")
        self.timestat = 0
        self.timestatnum = 0

    def paintEvent(self, e):
        self.qp.begin(self)
        self.qp.setRenderHints(QPainter.Antialiasing)
        self.qp.setPen(QPen(QColor(0, 0, 0)))
        field_img = self.fieldSettings.combo.getImage()
        if field_img != None:
            self.qp.drawPixmap(int(self.circle_center_x - GLOBALS.CIRCLE_RADIUS), int(self.circle_center_y - GLOBALS.CIRCLE_RADIUS), field_img)
        else:
            a = 0.9
            rad = int(GLOBALS.CIRCLE_RADIUS/a)
            center_x = int(self.circle_center_x) # + int(CIRCLE_RADIUS)
            center_y = int(self.circle_center_y) # + int(CIRCLE_RADIUS)
            radialGradient = QRadialGradient(QPoint(center_x, center_y), rad-3)
            radialGradient.setColorAt(0,   QColor(1, 168, 174))
            radialGradient.setColorAt(0.3, QColor(1, 132, 169))
            radialGradient.setColorAt(a, QColor(1, 83 , 125))
            radialGradient.setColorAt(a+0.05,   QColor(0, 0, 0))
            self.qp.setBrush(QBrush(radialGradient))
            self.qp.drawEllipse(QPoint(self.circle_center_x, self.circle_center_y), rad, rad)
            # self.qp.setPen(QPen(QColor(255, 255, 255)))
            # self.qp.drawEllipse(QPoint(self.circle_center_x, self.circle_center_y), int(GLOBALS.CIRCLE_RADIUS), int(GLOBALS.CIRCLE_RADIUS))
        self.stat1()
        self.drawClimate()
        # if self.mainBtns.drawClimate: self.drawClimate()
        # else:                         self.drawMovement()
        self.stat2()    
        self.qp.end()

    def drawClimate(self):
        self.qp.setPen(QPen(QColor(0, 0, 0), 2, Qt.SolidLine))
        for continent in self.sphere.continents: 
            rotated_points = funcs.allPointsRotationOffset(continent.points, self.rotation_x, self.rotation_y, self.rotation_z)
            for edge in continent.edges:
                edge.interpolatePoints(rotated_points)
            for triangle in continent.triangles:
                self.drawClimateTriangle(triangle, rotated_points)

    def drawClimateTriangle(self, triangle, rotated_points):
        path = QPainterPath()
        start = True
        neg_start = False
        connect = False
        end_arc = False
        connect_point1 = None
        connect_point2 = None
        self.qp.setPen(QPen(QColor(0, 0, 0), 2, Qt.SolidLine))
        if self.mainBtns.drawClimate:
            self.qp.setBrush(QBrush(triangle.color))
        else:
            self.qp.setBrush(QBrush(QColor(0, 120, 0)))
        p0 = self.sphereScreenPoint(triangle.vertices[0].value(rotated_points))
        p1 = self.sphereScreenPoint(triangle.vertices[1].value(rotated_points))
        p2 = self.sphereScreenPoint(triangle.vertices[2].value(rotated_points))
        
        minus_counter = 0
        if p0[2] < 0:
            minus_counter += 1
        if p1[2] < 0:
            minus_counter += 1
        if p2[2] < 0:
            minus_counter += 1

        if minus_counter > 2:
            return
        
        path.moveTo(p0[0], p0[1])
        path.lineTo(p1[0], p1[1])
        path.lineTo(p2[0], p2[1])
        path.lineTo(p0[0], p0[1])
        self.qp.drawPath(path)
        return

    def drawMovement(self):
        self.qp.setPen(QPen(QColor(0, 0, 0), 2, Qt.SolidLine))
        for continent in self.sphere.continents: 
            rotated_points = funcs.allPointsRotationOffset(continent.points, self.rotation_x, self.rotation_y, self.rotation_z)
            for edge in continent.edges:
                edge.interpolatePoints(rotated_points)
            self.drawFigure(continent)
            self.drawEdges(continent)
            self.qp.setBrush(QBrush(QColor(0, 0, 0, 0)))

    def drawEdges(self, continent):
        i = -1
        for edge in continent.edges:
            i += 1
            path = QPainterPath()
            self.qp.setPen(QPen(QColor(0, 0, 0), 2, Qt.SolidLine))
            if edge.border:
                self.qp.setPen(QPen(QColor(255, 0, 0), 2, Qt.SolidLine))
            points = self.sphereScreenPoint(edge.interpolatedPoints)
            rng = range(len(points))
            start = True
            for p in rng:
                point = points[p]
                if point[2] < 0:
                    start = True
                    continue
                if start:
                    path.moveTo(point[0], point[1])
                    start = False
                    continue
                if point[2] >= 0:
                    path.lineTo(point[0], point[1])
            self.qp.drawPath(path)         
            
    def drawFigure(self, continent: Continent):
        borders = continent.border_edges
        path = QPainterPath()
        self.qp.setPen(QPen(QColor(0, 0, 0), 1))
        self.qp.setBrush(QBrush(QColor(0, 120, 0)))
        start = True
        neg_start = False
        connect = False
        connect_point1 = None
        connect_point2 = None
        prev_border = None
        for i in range(len(borders)):
            if i == 0:
                edge = borders[i]
                prev_border = edge
                reversed_direction = False
            else:
                if reversed_direction == False:
                    check_vert: ContinentVertex = prev_border.vertex_2
                else:
                    check_vert: ContinentVertex = prev_border.vertex_1
                edge: ContinentEdge = check_vert.getNextBorderEdge(prev_border)
                if edge == None:
                    break
                prev_border = edge
                if edge.vertex_1 == check_vert:
                    reversed_direction = False
                else:
                    reversed_direction = True
            points = self.sphereScreenPoint(edge.interpolatedPoints)
            if not reversed_direction:
                rng = range(len(points))
            else:
                rng = reversed(range(len(points)))
            for p in rng:
                point = points[p]
                if start:
                    path.moveTo(point[0], point[1])
                    if point[2] < 0:
                        neg_start = True
                        continue
                    start = False
                    if neg_start:
                        connect_point2 = point
                    # self.qp.setPen(QPen(QColor(0, 0, 0), 10))
                    # self.qp.drawPoint(point[0], point[1])
                    # self.qp.setPen(QPen(QColor(0, 0, 0), 1))
                if point[2] < 0:
                    if not connect:
                        connect_point1 = point   
                        connect = True
                    else:
                        continue
                if connect:
                    if point[2] >= 0 and not neg_start:
                        connect_point2 = point
                    if not (connect_point1 is None) and not (connect_point2 is None):
                        if not neg_start:
                            connect = False
                        v1 = connect_point1[0:2] - self.center_p
                        v2 = connect_point2[0:2] - self.center_p
                        v1[1] = v1[1] * -1
                        v2[1] = v2[1] * -1
                        angle1 = math.degrees(funcs.flatAngle(v1))
                        angle2 = math.degrees(funcs.flatAngle(v2))
                        r = int(GLOBALS.CIRCLE_RADIUS)
                        diff = angle2 - angle1
                        if diff < -90:
                            start_angle = angle1
                            alpha = 360 + diff
                        else:
                            start_angle = angle1
                            alpha = diff
                        if diff < 0.001:
                            continue
                        # print(f"angle1: {angle1}, angle2: {angle2}, diff: {diff}, start_angle: {start_angle}, alpha: {alpha}")
                        path.arcTo(self.circle_center_x - r, self.circle_center_y - r, 2 * r, 2 * r, start_angle, alpha)
                        continue
                if point[2] >= 0:
                    path.lineTo(point[0], point[1])
        self.qp.drawPath(path)
    def sphereScreenPoint(self, points):
        return points * self.scrcoordcoef + self.scrcoord

def start():
    app = QApplication(sys.argv)
    ex = Map(app.primaryScreen().size())
    app.exec_()

if __name__ == '__main__':
    start()