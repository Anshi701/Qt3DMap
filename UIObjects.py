from PyQt5.QtWidgets import QSizePolicy, QComboBox, QLineEdit, QStyle, QGroupBox, QAbstractButton, QStyleOption, QApplication, QScrollArea, QWidget, QPushButton, QVBoxLayout, QHBoxLayout, QSlider, QSpacerItem, QCheckBox, QLabel, QDesktopWidget
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPainter
import GLOBALS
import math
import sys

class MyQWidget(QWidget):
    def __init__(self, parent = None, flags = Qt.WindowFlags()) -> None:
        super().__init__(parent, flags)
    
    def paintEvent(self, e) -> None:
        o = QStyleOption()
        o.initFrom(self)
        p = QPainter(self)
        self.style().drawPrimitive(QStyle.PE_Widget, o, p, self)

class MapHSlider(MyQWidget):
    def __init__(self, title, minmaxVals, defaultVals, globalVarSetter = None, scale = 100) -> None:
        super().__init__() # Qt.Horizontal
        self.title = title 
        self.minmaxVals = minmaxVals
        self.defaultVals = defaultVals
        self.scale = scale
        self.globalVarSetter = globalVarSetter
        self.setObjectName('SliderWidget')
        self.setStyleSheet("""
        QWidget#SliderWidget {
            border: 1px solid white;
            border-radius: 10px;
        }

        QLabel {
            font-size: 17pt;
        }
        """)
        hlays = []
        self.sliders = []
        self.indicators = []
        i = 0
        for minmax in minmaxVals:
            slider = QSlider(Qt.Horizontal, self)
            slider.setRange(int(minmax[0] * self.scale), int(minmax[1] * self.scale))
            slider.setValue(self.defaultVals[i])
            slider.valueChanged.connect(self.valueChanged)
            self.sliders.append(slider)

            valueIndicator = QLabel(str(self.defaultVals[i]))
            lbl_w = (max(   len(str(minmax[0] * self.scale)) , len(str(minmax[1] * self.scale))     ) +1)* 17
            valueIndicator.setFixedWidth(lbl_w)
            valueIndicator.setStyleSheet("""
            border: 1px solid white;
            """)
            self.indicators.append(valueIndicator)
            minLbl = QLabel(str(minmax[0]))
            maxLbl = QLabel(str(minmax[1]))
            lay1 = QHBoxLayout()
            lay1.addWidget(valueIndicator)
            lay1.addWidget(minLbl)
            lay1.addWidget(slider)
            lay1.addWidget(maxLbl)
            hlays.append(lay1)
            i+=1
        self.titleLabel = QLabel(self.title)
        self.lay0 = QVBoxLayout()
        self.lay0.addWidget(self.titleLabel)
        for hlay in hlays:
            self.lay0.addLayout(hlay)
        self.setLayout(self.lay0)
        
    def valueChanged(self):
        values = [0] * len(self.sliders)
        for i in range(len(self.sliders)):
            values[i] = self.sliders[i].value()/(self.scale)
            self.indicators[i].setText(str(values[i]))
        if self.globalVarSetter != None:
            self.globalVarSetter(values)
        
    def values(self):
        lst = []
        for i in range(len(self.sliders)):
            value = self.sliders[i].value()/(self.scale)
            lst.append(value)
        return lst

class MapCheckBox(MyQWidget):
    def __init__(self, title) -> None:
        super().__init__()
        self.state = 0
        self.style0 = """
        QCheckBox{
            font-size: 17pt;
        }
        QCheckBox::indicator{
            width: 40px;
            height: 40px;
            background-color: black;
            border-radius: 20px;
            border: 2px solid white;
        }
        QCheckBox::indicator:checked{
            width: 30px;
            height: 30px;
            border: 7px solid green;
            
        }
        QCheckBox::indicator:unchecked{
            border: 2px solid white;
        }
        """
        self.checkbox = QCheckBox(title, self)
        self.checkbox.setStyleSheet(self.style0)
        self.checkbox.stateChanged.connect(self.checkBoxChangedAction)
        mainLay = QHBoxLayout()
        mainLay.addWidget(self.checkbox)
        self.setLayout(mainLay)
        
    def checkBoxChangedAction(self):
        self.state = (self.state + 1)%3
        
class MapForceFieldViewCBox(MyQWidget):
    def __init__(self, field) -> None:
        super().__init__()
        self.field = field
        title = QLabel("Показать: ")
        self.items = {
            "Ничего":            -1,
            "Температура, верх": 0,
            "Температура, низ":  1,
            "Скорость, верх":    2,
            "Скорость, низ":     3,
        }
        self.state = -1
        self.combo = QComboBox()
        for s in self.items.keys():
            self.combo.addItem(s)
        self.combo.activated[str].connect(self.onChanged)
        stl = """
        border: 1px solid white;
        """
        self.combo.setStyleSheet(stl)
        lay = QHBoxLayout()
        lay.addWidget(title)
        lay.addWidget(self.combo)
        self.setLayout(lay)

    def onChanged(self, s):
        self.state = self.items[s]

    def getImage(self):
        if self.state != -1:
            if not self.field.updatedImages:
                self.field.create2DImage()
            images = (
                self.field.temp_pixmap_front  ,
                self.field.temp_pixmap_back   ,
                self.field.speed_pixmap_front ,
                self.field.speed_pixmap_back  
            )
            return images[self.state]
        else:
            return None

class FieldSettings(MyQWidget):
    def __init__(self, field) -> None:
        super().__init__()
        self.field = field
        self.createdField = True
        self.createdImage = True
        self.setObjectName('FieldSettings')
        self.setStyleSheet("""
        QWidget#FieldSettings {
            border: 1px solid white;
            border-radius: 10px;
        }
        QWidget{
            font-size: 22px;
        }
        """)
        self.labels = [
            QLabel("SEED"  ),
            QLabel("Всего магматических точек"  ),
            QLabel("Максимальная сумма всех сил"),
            QLabel("Максимальная сила точки"    ),
            QLabel("Минимальная сила точки"     ),
            QLabel("Интервал генерации (мсек)"  )
        ]
        self.globals = [
            GLOBALS.GENERATION_SEED  ,
            GLOBALS.NUMBER_OF_FIELDS ,
            GLOBALS.MAX_FORCE_SUM    ,
            GLOBALS.MIN_FORCE        ,
            GLOBALS.MAX_FORCE        ,
            GLOBALS.RE_FORCE_INTERVAL
        ]
        max_len = int(max(label.fontMetrics().boundingRect(label.text()).width() * 2.5 for label in self.labels))
        self.lines = []
        self.linestyle1 = """border: 1px solid white;"""
        self.linestyle2 = """border: 1px solid red;"""
        mainLay = QVBoxLayout()
        title = QLabel("Настройка системы магматической активности:")
        mainLay.addWidget(title)

        for i in range(len(self.labels)):
            lbl = self.labels[i]
            lbl.setFixedWidth(max_len)
            line = QLineEdit()
            line.setStyleSheet(self.linestyle1)
            line.setText(str(self.globals[i]))
            line.textChanged.connect(self.changedLineContent)
            self.lines.append(line)
            hlay = QHBoxLayout()
            hlay.addWidget(line)
            hlay.addWidget(lbl)
            mainLay.addLayout(hlay)
        btn_style1 = """
        border: 1px solid white;
        padding: 3px;
        margin: 3px;
        """
        min_h = 40
        button1 = QPushButton("Установить переменные")
        button1.setStyleSheet(btn_style1)
        button1.clicked.connect(self.setGlobals)
        button2 = QPushButton("Сгенерировать поле")
        button2.setStyleSheet(btn_style1)
        button2.clicked.connect(lambda: self.field.createField())
        button1.setMinimumHeight(min_h)
        button2.setMinimumHeight(min_h)
        self.combo = MapForceFieldViewCBox(self.field)
        self.combo.setMinimumHeight(min_h+10)
        mainLay.addWidget(button1)
        mainLay.addWidget(button2)
        mainLay.addSpacing(10)
        self.setLayout(mainLay)

    def changedLineContent(self):
        for i in range(len(self.lines)):
            if self.lines[i].text() != str(self.globals[i]):
                self.lines[i].setStyleSheet(self.linestyle2)
            else:
                self.lines[i].setStyleSheet(self.linestyle1)

    def setGlobals(self):
        values = [int(line.text()) for line in self.lines]
        self.globals = GLOBALS.FieldGlobalSetter(values)
        self.changedLineContent()
        self.createdField = False
        self.createdImage = False

class MovingBotton(MyQWidget):
    def __init__(self, parent_func) -> None:
        super().__init__()
        self.parent_func = parent_func
        self.text1 = "Старт"
        self.text2 = "Стоп"
        self.state = False
        self.style1 = """
        border: 5px solid green;
        border-radius: 10px;
        padding: 10px;
        font-size: 30pt;
        """
        self.style2 = """
        border: 5px solid red;
        border-radius: 10px;
        padding: 10px;
        font-size: 30pt;
        """
        self.button = QPushButton(self.text1)
        self.button.clicked.connect(self.toggle)
        self.button.setStyleSheet(self.style1)
        mainLay = QVBoxLayout()
        mainLay.addWidget(self.button)
        self.setLayout(mainLay)
    
    def toggle(self) -> None:
        self.state = not self.state
        if self.state:
            self.button.setText(self.text2)
            self.button.setStyleSheet(self.style2)
        else:
            self.button.setText(self.text1)
            self.button.setStyleSheet(self.style1)
        self.parent_func()

class ClimateButton(MyQWidget):
    def __init__(self, climate_func) -> None:
        super().__init__()
        self.climate_func = climate_func
        self.text = "Климат"
        self.style1 = """
        border: 3px solid blue;
        border-radius: 10px;
        padding: 10px;
        font-size: 30pt;
        """
        self.button = QPushButton(self.text)
        self.button.clicked.connect(self.onClick)
        self.button.setStyleSheet(self.style1)
        mainLay = QVBoxLayout()
        mainLay.addWidget(self.button)
        self.setLayout(mainLay)
    
    def onClick(self):
        self.climate_func()

class RestartButton(MyQWidget):
    def __init__(self, restart_func) -> None:
        super().__init__()
        self.text = "Рестарт"
        self.style1 = """
        border: 3px solid yellow;
        border-radius: 10px;
        padding: 10px;
        font-size: 20pt;
        """
        self.button = QPushButton(self.text)
        self.button.clicked.connect(restart_func)
        self.button.setStyleSheet(self.style1)
        mainLay = QVBoxLayout()
        mainLay.addWidget(self.button)
        self.setLayout(mainLay)

class MainBottons(MyQWidget):
    def __init__(self, restart_func, climate_func) -> None:
        super().__init__()
        self.evol = False
        self.drawClimate = False
        self.restart_func = restart_func
        self.climate_func = climate_func
        self.moveBtn =       MovingBotton(self.moveClick)
        self.climateBtn =   ClimateButton(self.climateClick)
        self.restartBtn =   RestartButton(self.restartClick)
        self.vlay = QVBoxLayout()
        self.hlay = QHBoxLayout()
        self.hlay.addWidget(self.moveBtn)
        self.hlay.addWidget(self.climateBtn)
        self.vlay.addLayout(self.hlay)
        self.vlay.addWidget(self.restartBtn)
        self.setLayout(self.vlay)

    def moveClick(self):
        self.evol = self.moveBtn.state
        self.drawClimate = False

    def climateClick(self):
        self.evol = False
        if self.moveBtn.state == True:
            self.moveBtn.toggle()
        self.drawClimate = not self.drawClimate
        if self.drawClimate:
            self.climate_func()

    def restartClick(self):
        self.evol = False
        if self.moveBtn.state == True:
            self.moveBtn.toggle()
        self.drawClimate = False
        self.restart_func()

class TestWin(QWidget):
    def __init__(self, size):
        super().__init__()
        self.size = size
        self.initUI()
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
        self.vlay = QVBoxLayout()
        self.vlay.setContentsMargins(10, 10, 30, 10)
        self.field = None
        self.mainBtns = MainBottons(self.restart, self.restart)
        self.triangleNetCheckBox = MapCheckBox("Треугольная сетка")
        self.angleSliders = MapHSlider("Поворот угла обзора (X, Y, Z)", [[-360, 360], [-360, 360], [-360, 360]], [0, 0, 0], scale=1)
        self.fieldSettings = FieldSettings(self.field)
        self.speedSlider = MapHSlider("Коефф. скорости", [[0, 1000]], [GLOBALS.SPEED], scale=1, globalVarSetter=GLOBALS.speedSetter)
        self.vlay.addWidget(self.mainBtns)
        self.vlay.addWidget(self.angleSliders)
        self.vlay.addWidget(self.fieldSettings)
        self.vlay.addWidget(self.speedSlider)
        self.vlay.addStretch()
        groupBox = QGroupBox()
        groupBox.setLayout(self.vlay)
        self.scrollw.setWidget(groupBox)
        self.centralLayout = QHBoxLayout()
        self.centralLayout.setAlignment(Qt.AlignLeft)
        self.centralLayout.addStretch(0)
        self.centralLayout.addStretch(0)
        self.centralLayout.addWidget(self.scrollw)
        self.setLayout(self.centralLayout)
        self.setWindowTitle('Test')
    
    def climate(self):
        print("climate")
    def restart(self):
        print("restart")
    def field_generator(self):
        print("field generator")
    def image_generator(self):
        print("image generator")

def start():
    app = QApplication(sys.argv)
    ex = TestWin(app.primaryScreen().size())
    app.exec_()

if __name__ == '__main__':
    start()