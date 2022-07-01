import numpy as np
SCREEN_WIDTH = 1920
SCREEN_HEIGHT = 1080
CIRCLE_RADIUS = SCREEN_HEIGHT/3
AXIS_X  = np.array([1, 0, 0])
AXIS_Y  = np.array([0, 1, 0])
AXIS_Z  = np.array([0, 0, 1])
SPEED = 100
SPEED_CONVERT_COEF = 1/1000000
def speedSetter(value):
    global SPEED
    if isinstance(value, list): SPEED = value[0]
    else: SPEED = value
SPHERE_AREA = 4 * np.pi * CIRCLE_RADIUS**2
CONTINENTS_AREA_PROCENT = 30
CONTINENT_AREA_CHANGING_VALUE =  (SPEED * SPEED_CONVERT_COEF) # * 2 * np.pi
MAXIMUM_OF_CONTINENTS = 10
NEW_CONTINENT_RADIUS_PROCENT = 0.10
MAXIMUM_AREA_PROCENT_OF_CONTINENT_TRIANGLE = 0.0009
MINIMUM_AREA_PROCENT_OF_CONTINENT_TRIANGLE = 0.0001
CHANGE_POSITION_COEF = 20 / 0.0001 # при 0.0001
BISECT_LVL = 4
INTERPOLATION_ACCURACY = 64 / CIRCLE_RADIUS
GENERATION_SEED = 1000
# 129 - первое время просто проскальзывают, затем сталкиваются
NUMBER_OF_FIELDS = 10 # 15
MAX_FORCE_SUM    = 30000
MIN_FORCE        = 1000
MAX_FORCE        = 2000
RE_FORCE_INTERVAL = 10000 # milisec

def FieldGlobalSetter(values):
    global GENERATION_SEED, NUMBER_OF_FIELDS , MAX_FORCE_SUM    , MIN_FORCE        , MAX_FORCE        , RE_FORCE_INTERVAL 
    GENERATION_SEED   = values[0]
    NUMBER_OF_FIELDS  = values[1]
    MAX_FORCE_SUM     = values[2]
    MIN_FORCE         = values[3]
    MAX_FORCE         = values[4]
    RE_FORCE_INTERVAL = values[5]
    return [GENERATION_SEED, NUMBER_OF_FIELDS , MAX_FORCE_SUM    , MIN_FORCE        , MAX_FORCE        , RE_FORCE_INTERVAL]