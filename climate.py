import numpy as np
from PyQt5.QtGui import QColor
import random

def InitClimate():
    check_coord = [
    [90 , 0, 90, 180, 270],
    [89 , 0, 90, 180, 270],
    [88 , 0, 90, 180, 270],
    [87 , 0, 90, 180, 270],
    [86 , 0, 90, 180, 270],
    [85 , 0, 90, 180, 270],
    [84 , 0, 90, 180, 270],
    [83 , 40, 70],
    [82 , 40],
    [81 , 40],
    [80 , 40],
    [79 , 40],
    [78 , 40],
    [77 , 110, 100, 80, 70, 60, 44, -107, -100,  40],
    [76 , 110, 100, 80, 70, 60, 44, -107, -100,  40],
    [75 , 110, 100, 80, 70, 60, 44, -107, -100,  40],
    [74 , 110, 100, 80, 70, 60, 44, -107, -100,  40],
    [73 , 110, 100, 80, 70, 60, 44, -107, -100,  40],
    [72 , 110, 100, 80, 70, 60, 44, -107, -100,  40],
    [71 , 110, 100, 80, 70, 60, 44, -107, -100,  40],
    [70 , 110, 100, 80, 70, 60, 44, -107, -100,  40],
    [69 , 110, 100, 80, 70, 60, 44, -107, -100,  40],
    [68 , 110, 100, 80, 70, 60, 44, -107, -100,  40],
    [67 , 110, 100, 80, 70, 60, 44, -107, -100, ],
    [66 , 47, 46, 48, 110, 100, 80, 70, 60, 44, -107, -100, ],
    [65 , 47, 46, 48, 110, 100, 80, 70, 60, 44, -107, -100, ],
    [64 , 47, 46, 48, 110, 100, 80, 70, 60, 44, -107, -100, ],
    [63 , 47, 46, 48, 110, 100, 80, 70, 60, 44, -107, -100, ],
    [62 , 47, 46, 48, 110, 100, 80, 70, 60, 44, -107, -100, ],
    [61 , 47, 46, 48, 110, 100, 80, 70, 60, 44, -107, -100, ],
    [60 , 47, 46, 48, 110, 100, 80, 70, 60, 44, -107, -100, ],
    [59 , 47, 46, 48, 110, 100, 80, 70, 60, 44, -107, -100, ],
    [58 , 47, 46, 48, 110, 100, 80, 70, 60, 44, -107, -100, ],
    [57 , 47, 46, 48, 110, 100, 80, 70, 60, 44, -107, -100, ],
    [56 , 47, 46, 48, 110, 100, 80, 70, 60, 44, -107, -100, ],
    [55 , 47, 46, 48, 110, 100, 80, 70, 60, 44, -107, -100, ],
    [54 , 47, 46, 48, 110, 100, 80, 70, 60, 44, -107, -100, ],
    [53 , 47, 46, 48, 110, 100, 80, 70, 60, 44, -107, -100, ],
    [52 , 47, 46, 48, 110, 100, 80, 70, 60, 44, -107, -100, ],
    [51 , 47, 46, 48, 110, 100, 80, 70, 60, 44, -107, -100, ],
    [50 , 47, 46, 48, 110, 100, 80, 70, 60, 44, -107, -100, ],
    [49 , 47, 46, 48, 110, 100, 80, 70, 60, 44, -107, -100, ],
    [48 , 47, 46, 48, 110, 100, 80, 70, 60, 44, -107, -100, ],
    [47 , 47, 46, 48, 110, 100, 80, 70, 60, 44, -107, -100, ],
    [46 , 47, 46, 48, 110, 100, 80, 70, 60, 44, -107, -100, ],
    [45 , 47, 46, 48, 110, 100, 80, 70, 60, 44, -107, -100, ],
    [44 , 47, 46, 48, 110, 100, 80, 70, 60, 44, -107, -100, ],
    [43 , 47, 46, 48, 110, 100, 80, 70, 60, 44, -107, -100, ],
    [42 , 47, 46, 48, 110, 100, 80, 70, 60, 44, -107, -100, ],
    [41 , 47, 46, 48, 110, 100, 80, 70, 60, 44, -107, -100, ],
    [40 , 47, 46, 48, 110, 100, 80, 70, 60, 44, -107, -100, ],
    [39 , 47, 46, 48, 110, 100, 80, 70, 60, 44, -107, -100, ],
    [38 , 47, 46, 48, 110, 100, 80, 70, 60, 44, -107, -100, ],
    [37 , 47, 46, 48, 110, 100, 80, 70, 60, 44, -107, -100, ],
    [36 , 47, 46, 48, 110, 100, 80, 70, 60, 44, -107, -100, ],
    [35 , 5, 6, 7, 8, 47, 46, 48, 110, 100, 80, 70, 60, 44, -107, -100, ],
    [34 , 5, 6, 7, 8, 47, 46, 48, 110, 100, 80, 70, 60, 44, -107, -100, ],
    [33 , 5, 6, 7, 8, 47, 46, 48, 110, 100, 80, 70, 60, 44, -107, -100, ],
    [32 , 5, 6, 7, 8, 47, 46, 48, 110, 100, 80, 70, 60, 44, -107, -100, ],
    [31 , 5, 6, 7, 8, 47, 46, 48, 110, 100, 80, 70, 60, 44, -107, -100, ],
    [30 , 20, 22, 25, 27, 30, 5, 6, 7, 8, 47, 46, 48, 110, 100, 80, 70, 60, 44, -107, -100, ],
    [29 , 20, 22, 25, 27, 30, 5, 6, 7, 8, 47, 46, 48, 110, 100, 80, 70, 60, 44, -107, -100, ],
    [28 , 20, 22, 25, 27, 30, 5, 6, 7, 8, 47, 46, 48, 110, 100, 80, 70, 60, 44, -107, -100, ],
    [27 , 20, 22, 25, 27, 30, 5, 6, 7, 8, 47, 46, 48, 110, 100, 80, 70, 60, 44, -107, -100, ],
    [26 , 20, 22, 25, 27, 30, 5, 6, 7, 8, 47, 46, 48, 110, 100, 80, 70, 60, 44, -107, -100, ],
    [25 , 20, 22, 25, 27, 30, 5, 6, 7, 8, 47, 46, 48, 110, 100, 80, 70, 60, 44, -107, -100, ],
    [24 , 20, 22, 25, 27, 30, 5, 6, 7, 8, 47, 46, 48, 110, 100, 80, 70, 60, 44, -107, -100, ],
    [23 , 20, 22, 25, 27, 30, 5, 6, 7, 8, 47, 46, 48, 110, 100, 80, 70, 60, 44, -107, -100, ],
    [22 , 20, 22, 25, 27, 30, 5, 6, 7, 8, 47, 46, 48, 110, 100, 80, 70, 60, 44, -107, -100, ],
    [21 , 20, 22, 25, 27, 30, 5, 6, 7, 8, 47, 46, 48,                                 -100, ],
    [20 , 20, 22, 25, 27, 30, 5, 6, 7, 8, 47, 46, 48,                                 -100, ],
    [19 , 20, 22, 25, 27, 30, 5, 6, 7, 8, 47, 46, 48,                                 -100, ],
    [18 , 20, 22, 25, 27, 30, 5, 6, 7, 8, 47, 46, 48,                                 -100, ],
    [17 , 20, 22, 25, 27, 30, 5, 6, 7, 8, 47, 46, 48,],
    [16 , 20, 22, 25, 27, 30, 5, 6, 7, 8, 47, 46, 48,],
    [15 , 20, 22, 25, 27, 30, 5, 6, 7, 8, 47, 46, 48,],
    [14 , 20, 22, 25, 27, 30, 5, 6, 7, 8,],
    [13 , 20, 22, 25, 27, 30, 5, 6, 7, 8,],
    [12 , 20, 22, 25, 27, 30, 5, 6, 7, 8,],
    [11 , 20, 22, 25, 27, 30, 5, 6, 7, 8,],
    [10 ,  -70, -69, -68, 20, 22, 25, 27, 30, 5, 6, 7, 8,],
    [9  ,  -70, -69, -68, 20, 22, 25, 27, 30, 5, 6, 7, 8,],
    [8  ,  -70, -69, -68, 20, 22, 25, 27, 30, 5, 6, 7, 8,],
    [7  ,  -70, -69, -68, 20, 22, 25, 27, 30, 5, 6, 7, 8,],
    [6  ,  -70, -69, -68, 20, 22, 25, 27, 30, 5, 6, 7, 8,],
    [5  ,  -60, -61, -63, -70, -69, -68, 20, 22, 25, 27, 30 ],
    [4  ,  -60, -61, -63, -70, -69, -68, 20, 22, 25, 27, 30 ],
    [3  ,  -60, -61, -63, -70, -69, -68, 20, 22, 25, 27, 30 ],
    [2  ,  -60, -61, -63, -70, -69, -68, 20, 22, 25, 27, 30 ],
    [1  ,  -60, -61, -63, -70, -69, -68, 20, 22, 25, 27, 30 ],
    [0  ,  -60, -61, -63, -70, -69, -68, 20, 22, 25, 27, 30 ],
    [-1 ,  -60, -61, -63, -70, -69, -68, 20, 22, 25, 27, 30],
    [-2 ,  -60, -61, -63, -70, -69, -68, 20, 22, 25, 27, 30],
    [-3 ,  -60, -61, -63, -70, -69, -68, 20, 22, 25, 27, 30],
    [-4 ,  -60, -61, -63, -70, -69, -68, 20, 22, 25, 27, 30],
    [-5 ,  -60, -61, -63, -70, -69, -68, 20, 22, 25, 27, 30],
    [-6 ,  -60, -61, -63, -70, -69, -68, 20, 22, 25, 27, 30],
    [-7 ,  -60, -61, -63, -70, -69, -68, 20, 22, 25, 27, 30],
    [-8 ,  -60, -61, -63, -70, -69, -68, 20, 22, 25, 27, 30],
    [-9 ,  -60, -61, -63, -70, -69, -68, 20, 22, 25, 27, 30],
    [-10,  -60, -61, -63, -70, -69, -68, 20, 22, 25, 27, 30],
    [-11,  -60, -61, -63, -70, -69, -68, 20, 22, 25, 27, 30],
    [-12,  -60, -61, -63, -70, -69, -68, 20, 22, 25, 27, 30],
    [-13,  -60, -61, -63, -70, -69, -68, 20, 22, 25, 27, 30],
    [-14,  -60, -61, -63, -70, -69, -68, 20, 22, 25, 27, 30],
    [-15, 145, 130, 135, -60, -61, -63, -70, -69, -68, 20, 22, 25, 27, 30],
    [-16, 145, 130, 135, -60, -61, -63, -70, -69, -68, 20, 22, 25, 27, 30],
    [-17, 145, 130, 135, -60, -61, -63, -70, -69, -68, 20, 22, 25, 27, 30],
    [-18, 145, 130, 135, -60, -61, -63, -70, -69, -68, 20, 22, 25, 27, 30],
    [-19, 145, 130, 135, -60, -61, -63, -70, -69, -68, 20, 22, 25, 27, 30],
    [-20, 145, 130, 135, -60, -61, -63, -70, -69, -68, 20, 22, 25, 27, 30],
    [-21, 145, 130, 135, -60, -61, -63, -70, -69, -68, 20, 22, 25, 27, 30],
    [-22, 145, 130, 135, -60, -61, -63, -70, -69, -68, 20, 22, 25, 27, 30],
    [-23, 145, 130, 135, -60, -61, -63, -70, -69, -68, 20, 22, 25, 27, 30],
    [-24, 145, 130, 135, -60, -61, -63, -70, -69, -68, 20, 22, 25, 27, 30],
    [-25, 145, 130, 135, -60, -61, -63, -70, -69, -68, 20, 22, 25, 27, 30],
    [-26, 145, 130, 135, -60, -61, -63, -70, -69, -68, 20, 22, 25, 27, 30],
    [-27, 145, 130, 135, -60, -61, -63, -70, -69, -68, 20, 22, 25, 27, 30],
    [-28, 145, 130, 135, -60, -61, -63, -70, -69, -68, 20, 22, 25, 27, 30],
    [-29, 145, 130, 135, -60, -61, -63, -70, -69, -68, 20, 22, 25, 27, 30],
    [-30, 145, 130, 135, -60, -61, -63, -70, -69, -68, 20, 22, 25, 27, 30],
    [-31, 145,  -60, -61, -63, -70, -69, -68,],
    [-32, 145,  -60, -61, -63, -70, -69, -68,],
    [-33, 145,  -60, -61, -63, -70, -69, -68,],
    [-34, 145,  -60, -61, -63, -70, -69, -68,],
    [-35, 145,  -60, -61, -63, -70, -69, -68,],
    [-36,  -70, -69, -68,],
    [-37,  -70, -69, -68,],
    [-38,  -70, -69, -68,],
    [-39,  -70, -69, -68,],
    [-40,  -70, -69, -68,],
    [-41,  -70, -69, -68,],
    [-42,  -70, -69, -68,],
    [-43,  -70, -69, -68,],
    [-44,  -70, -69, -68,],
    [-45,  -70, -69, -68,],
    [-46,  -70, -69, -68,],
    [-47,  -70, -69, -68,],
    [-48,  -70, -69, -68,],
    [-49,  -70, -69, -68,],
    [-50,  -70, -69, -68,],
    [-51,  -70, -69, -68,],
    [-52,  -70, -69, -68,],
    [-53,  -70, -69, -68,],
    [-54,  -70, -69, -68,],
    [-55,  -70, -69, -68,],
    [-56, ], [-57, ], [-58, ], [-59, ], [-60, ], [-61, ], [-62, ], [-63, ], [-64, ], [-65, ], [-66, ], [-67, ], [-68, ], [-69, ], [-70, ], [-71, ], [-72, ], [-73, ], [-74, ], [-75, ], [-76, ], [-77, ], [-78, ], [-79, ], [-80, ], [-81, ], [-82, ], [-83, ], [-84, ], [-85, ], [-86, ], [-87, ], [-88, ], [-89, ], [-90, ],
    ]
    air_temp_file_path = "DATA/air_temp.txt"
    precip_file_path = "DATA/precip.txt"
    Latitudes = []
    climate_data = []
    with open(air_temp_file_path, "rb") as air_temp_file, open(precip_file_path, "rb") as precip_file:
        n = 0
        while True:
            if n % 100000 == 0: print(n)
            n += 1
            air_temp_data = air_temp_file.readline().decode()
            precip_data   = precip_file.readline().decode()
            if air_temp_data == '':
                break
            long = float(air_temp_data[0:8]) - int(float(air_temp_data[0:8]))
            lat = float(air_temp_data[8:16]) - int(float(air_temp_data[8:16]))
            if long == 0.750 or long == -0.750:
                continue
            if lat  == 0.750 or lat == -0.750:
                continue
            Longitude = int(float(air_temp_data[0:8]))
            Latitude  = int(float(air_temp_data[8:16]))
            for r in check_coord:
                if Latitude ==  r[0]:
                    if len(r) > 1 and Longitude not in r[1:]:
                        continue
                    row = [Latitude, 1]
                    for i in range(12):
                        temp = float(air_temp_data[17 + i*7:  18 + i*7 + 6]) # / 360 / 4
                        row.append(temp)
                    for i in range(12):
                        precip = float(precip_data[17 + i*7:  18 + i*7 + 6]) # / 360 / 4
                        row.append(precip)
                    if Latitude in Latitudes:
                        j = Latitudes.index(Latitude)
                        for m in range(1, len(row)):
                            climate_data[j][m] += row[m]
                    else:
                        Latitudes.append(Latitude)
                        climate_data.append(row)

    CLIMATE_DATA = np.array(climate_data)
    for i in range(CLIMATE_DATA.shape[0]):
        CLIMATE_DATA[i, 2:] = CLIMATE_DATA[i, 2:] / CLIMATE_DATA[i, 1]
    CLIMATE_DATA = np.delete(CLIMATE_DATA, 1, 1)
    CLIMATE_DATA = CLIMATE_DATA[CLIMATE_DATA[:, 0].argsort()]
    return CLIMATE_DATA

CLIMATE_DATA = InitClimate()
import sys
np.set_printoptions(threshold=sys.maxsize, suppress=True, linewidth=150)
climate_types = {
    "Af"    : QColor(19, 0, 252), # 0
    "Am"    : QColor(14, 115, 252), # 1
    "Aw/As" : QColor(54, 154, 229), # 2
    "BWh"   : QColor(253, 0, 0), # 3
    "BWk"   : QColor(254, 149, 148), # 4
    "BSh"   : QColor(246, 162, 0), # 5
    "BSk"   : QColor(230, 198, 90), # 6
    "Cfa"   : QColor(197, 254, 75), # 7
    "Cfb"   : QColor(99, 253, 50), # 8
    "Cfc"   : QColor(53, 198, 0), # 9
    "Cwa"   : QColor(148, 254, 151), # 10
    "Cwb"   : QColor(95, 199, 101), # 11
    "Cwc"   : QColor(54, 150, 51), # 12
    "Csa"   : QColor(252, 254, 4), # 13
    "Csb"   : QColor(206, 204, 0), # 14
    "Csc"   : QColor(143, 143, 0), # 15
    "Dfa"   : QColor(0, 252, 253), # 16
    "Dfb"   : QColor(61, 198, 250), # 17
    "Dfc"   : QColor(0, 126, 126), # 18
    "Dfd"   : QColor(0, 79, 96), # 19
    "Dwa"   : QColor(165, 175, 255), # 20
    "Dwb"   : QColor(74, 120, 227), # 21
    "Dwc"   : QColor(72, 78, 180), # 22
    "Dwd"   : QColor(48, 0, 138), # 23
    "Dsa"   : QColor(252, 0, 251), # 24
    "Dsb"   : QColor(201, 0, 196), # 25
    "Dsc"   : QColor(152, 51, 150), # 26
    "Dsd"   : QColor(142, 93, 146), # 27
    "ET"    : QColor(158, 158, 158), # 28
    "EF"    : QColor(95, 95, 95), # 29
}
def check_A(data):
    all_months_temp_above_18 = True
    for temp in data[1:13]:
        if temp < 18: all_months_temp_above_18 = False
    if all_months_temp_above_18:
        all_months_prec_above_60 = True
        min_prec = 100000
        for prec in data[13:26]:
            if prec < min_prec: 
                min_prec = prec
            if prec < 60: all_months_prec_above_60 = False
        if all_months_prec_above_60:
            return "Af"
        if min_prec < 60 and min_prec >= (100 - np.sum(data[13:26])/25):
            return "Am"
        if min_prec < 60 and min_prec < (100 - np.sum(data[13:26])/25):
            return "Aw/As"
    return None

def check_B(data):
    all_months_temp_less_10 = True
    for temp in data[1:13]:
        if temp > 10: all_months_temp_less_10 = False
    if not all_months_temp_less_10:
        av_temp_for_year = np.sum(data[1:13])/12
        north_hemisphere = True
        if data[0] < 0: north_hemisphere = False
        add_part = 0
        all_months_sum = np.sum(data[13:25])
        if north_hemisphere:
            months_sum = np.sum(data[15:21])
        else:
            months_sum = np.sum(data[21:25]) + np.sum(data[13:15])
        procent = months_sum/all_months_sum * 100
        if procent >= 70: add_part = 280
        elif procent < 70 and procent > 30: add_part = 140
        elif procent < 30: add_part = 0
        treshold = av_temp_for_year * 20 + add_part
        if all_months_sum < treshold/2:
            # BW
            coolest_month_temp = 10000
            for temp in data[1:13]:
                if temp < coolest_month_temp: coolest_month_temp = temp
            if coolest_month_temp > 0:
                return "BWh"
            else:
                return "BWk"
        elif all_months_sum >= treshold/2 and all_months_sum <= treshold:
            # BS
            coolest_month_temp = 10000
            for temp in data[1:13]:
                if temp < coolest_month_temp: coolest_month_temp = temp
            if coolest_month_temp > 0:
                return "BSh"
            else:
                return "BSk"
    return None

def check_C(data):
    most_cold__month_temp = 10000000 # _betw_0_18 
    any_month_temp_above_10 = False
    for temp in data[1:13]:
        if temp < most_cold__month_temp: most_cold__month_temp = temp
        if temp > 10: any_month_temp_above_10 = True
    if most_cold__month_temp > 0 and most_cold__month_temp < 18 and any_month_temp_above_10:
        any_month_temp_above_22 = False
        months_have_temp_above_10 = 0
        for temp in data[1:13]:
            if temp > 10: months_have_temp_above_10 += 1
            if temp > 22: 
                any_month_temp_above_22 = True
        less_wet_month_summer = 10000000
        most_wet_month_summer = 0
        less_wet_month_winter = 10000000
        most_wet_month_winter = 0
        summer, winter = getSummerWinter(data)
        for s in summer[1]:
            if s < less_wet_month_summer: less_wet_month_summer = s
            if s > most_wet_month_summer: most_wet_month_summer = s
        for w in winter[1]:
            if w < less_wet_month_winter: less_wet_month_winter = w
            if w > most_wet_month_winter: most_wet_month_winter = w
        # Cw  
        if most_wet_month_summer > 10 * less_wet_month_winter:
            if months_have_temp_above_10 >= 4:
                if any_month_temp_above_22:
                    return "Cwa"
                if not any_month_temp_above_22:
                    return "Cwb"
            if months_have_temp_above_10 >= 1 and months_have_temp_above_10 <= 3:
                return "Cwc"
        # Cs
        elif most_wet_month_winter > 3 * less_wet_month_summer and less_wet_month_summer <= 40:
            if months_have_temp_above_10 >= 4:
                if any_month_temp_above_22:
                    return "Csa"
                if not any_month_temp_above_22:
                    return "Csb"
            if months_have_temp_above_10 >= 1 and months_have_temp_above_10 <= 3:
                return "Csc"
        # Cf
        else:
            if months_have_temp_above_10 >= 4:
                if any_month_temp_above_22:
                    return "Cfa"
                if not any_month_temp_above_22:
                    return "Cfb"
            if months_have_temp_above_10 >= 1 and months_have_temp_above_10 <= 3:
                return "Cfc"
        
def check_D(data):
    most_cold__month_temp = 10000000 # _betw_0_18 
    any_month_temp_above_10 = False
    for temp in data[1:13]:
        if temp < most_cold__month_temp: most_cold__month_temp = temp
        if temp > 10: any_month_temp_above_10 = True
    if most_cold__month_temp < 0 and any_month_temp_above_10:
        any_month_temp_above_22 = False
        months_have_temp_above_10 = 0
        for temp in data[1:13]:
            if temp > 10: months_have_temp_above_10 += 1
            if temp > 22: 
                any_month_temp_above_22 = True
        less_wet_month_summer = 10000000
        most_wet_month_summer = 0
        less_wet_month_winter = 10000000
        most_wet_month_winter = 0
        summer, winter = getSummerWinter(data)
        for s in summer[1]:
            if s < less_wet_month_summer: less_wet_month_summer = s
            if s > most_wet_month_summer: most_wet_month_summer = s
        for w in winter[1]:
            if w < less_wet_month_winter: less_wet_month_winter = w
            if w > most_wet_month_winter: most_wet_month_winter = w
        # Dw
        if most_wet_month_summer > 10 * less_wet_month_winter: 
            if months_have_temp_above_10 >= 4:
                if any_month_temp_above_22:
                    return "Dwa"
                if not any_month_temp_above_22:
                    return "Dwb"
            if months_have_temp_above_10 >= 1 and months_have_temp_above_10 <= 3:
                if most_cold__month_temp < -38:
                    return "Dwd"
                else:
                    return "Dwc"
        # Ds
        elif most_wet_month_winter > 3 * less_wet_month_summer and less_wet_month_summer <= 40:
            if months_have_temp_above_10 >= 4:
                if any_month_temp_above_22:
                    return "Dsa"
                if not any_month_temp_above_22:
                    return "Dsb"
            if months_have_temp_above_10 >= 1 and months_have_temp_above_10 <= 3:
                if most_cold__month_temp < -38:
                    return "Dsd"
                else:
                    return "Dsc"
        # Df
        else:
            if months_have_temp_above_10 >= 4:
                if any_month_temp_above_22:
                    return "Dfa"
                if not any_month_temp_above_22:
                    return "Dfb"
            if months_have_temp_above_10 >= 1 and months_have_temp_above_10 <= 3:
                if most_cold__month_temp < -38:
                    return "Dfd"
                else:
                    return "Dfc"

def check_E(data):
    every_month_temp_below_10 = True
    any_month_temp_above_0 = False
    for temp in data[1:13]:
        if temp > 0: any_month_temp_above_0 = True
        if temp > 10: every_month_temp_below_10 = False
    if every_month_temp_below_10:
        if any_month_temp_above_0:
            return "ET"
        else:
            return "EF"

def getSummerWinter(data):
    north_hemisphere = True
    if data[0] < 0: north_hemisphere = False
    summer = []
    winter = []
    if north_hemisphere:
        summer.append(data[6:9])
        summer.append(data[18:21])
        winter.append(  np.concatenate([  data[11:13], [data[1]]   ])  )
        winter.append(  np.concatenate([  data[23:25], [data[13]]  ])  )
    else:
        summer.append(  np.concatenate([  data[11:13], [data[1]]   ])  )
        summer.append(  np.concatenate([  data[23:25], [data[13]]  ])  )
        winter.append(data[6:9])
        winter.append(data[18:21])
    return [summer, winter]

def climateType(data):
    E = check_E(data)
    A = check_A(data)
    B = check_B(data)
    C = check_C(data)
    D = check_D(data)
    types = [A, B, C, D, E]
    answer = []
    for t in types:
        if t != None:
            answer.append(t)   
    if len(answer) > 1:
        ans = random.choice(answer)
    else:
        ans = answer[0]
    return ans