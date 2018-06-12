#!/usr/bin/env python

import rospy

servo1_min = -0.95
servo1_max = -0.25
servo1_neutral = -0.45
servo3_min = -2.0
servo3_neutral = -1.68
servo3_max = -1.3
servo4_min = -0.2
servo4_neutral = 0.4
servo4_max = 0.6
servo5_min = -0.4
servo5_neutral = -0.03
servo5_max = 0.33
servo5_nod_min = -0.3
servo5_nod_max = -0.1
servo6_min = -3.5
servo6_neutral = -2.25
servo6_max = -1.70

def str2command (command, i):
    if command == 'blink':
        result = [[servo1_neutral, servo1_min, servo1_max, servo1_neutral, servo1_min, servo1_max],
                  [servo1_neutral, servo1_min, servo1_max, servo1_neutral, servo1_min, servo1_max],
                  [],
                  [],
                  [],
                  []]
    elif command == 'nod':
        result = [[servo1_neutral],
                  [servo1_neutral],
                  [],
                  [],
                  [servo5_neutral, servo5_max],
                  [servo6_neutral]]
    else:
        result = [[],
                  [],
                  [],
                  [],
                  [],
                  []]

    
    return result[i - 1]
        
