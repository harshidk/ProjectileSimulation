import sim
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from tqdm import tqdm
import json

FILE_NAME = 'data.json'
dictionary = {}

def convertKeyStringToTuple(key):
    new_key = tuple(str(key).split(','))
    return new_key

with open(FILE_NAME, 'r') as file:
    dictionary = json.load(file)

keys = list(dictionary.keys())
tuple_keys = []
optimal_states = []

for k in keys:
    if dictionary[k][0][0] != 0.0:
        optimal_states.append(dictionary[k])
        tuple_keys.append(convertKeyStringToTuple(k))

# print(optimal_states)
# sim.meshcatVisualizeMultipleShots(tuple_keys, optimal_states, sim.TARGET_POSE)

def getNumpyArrayPositions():
    array = []
    for key in tuple_keys:
        x = float(key[0])
        y = float (key[1])
        array.append(np.array([x, y]))
    return array

def getOptimalStates():
    return optimal_states