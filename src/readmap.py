import sim
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from tqdm import tqdm
import json

FILE_NAME = 'src/data.json'
dictionary = {}

def convertKeyStringToTuple(key):
    new_key = tuple(str(key).split(','))
    return new_key

with open(FILE_NAME, 'r') as file:
    dictionary = json.load(file)

keys = list(dictionary.keys())

print(convertKeyStringToTuple(keys[0]))
optimum = dictionary[keys[0]]
print(optimum)

p, v = sim.simulateShotDrag(optimum[0][0], [float(convertKeyStringToTuple(keys[0])[0]), float(convertKeyStringToTuple(keys[0])[1]), optimum[1]], optimum[0][1], 0.5)
sim.meshcatVisualizeShot(p, sim.TARGET_POSE)
