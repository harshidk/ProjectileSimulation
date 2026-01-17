import sim
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from tqdm import tqdm
import json

FILE_NAME = 'data.json'

def convertKeyStringToTuple(key):
    new_key = tuple(str(key).split(','))
    return new_key

