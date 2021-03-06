from scipy import ndimage
import numpy as np
import os

class Map:
    def __init__(self, img_file):
        self.map = ndimage.imread(os.path.expanduser(img_file))
        self.width = self.map.shape[0]
        self.height = self.map.shape[1]

    def valid_pos(self, x, y):
        return True
