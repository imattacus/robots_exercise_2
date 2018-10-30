from scipy import misc


class Map:
    def __init__(self, img_file):
        self.width = 0
        self.height = 0
        self.map = misc.imread(img_file)


def valid_pos(self, x, y):
    return True
