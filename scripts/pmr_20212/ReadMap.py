import numpy
from PIL import Image

class ReadMap():
    def __init__(self,image_path):
        image = Image.open(image_path, "r")
        # width, height = image.size
        pixels_values = list(image.getdata())
        pixels_width = image.width
        pixels_height = image.height
        Matriz = []
        for i in range(0,len(pixels_values),pixels_height):  
            coluna = []
            for j in range(0,pixels_height):
                P = pixels_values[i+j]
                coluna.append(P)
            Matriz.append(coluna)

        self.pixels = Matriz

        # self.scale_x = 205/pixels_width
        # x_metros = x_pixels*self.scale_x

    def get_pixels(self):
        return self.pixels
