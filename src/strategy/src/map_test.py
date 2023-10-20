#!/usr/bin/env python3
from curses import meta

import sys

import cv2
import rospy
import numpy as np
import tf
from PIL import Image

# Abra a imagem bitmap
imagem_limited = Image.open("/home/jarvis/open_workspace/src/open_navigation/src/maps/maze_limited.png")
imagem_unlimited = Image.open("/home/jarvis/open_workspace/src/open_navigation/src/maps/maze.png")

# Obtenha as dimensões da imagem
largural, altural = imagem_limited.size
larguraul, alturaul = imagem_unlimited.size

# Crie uma lista para armazenar as tuplas de inteiros de 8 bits
map_limited = []
map_unlimited = []

# Itere sobre os pixels da imagem
for y in range(altural):
    for x in range(largural):
        # Obtenha a cor do pixel (r, g, b)
        r, g, b = imagem_limited.getpixel((x, y))

        # Converta os valores de cor para 8 bits (0-255)
        r = min(255, max(0, int(r * 255)))
        g = min(255, max(0, int(g * 255)))
        b = min(255, max(0, int(b * 255)))

        # Crie a tupla de inteiros de 8 bits (r, g, b)
        tupla_int_8 = (r, g, b)

        # Adicione a tupla à lista
        map_limited.append(tupla_int_8)

for y in range(altural):
    for x in range(largural):
        # Obtenha a cor do pixel (r, g, b)
        r, g, b = imagem_unlimited.getpixel((x, y))

        # Converta os valores de cor para 8 bits (0-255)
        r = min(255, max(0, int(r * 255)))
        g = min(255, max(0, int(g * 255)))
        b = min(255, max(0, int(b * 255)))

        # Crie a tupla de inteiros de 8 bits (r, g, b)
        tupla_int_82 = (r, g, b)

        # Adicione a tupla à lista
        map_unlimited.append(tupla_int_82)

# O vetor_int_8 agora contém as tuplas de inteiros de 8 bits representando a imagem
print(map_limited)
print(map_unlimited)

#map_limited = cv2.imread("/home/jarvis/open_workspace/src/open_navigation/src/maps/maze_limited.png", cv2.IMREAD_GRAYSCALE).astype(np.int8)
#map_unlimited = cv2.imread("/home/jarvis/open_workspace/src/open_navigation/src/maps/maze.png", cv2.IMREAD_GRAYSCALE).astype(np.int8)

#map_limited[map_limited == 0] = 100
#map_limited[map_limited == -1] = 0
#map_unlimited[map_unlimited == 0] = 100
#map_unlimited[map_unlimited == -1] = 0

#print(map_limited)
#print(map_unlimited)

if __name__ == '__main__':
    rospy.init_node('map_test', anonymous=False)
    rospy.loginfo("node started")
    
    rospy.spin()