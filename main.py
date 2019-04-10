from bmp import *
from obj import *
from math import *
from collections import namedtuple

# Universidad del Valle de Guatemala
# Grafica por Computadora
# Nombre: Marcos Gutierrez
# Carne: 17909

def renderer():
	renderer = Bitmap(1200,1200)
	#tex = Texture('./modelos/box.bmp')
	renderer.load('./modelos/sphere.obj', mtlFile='./modelos/ReyBoo.mtl', translate=(0.1,-0.5,0), scale=(0.2,0.2,0.2), rotate=(0,0,0), ojo=V3(0.4,-0.2,1), arriba=V3(0,1,0), centro=V3(0,0,0), texture=None)
	renderer.archivo('goku.bmp')

#Imprimimos las funciones realizadas
renderer()
