from bitmap import *
from obj import *
# Universidad del Valle de Guatemala
# Grafica por Computadora
# Nombre: Marcos Gutierrez
# Carne: 17909

def reyBoo():
	renderizando = Bitmap(1000,1000)
	glViewPort(0,0,800,800)
	#renderizando.renderer(./modelos/test3.obj, scale=(0,0,0), translate=(0,0,0))
	#renderizando.texture_render('./modelos/reyBoo.obj',(200,200,200),(3,3,3))
	#renderizando.renderer('./modelos/reyBoo.obj', scale=(200,200,200),translate=(3,3,3))
	renderizando.vista(v3(2,0,20), v3(0,0,0), normal(v3(0,1,0)))
	renderizando.glVM(0,0)
	renderizando.loadMatriz('./modelos/reyBoo.obj', scale=(200,200,200), rotate=(4,-4,0), translate=(3,3,3))
	#Resultado, es decir, la imagen de prueba
	renderizando.archivo('transformacion.bmp')

print("Renderizando los modelos obj")

print("Renderizando Modelo de blender")
print(reyBoo())
