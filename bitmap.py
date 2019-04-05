# Universidad del Valle de Guatemala
# Grafica por Computadora
# Nombre: Marcos Gutierrez
# Carne: 17909

import struct
import math
from obj import *
#Importamos la coleccion
from collections import namedtuple

#Variables globales
ViewPort_X = None
ViewPort_Y = None
ViewPort_H = None
ViewPort_W = None

v2 = namedtuple('Punto2', ['x', 'y'])
v3 = namedtuple('Punto3', ['x', 'y', 'z'])


def char(c):
	return struct.pack("=c",c.encode('ascii'))

def word(c):
	return struct.pack("=h",c)

def dword(c):
	return struct.pack("=l",c)

def color(r,g,b):
	return bytes([b,g,r])

#------ FUNCIONES PARA TRABJAR CON VECTORES DE LONGITUD 3 ------#
#Suma de vectores
def suma(v0,v1):
	#Puntos en cada coordenadas
	px = v0.x + v1.x
	py = v0.y + v1.y
	pz = v0.z + v1.z

	#retorna un vector nuevo con la suma
	return v3(px,py,pz)

#Resta de coordeadas
def resta(v0,v1):
	#Puntos en cada coordenadas
	px = v0.x - v1.x
	py = v0.y - v1.y
	pz = v0.z - v1.z

	#retorna un vector nuevo con la resta
	return v3(px,py,pz)

def mul(v0,k):
	#Puntos en cada coordenadas
	px = v0.x * k
	py = v0.y * k
	pz = v0.z * k

	#retorna un vector nuevo con la multiplicacion a un escalar
	return v3(px,py,pz)

def dot(v0, v1):
  return v0.x * v1.x + v0.y * v1.y + v0.z * v1.z

#Funcion que encontrar un vector nuevo, utlizando algebra producto cruz
def pCruz(v0,v1):
	#Puntos en cada coordenadas
	p1 = v0.y * v1.z - v0.z * v1.y
	p2 = v0.z * v1.x - v0.x * v1.z
	p3 = v0.x * v1.y - v0.y * v1.x

	#Retorna un nuevo vector
	return v3(p1,p2,p3)

#Funcion para la longitud del vector
def longitud(v0):
	px = v0.x ** 2
	py = v0.y ** 2
	pz = v0.z ** 2
	#Suma de puntos
	len = (px+py+pz)**0.5
	return len

#Funcion para encontrar el vector normalr
def normal(v0):
	v0Lon = longitud(v0)

	if not v0Lon:
		return v3(0,0,0)

	px = v0.x/v0Lon
	py = v0.y/v0Lon
	pz = v0.z/v0Lon

	return v3(px,py,pz)

#Bounding Box
def bbox(*vertices):
	xs = [vertex.x for vertex in vertices]
	ys = [vertex.y for vertex in vertices]
	xs.sort()
	ys.sort()

	p1 = xs[0], ys[0]
	p2 = xs[-1], ys[-1]
	return v2(xs[0], ys[0]), v2(xs[-1], ys[-1])

#Funcion para encontrar las coordenadas barycentricas
def baricentricas(A,B,C,P):
	bcoor = pCruz(v3(C.x - A.x, B.x - A.x, A.x - P.x), v3(C.y - A.y, B.y - A.y, A.y - P.y))

	if abs(bcoor.z) < 1:
		return(-1,-1,-1)

	return (1 - (bcoor.x + bcoor.y) / bcoor.z, bcoor.y / bcoor.z, bcoor.x / bcoor.z)

#------ FUNCIONES PARA TRABJAR CON MATRICES ------#
# matriz uno = m1
# matriz dos = m2
def multiplicarMatrices(m1,m2):
	#Condicion para multiplicar matrices es que el numero de columnas de una matriz debe ser el mismo que el numero de filas en la otra matriz
	#Las matrices deben de tener la misma longitud (2x2 * 2x2).... (4x4 * 4x4)
	#Basado en: https://www.geeksforgeeks.org/c-program-multiply-two-matrices/
	"""
		MATRIZ 1			MATRIZ 2
	[ 0, 0 , 0 , 0 ]	[ 0, 0 , 0 , 0 ]
	[ 0, 0 , 0 , 0 ]	[ 0, 0 , 0 , 0 ]
	[ 0, 0 , 0 , 0 ]	[ 0, 0 , 0 , 0 ]
	[ 0, 0 , 0 , 0 ]	[ 0, 0 , 0 , 0 ]
	"""
	columnas_M1 = len(m1[0])
	filas_M2 = len(m2)
	#Condicon para crear la MATRIZ
	if columnas_M1 == filas_M2:
		#Creamos la matriz resultante de la multiplicacion
		matrizResultado = [[0] * len(m2[0]) for i in range(len(m1))]
		#print(matrizResultado)
		for x in range(len(m1)):
			for y in range(len(m2[0])):
				for z in range(len(m1[0])):
					matrizResultado[x][y] += m1[x][z] * m2[z][y]
		#retornamos la matriz resultante
		return matrizResultado
	else:
		print('Las matrices no cumplen con la condiciones para realizar una multiplicacion')
		return 0

#------ FUNCIONES PARA TRABAJAR TIPO OPENGL ------#
#Funcion definara el area de la imagen
def glViewPort(x,y,width,height):
	global ViewPort_X, ViewPort_Y, ViewPort_H, ViewPort_W
	#variable View Port en el eje x
	ViewPort_X = x
	#variable View Port en el eje y
	ViewPort_Y = y
	#variable View Port del ancho de la imagen
	ViewPort_H = height
	#variable View Port de la altura
	ViewPort_W = width

#Funcion que limpiara
def glClear():
	global windows
	#Llamo la funcion clear que se encuentra en la clase bitmap
	windows.clear()

#Funcion que limpiara el color
def glClearColor(r,g,b):
	global windows
	#windows.clear(int(255*r),int(255*g),int(255*b))

	#Se realiza la multiplicacion para llevar a otro color, FUNCION FLOOR, para aproximar al numero mas pequeño
	R = int(math.floor(r * 255))
	G = int(math.floor(g * 255))
	B = int(math.floor(b * 255))
	#Devuelve los numeros para crear otro color, es decir limpiar el color.
	#print("Limpieza de color: %d, %d, %d" % (R,G,B))
	windows.clearColor = color(R,G,B)

#Funcion que cambie el color de un punto.
def glVertex(x,y):
	global ViewPort_X, ViewPort_Y, ViewPort_H, ViewPort_W, windows

	PortX = int((x+1) * ViewPort_W * (1/2) + ViewPort_X)
	PortY = int((y+1) * ViewPort_H * (1/2) + ViewPort_Y)
	#print('glVertex X: %d y %d' % (PortX,PortY))
	windows.point(PortX,PortY)

#Funcion para cambiar el color de la funcion glVertex
def glColor(r,g,b):
	global windows
	#Se realiza la multiplicacion para llevar a otro color, FUNCION FLOOR, para aproximar al numero mas pequeño
	R = int(math.floor(r * 255))
	G = int(math.floor(g * 255))
	B = int(math.floor(b * 255))
	#Devuelve los numeros para crear otro color, es decir limpiar el color.
	#print("Vertex Color: %d, %d, %d" % (R,G,B))
	windows.vertexColor = color(R,G,B)

#Transformar datos a normal
def nor(n):
	global ViewPort_X, ViewPort_Y, ViewPort_H, ViewPort_W, windows
	nx = int(ViewPort_H * (n[0]+1) * (1/2) + ViewPort_X)
	ny = int(ViewPort_H * (n[1]+1) * (1/2) + ViewPort_Y)
	return nx, ny


Negro = color(0,0,0)
Blanco = color(255,255,255)
#CLASE QUE GENERA ESCRITORIO DE IMAGEN
class Bitmap(object):
	#constructor de la clase
	def __init__(self, width,height):
		self.width = width
		self.height = height
		self.framebuffer = []
		self.clearColor = Blanco
		self.vertexColor = Blanco
		#self.mPipeline = None
		self.clear()

	def clear(self):
		self.framebuffer = [[Negro for x  in range(self.width)] for y in range(self.height)]
		self.zbuffer = [[-float('inf') for x in range(self.width)] for y in range(self.height)]

	def write(self,filename="out.bmp"):
		f = open(filename,'bw')
		#file header (14)
		f.write(char('B'))
		f.write(char('M'))
		f.write(dword(14 + 40 + self.width * self.height * 3))
		f.write(dword(0))
		f.write(dword(14 + 40))

		#image header (40)
		f.write(dword(40))
		f.write(dword(self.width))
		f.write(dword(self.height))
		f.write(word(1))
		f.write(word(24))
		f.write(dword(0))
		f.write(dword(0))
		f.write(dword(self.width * self.height * 3))
		f.write(dword(0))
		f.write(dword(0))
		f.write(dword(0))
		f.write(dword(0))

		for x in range(self.height):
			for y in range(self.width):
				f.write(self.framebuffer[x][y])
		f.close()

	def archivo(self, filename='out.bmp'):
		self.write(filename)

	def point(self, x, y,color=None):
		self.framebuffer[y][x]= color or self.vertexColor

	#Funcion para la matriz
	def glVM(self,x,y):
		self.ViewPortMatriz(x,y)

	def glLine(self, vertex1, vertex2):
		x1 = vertex1[0]
		y1 = vertex1[1]
		x2 = vertex2[0]
		y2 = vertex2[1]
		#--------# y = mx + b #--------#
		#Valores en X
		dx = abs(x2-x1)
		#Valores en Y
		dy = abs(y2-y1)
		st = dy > dx
		#Condicion cuando la pendiente es ---> (dy/0)
		if dx == 0:
			for y in range(y1, y2+1):
				self.point(x1, y)
			return 0
		#Condicion para completar la linea
		if(st):
			x1,y1 = y1,x1
			x2,y2 = y2,x2
		if(x1>x2):
			x1,x2 = x2,x1
			y1,y2 = y2,y1
		#Valores en X
		dx = abs(x2-x1)
		#Valores en Y
		dy = abs(y2-y1)
		llenar = 0
		limite = dx
		y = y1
		#pendiente
		#m = dy/dx
		for x in range(x1,(x2+1)):
			if (st):
				self.point(y,x)
			else:
				self.point(x,y)
			llenar += dy * 2
			if llenar >= limite:
				y += 1 if y1 < y2 else -1
				limite += 2*dx

	#Funcion para cargar los archivos
	def load(self, filename, scale=(1, 1), translate=(0, 0)):
		objeto = Obj(filename)
		caras = objeto.faces
		vertexes = objeto.vertices
		for face in caras:
			vcount = len(face)
			for j in range(vcount):
				#Por cada cara se saca modulo para emparejamiento.
				i = (j+1)%vcount
				#Creacion del cuerpo
				f1 = face[j][0]
				f2 = face[i][0]
				v1 = vertexes[f1 - 1]
				v2 = vertexes[f2 - 1]
				#Le damos un valor a las coordeadas redondeadas
				x1 = round((v1[0] + translate[0]) * scale[0]);
				y1 = round((v1[1] + translate[1]) * scale[1]);
				x2 = round((v2[0] + translate[0]) * scale[0]);
				y2 = round((v2[1] + translate[1]) * scale[1]);
				#Guardando las coordenadas en un lista
				vertex = []
				vertex.append(x1)
				vertex.append(y1)
				vertex.append(x2)
				vertex.append(y2)
				self.glLine((vertex[0],vertex[1]),(vertex[2],vertex[3]))

	#Funcion para la renderizacion sin utilizar texturas
	def triangulos(self,A,B,C, color=None):
		b_min, b_max = bbox(A,B,C)
		for x in range(b_min.x, b_max.x+1):
			for y in range(b_min.y, b_max.y):
				w, v, u = baricentricas(A,B,C, v2(x,y))
				#Si los valores son negativos que continue sin nada
				if  w < 0 or v < 0 or u < 0:
					continue
				#Encontramos el valor z
				z = A.z * w + B.z * v + C.z * u
				#Coloreando el objeto
				if z > self.zbuffer[x][y]:
					self.point(x,y,color)
					self.zbuffer[x][y] = z

	#Funcion para la renderizacion con texturas
	def triangulos_texturas(self,A,B,C, color=None, texture=None, coordenadas=(), intensity=1):
		b_min, b_max = bbox(A,B,C)
		for x in range(b_min.x, b_max.x+1):
			for y in range(b_min.y, b_max.y):
				w, v, u = baricentricas(A,B,C, v2(x,y))
				#Si los valores son negativos
				if (w<0) or (v<0) or (u<0):
					continue
				#Valores de las triangulos_texturas
				if texture:
					#Coordenadas para las texturas
					tA, tB, tC = coordenadas
					tx = tA.x * w + tB.x * v + tC.x * u
					ty = tA.y * w + tB.y * v + tC.y * u
					#Agregamos el Color
					color = texture.get_color(tx,ty,intensity)
				#Valor de z
				z = A.z * w + B.z * v + C.z * u
				#Funcion para evitar los numeros negativos
				if (x<0) or (y<0):
					continue
				if x < len(self.zbuffer[x]) and y < len(self.zbuffer[y]) and z < self.zbuffer[x][y]:
					self.point(x,y,color)
					z = self.zbuffer[x][y]

	#Vector 3 transformado
	def transform(self, vertex, translate=(0, 0, 0), scale=(1, 1, 1)):
		#Transformando los datos
		t1 = round((vertex[0] + translate[0]) * scale[0])
		t2 = round((vertex[1] + translate[1]) * scale[1])
		t3 = round((vertex[2] + translate[2]) * scale[2])

		return v3(t1,t2,t3)
	#Funcion para la renderizacion sin utilizar texturas
	def renderer(self, filename, scale=(1, 1, 1), translate=(0, 0, 0)):
		#Abrimos el archivo
		objetos = Obj(filename)
		luz = v3(0,0,1)
		#Objetos obtenido del obj
		caras = objetos.faces
		vertexes = objetos.vertices
		for face in caras:
			vcount = len(face)
			if vcount == 3:
				f1 = face[0][0] - 1
				f2 = face[1][0] - 1
				f3 = face[2][0] - 1
				#Vectores que contedran
				vector_1 = self.transform(vertexes[f1], translate, scale)
				vector_2 = self.transform(vertexes[f2], translate, scale)
				vector_3 = self.transform(vertexes[f3], translate, scale)
				print(vector_1)
				#Calculos de algebra
				vector_normal = normal(pCruz(resta(vector_1, vector_2), resta(vector_3, vector_1)))
				intensidad = dot(vector_normal, luz)
				tonalidad = round(255 * intensidad)
				#Si la tonalidad es menor a 0, es decir, negativo, que no pinte nada
				if tonalidad < 0:
					continue
				#Mandamos los colores a la funcion que lo dibuja
				self.triangulos(vector_1, vector_2, vector_3, color(tonalidad, tonalidad, tonalidad))
			else:
				f1 = face[0][0] - 1
				f2 = face[1][0] - 1
				f3 = face[2][0] - 1
				f4 = face[3][0] - 1
				#lista de vertices
				lista_vertices = []
				V1 = self.transform(vertexes[f1], translate, scale)
				V2 = self.transform(vertexes[f2], translate, scale)
				V3 = self.transform(vertexes[f3], translate, scale)
				V4 = self.transform(vertexes[f4], translate, scale)
				#guardamos datos en la lista
				lista_vertices.append(V1)
				lista_vertices.append(V2)
				lista_vertices.append(V3)
				lista_vertices.append(V4)
				#Calculo de algebra
				vector_normal = normal(pCruz(resta(lista_vertices[0], lista_vertices[1]), resta(lista_vertices[1], lista_vertices[2])))  # no necesitamos dos normales!!
				intensidad = dot(vector_normal, luz)
				tonalidad = round(255 * intensidad)
				#Si la tonalidad es menor a 0, es decir, negativo, que no pinte nada
				if tonalidad < 0:
					continue
				#Mandamos los colores a la funcion que lo dibuja
				self.triangulos(lista_vertices[0], lista_vertices[1], lista_vertices[2], color(tonalidad, tonalidad, tonalidad))
				self.triangulos(lista_vertices[0], lista_vertices[2], lista_vertices[3], color(tonalidad, tonalidad, tonalidad))

	#Funcion para texturas renderizando
	def texture_render(self, filename, scale=(1, 1, 1), translate=(0, 0, 0), texture=None):
		objetos = Obj(filename)
		cara = objetos.faces
		#Vector para la luz
		luz = v3(0,0,1)

		#Ciclo para recorrer cada una de las caras
		for caras in cara:
			longitud = len(caras)
			#Ciclo para dibujar triangulos
			if longitud == 3:
				f1 = caras[0][0] - 1
				f2 = caras[1][0] - 1
				f3 = caras[2][0] - 1
				#Valores para el triangulos_texturas
				a = self.transform(objetos.vertices[f1],translate,scale)
				b = self.transform(objetos.vertices[f2],translate,scale)
				c = self.transform(objetos.vertices[f3],translate,scale)
				#Vector normal
				vector_normal = normal(pCruz(resta(b,a),resta(c,a)))
				intensidad = dot(vector_normal, luz)
				#Si no hay triangulos_texturas
				if not texture:
					gris = round(255*intensidad)
					if gris<0:
						continue
					self.triangulos_texturas(a,b,c, color=color(gris,gris,gris))
				else:
					#Trabajando con cuadrados
					t1 = caras[0][0] - 1
					t2 = caras[1][0] - 1
					t3 = caras[2][0] - 1
					#Coordenads para la texturas
					tA = v3(*objetos.vertices[t1])
					tB = v3(*objetos.vertices[t2])
					tC = v3(*objetos.vertices[t3])
					#Damos valor al color
					self.triangulos_texturas(a,b,c, texture=texture, coordenadas=(tA,tB,tC), intensity=intensidad)
			else:
				#Asuminedo que es un triangulo
				f1 = caras[0][0] - 1
				f2 = caras[1][0] - 1
				f3 = caras[2][0] - 1
				f4 = caras[3][0] - 1
				#vertices dependiendo de las caras
				listVertices = [
					self.transform(objetos.vertices[f1],translate,scale),
					self.transform(objetos.vertices[f2],translate,scale),
					self.transform(objetos.vertices[f3],translate,scale),
					self.transform(objetos.vertices[f4],translate,scale)
					]
				#Vertor normal
				vector_normal = normal(pCruz(resta(listVertices[0],listVertices[1]),resta(listVertices[2],listVertices[3])))
				intensidad = dot(vector_normal,luz)
				gris = round(255*intensidad)
				#Valores para cada coordenada de la listVertices
				A,B,C,D = listVertices
				#Condicion por si no hay texturas para añadir
				if not texture:
					gris = round(255*intensidad)
					#Condicion para evitar los numeros negativos
					if gris<0:
						continue
					#Anadimos los colores
					self.triangulos_texturas(A,B,C, color(gris,gris,gris))
					self.triangulos_texturas(A,C,D, color(gris,gris,gris))
				else:
					t1 = caras[0][1] - 1
					t2 = caras[1][1] - 1
					t3 = caras[2][1] - 1
					t4 = caras[3][1] - 1
					#Añadimos cada valor de la lista_vertices
					tA = v3(*objetos.vertices[t1])
					tB = v3(*objetos.vertices[t2])
					tC = v3(*objetos.vertices[t3])
					tD = v3(*objetos.vertices[t4])
					#Anadimos los colores mandandola a la funcion
					self.triangulos_texturas(A,B,C, texture=texture, coordenadas=(tA,tB,tC), intensity=intensidad)
					self.triangulos_texturas(A,C,D, texture=texture, coordenadas=(tA,tC,tD), intensity=intensidad)

	def ModelMatriz(self, translate=(0,0,0), scale=(1,1,1), rotate=(0,0,0)):
		#Matriz translate
		matrizTranslate = [
			[1,0,0, translate[0]],
			[0,1,0, translate[1]],
			[0,0,1, translate[2]],
			[0,0,0,1]
		]
		#Matriz de escalar
		matrizScale = [
			[scale[0],0,0,0],
			[0,scale[1],0,0],
			[0,0,scale[2],0],
			[0,0,0,1]
		]
		#Matriz para rotar
		valor = rotate[0]
		#Matriz rotar en el eje x
		matrizRotateX = [
			[1,0,0,0],
			[0,math.cos(valor),-1*(math.sin(valor)),0],
			[0,math.sin(valor),math.cos(valor),0],
			[0,0,0,1]
		]
		#Matriz de rotacion en Y
		valor = rotate[1]
		matrizRotateY = [
			[math.cos(valor),0,math.sin(valor),0],
			[0,1,0,0],
			[-1*(math.sin(valor)),0,math.cos(valor),0],
			[0,0,0,1]
		]
		#Matriz de rotacion en Z
		valor = rotate[2]
		matrizRotateZ = [
			[math.cos(valor),-1*(math.sin(valor)),0,0],
			[math.sin(valor),math.cos(valor),0,0],
			[0,0,1,0],
			[0,0,0,1]
		]
		#Mutliplicacion de las MATRICES
		matriz1 = multiplicarMatrices(matrizRotateZ,matrizRotateY)
		matrizRotate = multiplicarMatrices(matriz1,matrizRotateX)
		#Segunda multiplicacion de MATRICES
		matriz2 = multiplicarMatrices(matrizRotate,matrizScale)
		self.model = multiplicarMatrices(matrizTranslate,matriz2)

	#Funcion para encontrar la matriz de View
	def ViewMatriz(self, x, y, z, center):
		#Matriz de x
		MX = [
			[(x.x),(x.y),(x.z), 0],
			[(y.x),(y.y),(y.z), 0],
			[(z.x),(z.y),(z.z), 0],
			[0,0,0,		1		 ]
		]
		#Matriz de y
		MY = [
			[1,0,0,(-1*center.x)],
			[0,1,0,(-1*center.y)],
			[0,0,0,(-1*center.z)],
			[0,0,0,   	1		]
		]
		#Multiplicamos las 2 matrices
		self.View_Matriz = multiplicarMatrices(MX,MY)

	#Funcion para encontrar la funcion de proyeccion
	def ProjectionMatriz(self, coeficiente):
		self.projection = [
			[1,0,      0	,0],
			[0,1,	   0	,0],
			[0,0,      1	,0],
			[0,0,coeficiente,1]
		]

	#Funcion para contrar la matriz de viewPort
	def ViewPortMatriz(self,x,y):
		self.viewPort = [
			[round(self.width*0.5),0,0,(x+self.width*0.5)],
			[0,(self.height*0.5),0,(y+self.height*0.5)],
			[0,0,128,128],
			[0,0,0,1]
		]

	#Mirar los datos
	def vista(self, vista, centrar, arriba):
		z = normal(resta(vista, centrar))
		x = normal(pCruz(arriba,z))
		y = normal(pCruz(z,x))
		#Mandamos los aes a la funcion ViewMatriz para encontrar la matriz|
		self.ViewMatriz(x,y,z,centrar)
		self.ModelMatriz()
		self.ProjectionMatriz(1/round(longitud(resta(vista,centrar))))
		#print(self.ProjectionMatriz)
		#----------------#
	#Cramos las matrices solicitadas (Model, View, Projection, ViewPort)
	def matrizPipeline(self):
		mp1 = multiplicarMatrices(self.model, self.View_Matriz)
		mp2 = multiplicarMatrices(self.projection, mp1)
		mp3 = multiplicarMatrices(self.viewPort, mp2)
		self.mPipeline = mp3
		print(self.projection)
		print(mp2)
		print(mp3)
	#Transformamos la matriz
	def transformMatriz(self, vector):
		#Mandamos la Funcion
		self.matrizPipeline()
		#creamos las matrices
		newVertex = [vector.x,vector.y,vector.z,1]
		multVector = multiplicarMatrices(self.mPipeline,newVertex)
		#Valores para el vector
		a = multVector[0][0]/multVector[3][0]
		b = multVector[1][0]/multVector[3][0]
		c = multVector[2][0]/multVector[3][0]
		#Retronamos los Valores
		return v3(a,b,c)

	#Funcion para cargar junto a la matriz
	def loadMatriz(self, filename,texture=None, translate=(0,0,0), scale=(1,1,1), rotate=(0,0,0)):
		#Abrimos el archivos
		objetos = Obj(filename)
		luz = v3(0,0,1)
		#Datos del archivo
		caras = objetos.faces
		vertexes = objetos.vertices
		#Ciclo para completar
		for face in caras:
			vcount=len(face)
			if vcount == 3:
				f1 = face[0][0] - 1
				f2 = face[1][0] - 1
				f3 = face[2][0] - 1
				#Extraemos las Coordenadas
				vector_1 = self.transform(vertexes[f1], translate, scale)
				vector_2 = self.transform(vertexes[f2], translate, scale)
				vector_3 = self.transform(vertexes[f3], translate, scale)
				#print(vector_1)
				#transformamos los datos con la matriz
				vector_1 = self.transformMatriz(vector_1)
				vector_2 = self.transformMatriz(vector_2)
				vector_3 = self.transformMatriz(vector_3)
				#Datos para pintar cada dato
				vector_normal = normal(pCruz(resta(V1, V2), resta(V3, V1)))
				intensidad = dot(vector_normal, luz)
				tonalidad = round(255 * intensidad)
				#Si la tonalidad es menor a 0, es decir, negativo, que no pinte nada
				if tonalidad < 0:
					continue
				#Mandamos los datos a la funcion de colorear
				self.triangulos(V1, V2, V3, color(tonalidad, tonalidad, tonalidad))
			else:
				#Valores para cuadrados
				f1 = face[0][0] - 1
				f2 = face[1][0] - 1
				f3 = face[2][0] - 1
				f4 = face[3][0] - 1
				#Lista de vertices
				lista_vertices = [
					self.transform(vertexes[f1], translate, scale),
					self.transform(vertexes[f2], translate, scale),
					self.transform(vertexes[f3], translate, scale),
					self.transform(vertexes[f4], translate, scale)
				]

				#print(lista_vertices[0])
