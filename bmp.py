# Universidad del Valle de Guatemala
# Grafica por Computadora
# Nombre: Marcos Gutierrez
# Carne: 17909

import struct
from obj import *
from math import *
from math import sin, cos
#Importamos la coleccion
from collections import namedtuple
#Vectores necesarios
V2 = namedtuple('Punto2', ['x', 'y'])
V3 = namedtuple('Punto3', ['x', 'y', 'z'])
#----------Funciones necesarios para trabajar en opengl--------#
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
def sum(v0,v1):
	#Puntos en cada coordenadas
	px = v0.x + v1.x
	py = v0.y + v1.y
	pz = v0.z + v1.z

	#retorna un vector nuevo con la suma
	return V3(px,py,pz)

#Resta de coordeadas
def sub(v0,v1):
	#Puntos en cada coordenadas
	px = v0.x - v1.x
	py = v0.y - v1.y
	pz = v0.z - v1.z

	#retorna un vector nuevo con la resta
	return V3(px,py,pz)

def mul(v0,k):
	#Puntos en cada coordenadas
	px = v0.x * k
	py = v0.y * k
	pz = v0.z * k

	#retorna un vector nuevo con la multiplicacion a un escalar
	return V3(px,py,pz)

def dot(v0, v1):
  return v0.x * v1.x + v0.y * v1.y + v0.z * v1.z

#Funcion que encontrar un vector nuevo, utlizando algebra producto cruz
def cross(v0,v1):
	#Puntos en cada coordenadas
	p1 = v0.y * v1.z - v0.z * v1.y
	p2 = v0.z * v1.x - v0.x * v1.z
	p3 = v0.x * v1.y - v0.y * v1.x

	#Retorna un nuevo vector
	return V3(p1,p2,p3)

#Funcion para la longitud del vector
def length(v0):
	px = v0.x ** 2
	py = v0.y ** 2
	pz = v0.z ** 2
	#Suma de puntos
	len = (px+py+pz)**0.5
	return len

#Funcion para encontrar el vector normalr
def norm(v0):
	v0Lon = length(v0)

	if not v0Lon:
		return V3(0,0,0)

	px = v0.x/v0Lon
	py = v0.y/v0Lon
	pz = v0.z/v0Lon

	return V3(px,py,pz)

#Bounding Box
def bbox(*vertices):
	xs = [vertex.x for vertex in vertices]
	ys = [vertex.y for vertex in vertices]
	xs.sort()
	ys.sort()
	'''
	p1 = xs[0], ys[0]
	p2 = xs[-1], ys[-1]
	'''
	return V2(xs[0], ys[0]), V2(xs[-1], ys[-1])

#Funcion para encontrar las coordenadas barycentricas
def baricentricas(A,B,C,P):
	bcoor = cross(
		V3(C.x - A.x, B.x - A.x, A.x - P.x),
		V3(C.y - A.y, B.y - A.y, A.y - P.y)
	)

	if abs(bcoor.z) < 1:
		return(-1,-1,-1)

	return (1 - (bcoor.x + bcoor.y) / bcoor.z, bcoor.y / bcoor.z, bcoor.x / bcoor.z)

#------ FUNCIONES PARA TRABJAR CON MATRICES ------#
# matriz uno = m1
# matriz dos = m2

#Comprobando teorema para crear una multiplicacion
def teorema(filas, columna):
	matriz = []
	for i in range(filas):
		#Añadimos una lista vacia
		matriz.append([])
		for j in range(columna):
			matriz[-1].append(0.0)
	#Retornamos la matriz creada
	return matriz

#Funcion para multiplicar las matrices
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
	matrizResultante = teorema(len(m1), len(m2[0]))
	for i in range(len(m1)):
		#Creamos la matriz
		for j in range(len(m2[0])):
			for k in range(len(m2)):
				#damos valores a la matriz resultante
				matrizResultante[i][j] += m1[i][k] * m2[k][j]
	#retornmaos los resultados de la matriz
	return matrizResultante


#Clase de Bitmap
class Bitmap(object):
	def __init__(self, width, height):
		#Variables
		self.width = width
		self.height = height
		self.framebuffer = []
		self.color = color(255,255,255)
		self.vertexColor = color(255,255,255)
		self.glCreateWindows()
		self.clear()
		self.zbuffer = [
			[-float('inf') for x in range(self.width)]
			for y in range(self.height)
		]

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
		#Ciclo para recorrer
		for x in range(self.height):
			for y in range(self.width):
				f.write(self.framebuffer[x][y])
		f.close()

	#Creacion de archivo
	def archivo(self, filename='out.bmp'):
		self.write(filename)

	#Crear la pantalla
	def glCreateWindows(self):
		self.framebuffer = [[0 for x in range(self.height)] for y in range(self.width)]

	#Limpiar
	def clear(self):
		self.framebuffer = [[color(0, 0, 0) for x  in range(self.width)] for y in range(self.height)]
		self.zbuffer = [[-float('inf') for x in range(self.width)] for y in range(self.height)]

	#Limpiar el color
	def glClearColor(self, r,g,b):
		R = int(r*255)
		G = int(g*255)
		B = int(b*255)
		#Le damos valor al buffer
		self.framebuffer = [
			[color(R,G,B)
			for x in range(self.width)]
			for y in range(self.height)]

	#Funcion que definira el area de la imagen
	def glViewPort(x,y,width,height):
		#Inicializamos los componentes
		self.x = x
		self.y = y
		self.width_vp = width
		self.heigth_vp = heigth

	#Fucnion para los valores
	def glVertex(self, x, y):
		vertex_x = int((x+1)*(self.width_vp*0.5)) + self.x
		vertex_y = int((y+1)*(self.heigth_vp*0.5)) + self.y
		#Funcion para evitar los numeros que no se encuentren entre 0 y 1
		try:
			if (x==1) and (y==1):
				vertex_x = vertex_x - 1
				vertex_y = vertex_y - 1
		except IndexError:
			print('Revise los valores ingresados para el glViewPort')
		#retornamos los valores del vertex
		return (vertex_x,vertex_y)

	#Funcion para cambiar color al vertex
	def glColor(self, r,g,b):
		self.vertexColor = color(r,g,b)

	#Funcion para los puntos
	def point(self, x, y,color=None):
		self.framebuffer[y][x]= color or self.vertexColor

	#Funcion para realizar las lineas
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

	#Funcion para shaders
	def shaders(self, **kwargs):
		#Coordenadas barycentricas
		w,u,v = kwargs['bary']
		#Coordenadas x e y
		x,y = ['xy']
		#Coordenadas de las texturas
		tA, tB, tC = kwargs['tc']
		#Vectores normales
		nA, nB, nC = ['varing_normals']
		#Se toma la luz
		ligth = ['ligth']
		#Datos barycentricas con los vectores normales
		nx = nA.x * w + nB.x * v + nC.x * u
		ny = nA.y * w + nB.y * v + nC.y * u
		nz = nA.z * w + nB.z * v + nC.z * u
		#guardamos los valores en la tupla
		vn = V3(nx, ny, nz)
		#Genereamos la intensidad
		intensity = dot(vn, ligth)
		#retonamos Valores
		a = int(0 * intensity) if 0 * intensity > 0 else 0
		b = int(0 * intensity) if 0 * intensity > 0 else 0
		c = int(255 * intensity) if 255 * intensity > 0 else 0
		return bytes([a,b,c])

	def triangulos(self, A, B, C, laLuz, color=None,texture=None, tc=(), intensity=1, nc=()):
		bbox_min, bbox_max = bbox(A,B,C)
		for x in range(bbox_min.x, bbox_max.x + 1):
			for y in range(bbox_min.y, bbox_max.y + 1):
				#Utilizamos coordenadas baricentricas
				w, v, u = baricentricas(A, B, C, V2(x, y))
				#Evitamos numero negativos
				if w < 0 or v < 0 or u < 0:
					continue
				#Condicion de color
				if color:
					self.vertexColor = color
				'''
				Metodo de gourad
				coloring = self.shaders(
					xy = (x,y),
					bary = (w,v,u),
					varing_normals = nc,
					ligth = laLuz
				)
				self.vertexColor = coloring
				'''
				#Si encuentra un archivo de textura
				if texture:
					tA, tB, tC = texture_coords
					tx = tA.x * w + tB.x * v + tC.x * u
					ty = tA.y * w + tB.y * v + tC.y * u
					color = texture.get_color(tx, ty, intensity)
					self.vertexColor = color
				#Encontramos el valor de z
				z = A.z * w + B.z * v + C.z * u
				if x < self.width and y<self.height and x >=0 and y>=0:
					#Funcion para llenar cada uno de los espacios
					if x < len(self.zbuffer) and y < len(self.zbuffer[x]) and z > self.zbuffer[x][y]:
						self.point(x, y, color)
						self.zbuffer[x][y] = z


	#Funcion para realizar las transformaciones
	def transform(self, vector, translate=(0,0,0), scale=(1,1,1), rotate=(0,0,0)):
		newMatriz = [[vector.x],[vector.y],[vector.z],[1]]
		#Matrix de la vistas
		mV = multiplicarMatrices(self.viewPortM, self.projection)
		view = multiplicarMatrices(mV, self.View_Matriz)
		#Matriz de  vista y projection
		vP = multiplicarMatrices(view, self.Model)
		newVertex = multiplicarMatrices(vP,newMatriz)
		#Valores para la transformacion de vertices
		a = round((newVertex[0][0])/newVertex[3][0])
		b = round((newVertex[1][0])/newVertex[3][0])
		c = round((newVertex[2][0])/newVertex[3][0])
		#Añadimos los valores a una nueva lista
		return (V3(a,b,c))

	#Matriz Model
	def ModelMatriz(self, translate, scale, rotate):
		translate = V3(*translate)
		scale = V3(*scale)
		rotate = V3(*rotate)
		#Le damos una matriz al modelo
		matrixTranslate = [[0 for x in range(4)] for y in range(4)]
		#Matriz translate
		matrixTranslate = [
			[1,0,0, translate.x],
			[0,1,0, translate.y],
			[0,0,1, translate.z],
			[0,0,0,1]
		]
		#Matriz de escalar
		matrizScale = [
			[scale.x,0,0,0],
			[0,scale.y,0,0],
			[0,0,scale.z,0],
			[0,0,0,1]
		]
		#Matriz de rotacion
		matrizRotateX = [
			[1,0,0,0],
			[0,cos(rotate.x),-sin(rotate.x),0],
			[0,sin(rotate.x), cos(rotate.x),0],
			[0,0,0,1]
		]
		#rotacion en y
		matrizRotateY = [
			[cos(rotate.y),0,sin(rotate.y),0],
			[0,1,0,0],
			[sin(rotate.y),0,cos(rotate.y),0],
			[0,0,0,1]
		]
		#rotacion en z
		matrizRotateZ = [
            [cos(rotate.z),-sin(rotate.z),0,0],
            [sin(rotate.z), cos(rotate.z),0,0],
            [0,0,1,0],
            [0,0,0,1]
        ]
		rotateMatriz = multiplicarMatrices(multiplicarMatrices(matrizRotateX,matrizRotateY) ,matrizRotateZ)
		self.Model = multiplicarMatrices(multiplicarMatrices(matrixTranslate,rotateMatriz), matrizScale)

	#Funcion para contrar la matriz de viewPort
	def ViewPortMatriz(self,x=0,y=0):
		self.viewPortM = [
			[(self.width*0.5),0,0,(x+self.width*0.5)],
			[0,(self.height*0.5),0,(y+self.height*0.5)],
			[0,0,128,128],
			[0,0,0,1]
		]

	#Funcion para encontrar la funcion de proyeccion
	def ProjectionMatriz(self, coeficiente):
		self.projection = [
			[1,0,      0	,0],
			[0,1,	   0	,0],
			[0,0,      1	,0],
			[0,0,coeficiente,1]
		]

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

	#Funcion para controlar las vistas
	def lookAt(self, ojo, arriba, centro):
		z = norm(sub(ojo,centro))
		x = norm(cross(arriba,z))
		y = norm(cross(z,x))
		#Mandamos valores a la matriz de vistas
		self.ViewMatriz(x,y,z, centro)
		valor = norm(sub(ojo,centro))
		#Mandamos valores a la matriz de projection
		self.ProjectionMatriz(-1/length(sub(ojo,centro)))

	#Funcion para cargar
	def load(self, filename, mtlFile=None, translate=(0,0,0), scale=(1,1,1), rotate=(0,0,0), ojo=V3(0,0,0), arriba=V3(0,0,0), centro=V3(0,0,0), texture=None):
		objetos = Obj(filename)
		#Mandamos los valores a las funciones que encuentran las MATRICES
		self.ModelMatriz(translate,scale,rotate)
		self.lookAt(ojo, arriba, centro)
		#Llamamos la funcion necesaria
		self.ViewPortMatriz()
		#Verificamos si existe un archivo para la textura
		if texture == None:
			pass
		else:
			textura = Texture(texture)
		#Archivo mtlFile
		if mtlFile:
			mtl = Mtl(mtlFile)
			vR = mtl.materiales[0][0]
			vG = mtl.materiales[0][1]
			vB = mtl.materiales[0][2]
			#print(vR)
		else:
			vR = color(255,255,255)
			vG = color(255,255,255)
			vB = color(255,255,255)
			#print(vR)
		#Luz
		ligth = V3(0,0,1)
		#Datos para reccorer el Ciclo
		caras = objetos.faces
		vertexes = objetos.vertices
		nver = objetos.vn
		for face in caras:
			vcount = len(face)
			#print(vcount)
			if vcount == 3:
				f1 = face[0][0] - 1
				f2 = face[1][0] - 1
				f3 = face[2][0] - 1
				#transformamos los datos a los VECTORES
				a = self.transform(V3(*vertexes[f1]),translate, scale)
				b = self.transform(V3(*vertexes[f2]),translate, scale)
				c = self.transform(V3(*vertexes[f3]),translate, scale)
				'''
				print('1', a)
				print('2', b)
				print('3', c)
				'''
				#Por si no existe un archivo de texturas
				vector_normal = norm(cross(sub(b,a), sub(c,a)))
				intensidad = dot(vector_normal, ligth)
				#Si no encuentra un texturas
				if not texture:
					red = round(255*vR*intensidad)
					green = round(255*vG*intensidad)
					blue = round(255*vB*intensidad)
					#Evitamos los numeros negativos
					if (red<0) or (green<0) or (blue<0):
						continue
					#Mandamos los valores a triangulos
					self.triangulos(a,b,c, color(red,green,blue))
				else:
					t1 = face[0][0] - 1
					t2 = face[1][0] - 1
					t3 = face[2][0] - 1
					#Mandamos los Valores
					Texture_A = V3(*objetos.textVert[t1],0)
					Texture_B = V3(*objetos.textVert[t2],0)
					Texture_C = V3(*objetos.textVert[t3],0)
					#Mandamos los datos al triangulos
					self.triangulos(Texture_A,Texture_B,Texture_C, color(), None, texture=textura, tc=(Texture_A,Texture_B,Texture_C), intensity=intensidad)
