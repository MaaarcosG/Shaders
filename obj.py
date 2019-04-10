# Universidad del Valle de Guatemala
# Grafica por Computadora
# Nombre: Marcos Gutierrez
# Carne: 17909

import struct
#Funcion para el color
def color(r,g,b):
    return bytes([r,g, b])

#Clase para leer los objetos
class Obj(object):
    def __init__(self, filename):
        self.verificador = False
        with open(filename) as f:
            self.lines = f.read().splitlines()
        '''
        if mtl:
            self.verificador = True
            with open(mtl) as m:
                self.lines2 = m.read().splitlines()
        '''
        #Variables guardar en una lista
        self.vertices = []
        self.faces = []
        self.vt = []
        self.vn = []
        self.materiales = []
        self.kd = []
        self.tipo = []
        self.ordenar = []
        self.texture = []
        self.read()

    def read(self):
        #Condicion para ver los colores
        if self.verificador:
            for line2 in self.lines2:
                if line2:
                    prefixes, valor = linea.split(' ', 1)
                    if prefixes == 'newmtl':
                        self.tipo.append(valor)
                    if prefixes == 'Kd':
                        self.kd.append(list(map(float, valor.split(' '))))
            for indice in range(len(self.tipo)-1):
                self.materiales[self.tipo[indice]] = self.kd[indice]
        #Separador
        self.mater = ''
        #Ciclo para encontrar los valores para el eskeleto del objeto
        for lineas in self.lines:
            if lineas:
                prefix, value = lineas.split(' ', 1)
                if prefix == 'v':
                    self.vertices.append(list(map(float, value.split(' '))))
                elif prefix == 'f':
                    if self.verificador:
                        #Agregando
                        lista = [list(map(int, face.split('/'))) for face in value.split(' ')]
                        lista.append(mater)
                        self.faces.append(lista)
                    else:
                        self.faces.append([list(map(int, face.split('/'))) for face in value.split(' ')])
                elif prefix == 'vt':
                    self.vt.append(list(map(float, value.split(' '))))
                elif prefix == 'vn':
                    self.vn.append(list(map(float, value.split(' '))))
                elif prefix == 'usemtl':
                    self.mater = value

class Texture(object):
    def __init__(self, path):
        self.path = path
        self.read()

    def read(self):
        image = open(self.path, "rb")
        # we ignore all the header stuff
        image.seek(2 + 4 + 4)  # skip BM, skip bmp size, skip zeros
        header_size = struct.unpack("=l", image.read(4))[0]  # read header size
        image.seek(2 + 4 + 4 + 4 + 4)

        self.width = struct.unpack("=l", image.read(4))[0]  # read width
        self.height = struct.unpack("=l", image.read(4))[0]  # read width
        self.pixels = []
        image.seek(header_size)
        for y in range(self.height):
            self.pixels.append([])
            for x in range(self.width):
                b = ord(image.read(1))
                g = ord(image.read(1))
                r = ord(image.read(1))
                self.pixels[y].append(color(r,g,b))
        image.close()

    def get_color(self, tx, ty, intensity=1):
        x = int(tx * self.width)
        y = int(ty * self.height)
        # return self.pixels[y][x]
        try:
            return bytes(map(lambda b: round(b*intensity) if b*intensity > 0 else 0, self.pixels[y][x]))
        except:
            pass  # what causes this

class Mtl(object):
	def __init__(self,filename):
		with open(filename) as f:
			self.lines=f.read().splitlines()

		self.materiales = []
		self.read()

	def read(self):
		for line in self.lines:
			if line:
				prefix, value = line.split(" ", 1)
				if prefix == 'Kd':
					#valores RGB se obtienen leyendo
					self.materiales.append(list(map(float, value.split(" "))))
