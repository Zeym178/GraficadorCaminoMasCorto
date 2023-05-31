import tkinter as tk
from tkinter import ttk
import time
import networkx as nx

# Dimensiones del mapa
ANCHO_MAPA = 20
ALTO_MAPA = 20

# Colores para los diferentes tipos de terreno
COLOR_TIERRA = "#D2B48C"
COLOR_CAMINO = "#00BFFF"
COLOR_INICIO = "#00FF00"
COLOR_FINAL = "#FF0000"
COLOR_CAMINO_CORTO = "#FFFF00"

class MapaInteractivo(tk.Tk):
    def __init__(self, ancho, alto):
        super().__init__()
        self.title("Mapa Interactivo")

        self.canvas = tk.Canvas(self, width=ancho * 30, height=alto * 30, bg=COLOR_TIERRA)
        self.canvas.pack(fill=tk.BOTH, expand=1)

        self.mapa = [[0 for _ in range(ancho)] for _ in range(alto)]
        self.inicio = None
        self.final = None
        self.dibujar_mapa()

        self.canvas.bind("<Button-1>", self.actualizar_casilla)

        self.generar_grafo_button = tk.Button(self, text="Generar Grafo", command=self.generar_grafo)
        self.generar_grafo_button.pack()

        self.marcar_inicio_button = tk.Button(self, text="Marcar Inicio", command=self.marcar_inicio)
        self.marcar_inicio_button.pack()

        self.marcar_final_button = tk.Button(self, text="Marcar Final", command=self.marcar_final)
        self.marcar_final_button.pack()

        self.method_combobox = ttk.Combobox(self, values=["Fuerza Bruta", "Backtracking", "Algoritmo Goloso",
                                                          "Programación Dinámica"], state="readonly")
        self.method_combobox.pack(side=tk.TOP)

        self.dibujar_camino_button = tk.Button(self, text="Dibujar Camino", command=self.dibujar_camino)
        self.dibujar_camino_button.pack()

        self.time_label = tk.Label(self, text="")
        self.time_label.pack(side=tk.TOP)

    def dibujar_mapa(self):
        self.canvas.delete("all")

        for fila in range(len(self.mapa)):
            for columna in range(len(self.mapa[0])):
                x1 = columna * 30
                y1 = fila * 30
                x2 = x1 + 30
                y2 = y1 + 30

                if (fila, columna) == self.inicio:
                    color = COLOR_INICIO
                elif (fila, columna) == self.final:
                    color = COLOR_FINAL
                elif self.mapa[fila][columna] == 2:
                    color = COLOR_CAMINO_CORTO
                else:
                    color = COLOR_TIERRA if self.mapa[fila][columna] == 0 else COLOR_CAMINO

                self.canvas.create_rectangle(x1, y1, x2, y2, fill=color, outline="black")

    def actualizar_casilla(self, event):
        columna = event.x // 30
        fila = event.y // 30

        if 0 <= fila < len(self.mapa) and 0 <= columna < len(self.mapa[0]):
            self.mapa[fila][columna] = 1 - self.mapa[fila][columna]  # Alternar entre tierra y agua
            self.dibujar_mapa()

    def marcar_inicio(self):
        self.marcar_casilla(COLOR_INICIO)
        self.marcar_inicio_button.configure(state="disabled")  # Deshabilitar el botón después de marcar el inicio

    def marcar_final(self):
        self.marcar_casilla(COLOR_FINAL)
        self.marcar_inicio_button.configure(state="normal")

    def marcar_casilla(self, color):
        def marcar(event):
            columna = event.x // 30
            fila = event.y // 30

            if 0 <= fila < len(self.mapa) and 0 <= columna < len(self.mapa[0]):
                if color == COLOR_INICIO:
                    if self.inicio:
                        self.mapa[self.inicio[0]][self.inicio[1]] = 0  # Restablecer la casilla anterior del inicio
                    self.inicio = (fila, columna)
                elif color == COLOR_FINAL:
                    if self.final:
                        self.mapa[self.final[0]][self.final[1]] = 0  # Restablecer la casilla anterior del final
                    self.final = (fila, columna)

                self.mapa[fila][columna] = 1
                self.dibujar_mapa()

        self.canvas.bind("<Button-1>", marcar)

    def generar_grafo(self):
        G = nx.Graph()

        if self.inicio and self.final and self.grafo:
            for fila in range(len(self.mapa)):
                for columna in range(len(self.mapa[0])):
                    if self.mapa[fila][columna] == 2:
                        self.mapa[fila][columna] = 1

        for fila in range(len(self.mapa)):
            for columna in range(len(self.mapa[0])):
                if self.mapa[fila][columna] == 1:
                    G.add_node((fila, columna))

                    # Verificar vecinos
                    if fila > 0 and self.mapa[fila - 1][columna] == 1:
                        G.add_edge((fila, columna), (fila - 1, columna), weight=1)

                    if fila < len(self.mapa) - 1 and self.mapa[fila + 1][columna] == 1:
                        G.add_edge((fila, columna), (fila + 1, columna), weight=1)

                    if columna > 0 and self.mapa[fila][columna - 1] == 1:
                        G.add_edge((fila, columna), (fila, columna - 1), weight=1)

                    if columna < len(self.mapa[0]) - 1 and self.mapa[fila][columna + 1] == 1:
                        G.add_edge((fila, columna), (fila, columna + 1), weight=1)

        print("Grafo generado:")
        print(G.nodes)
        print(G.edges)

        self.grafo = G



    def dibujar_camino(self):
        if self.inicio and self.final and self.grafo:
            for fila in range(len(self.mapa)):
                for columna in range(len(self.mapa[0])):
                    if self.mapa[fila][columna] == 2:
                        self.mapa[fila][columna] = 1

            method = self.method_combobox.get()
            start_time = time.perf_counter_ns()

            if method == "Fuerza Bruta":
                shortest_path = fuerza_bruta(self.grafo, self.inicio, self.final)
            elif method == "Backtracking":
                shortest_path = backtracking(self.grafo, self.inicio, self.final)
            elif method == "Algoritmo Goloso":
                shortest_path = algoritmo_goloso(self.grafo, self.inicio, self.final)
            elif method == "Programación Dinámica":
                shortest_path = backtracking_dynamic(self.grafo, self.inicio, self.final)
            else:
                return

            end_time = time.perf_counter_ns()
            elapsed_time = (end_time - start_time)

            for fila, columna in shortest_path:
                self.mapa[fila][columna] = 2  # Marcar el camino más corto como tipo "2"
            self.dibujar_mapa()

            self.time_label.config(text=f"Tiempo de ejecución ({method}): {elapsed_time} nanosegundos")
        self.canvas.bind("<Button-1>", self.actualizar_casilla)

def fuerza_bruta(graph, start, end):
    # Generar todas las posibles rutas
    all_paths = nx.all_simple_paths(graph, source=start, target=end)

    # Calcular la longitud de cada ruta y encontrar la más corta
    shortest_path = None
    shortest_length = float('inf')

    for path in all_paths:
        length = sum(graph[u][v]['weight'] for u, v in zip(path, path[1:]))
        if length < shortest_length:
            shortest_length = length
            shortest_path = path

    return shortest_path


# Función para calcular el camino más corto por backtracking
def backtracking(graph, start, end):
    # Variables para almacenar la ruta más corta y su longitud
    shortest_path = None
    shortest_length = float('inf')

    # Función recursiva de backtracking
    def backtrack(node, path, length):
        nonlocal shortest_path, shortest_length

        # Agregar el nodo actual al camino y actualizar la longitud
        path.append(node)
        length += graph[path[-2]][node]['weight'] if len(path) > 1 else 0

        # Si se alcanza el nodo de destino, comparar y actualizar la ruta más corta
        if node == end:
            if length < shortest_length:
                shortest_length = length
                shortest_path = list(path)
        else:
            # Recorrer los nodos adyacentes y realizar la recursión
            for neighbor in graph.neighbors(node):
                if neighbor not in path:
                    backtrack(neighbor, path, length)

        # Eliminar el nodo actual del camino para retroceder
        path.pop()

    # Inicializar la recursión desde el nodo de inicio
    backtrack(start, [], 0)

    return shortest_path


# Función para calcular el camino más corto por algoritmo goloso
def algoritmo_goloso(graph, start, end):
    current_node = start
    path = [current_node]

    while current_node != end:
        neighbors = list(graph.neighbors(current_node))
        next_node = None
        shortest_weight = float('inf')

        for neighbor in neighbors:
            weight = graph[current_node][neighbor]['weight']
            if weight < shortest_weight and neighbor not in path:
                shortest_weight = weight
                next_node = neighbor

        if next_node is None:
            break

        path.append(next_node)
        current_node = next_node

    return path


# Función para calcular el camino más corto por backtracking con programación dinámica
def backtracking_dynamic(graph, start, end):
    # Crear un diccionario para almacenar las distancias más cortas
    shortest_distances = {}

    # Función recursiva de backtracking con programación dinámica
    def backtrack(node, path, length):
        nonlocal shortest_distances

        # Agregar el nodo actual al camino y actualizar la longitud
        path.append(node)
        length += graph[path[-2]][node]['weight'] if len(path) > 1 else 0

        # Si se alcanza el nodo de destino, comparar y actualizar la distancia más corta
        if node == end:
            if end not in shortest_distances or length < shortest_distances[end][0]:
                shortest_distances[end] = (length, list(path))
        else:
            # Recorrer los nodos adyacentes y realizar la recursión
            for neighbor in graph.neighbors(node):
                if neighbor not in path:
                    # Si ya se ha calculado la distancia más corta a este vecino, usarla
                    if neighbor in shortest_distances and length + graph[node][neighbor]['weight'] >= shortest_distances[neighbor][0]:
                        continue
                    backtrack(neighbor, path, length)

        # Eliminar el nodo actual del camino para retroceder
        path.pop()

    # Inicializar la recursión desde el nodo de inicio
    backtrack(start, [], 0)

    return shortest_distances[end][1] if end in shortest_distances else None

# Crear y ejecutar la aplicación
app = MapaInteractivo(ANCHO_MAPA, ALTO_MAPA)
app.mainloop()
