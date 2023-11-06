from mesa.model import Model
from mesa.agent import Agent
from mesa.space import MultiGrid
from mesa.time import SimultaneousActivation
from mesa.datacollection import DataCollector

from collections import deque

import numpy as np

class Celda(Agent):
    def __init__(self, unique_id, model, suciedad: bool = False, es_estacion_carga: bool = False):
        super().__init__(unique_id, model)
        self.sucia = suciedad
        self.es_estacion_carga = es_estacion_carga

class Mueble(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)

class Carga(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)
    
class RobotLimpieza(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)
        self.sig_pos = None
        self.movimientos = 0
        self.carga = 100
        self.apartada = False

    def calc_dist(self, p1, p2):
        return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])

    def limpiar_una_celda(self, lista_de_celdas_sucias):
        celda_a_limpiar = self.random.choice(lista_de_celdas_sucias)
        celda_a_limpiar.sucia = False
        self.sig_pos = celda_a_limpiar.pos

    def sucia_bfs(self):
        queue = deque()
        queue.append(self.pos)
        visited = dict()

        while queue:

            cur_pos = queue.popleft()

            if cur_pos in visited:
                print("Visited")
                continue

            visited[cur_pos] = True
            vecinos = self.model.grid.get_neighbors(
                cur_pos, moore=True, include_center=False)
        
            for vecino in vecinos:
                if isinstance(vecino, Celda) and vecino.sucia:
                    return vecino.pos

                if not isinstance(vecino, (Mueble, RobotLimpieza)):
                    queue.append(vecino.pos)
        
        return 0


    def obtener_celda_sucia_mas_cercana(self):
        return self.sucia_bfs()

    def obtener_prioridad_de_vecinos(self, vecinos, posicion_destino):
        
        prioridades = []
        for vecino in vecinos:
            vecino_con_prioridad = (self.calc_dist(vecino.pos, posicion_destino), vecino)
            prioridades.append(vecino_con_prioridad)

        prioridades.sort(key=lambda x: x[0])
        print(prioridades)

        for i in range(len(vecinos)):
            vecinos[i] = prioridades[i][1]

        return vecinos
    
    def seleccionar_nueva_pos(self, lista_de_vecinos):
        pos_sucia_mas_cercana = self.obtener_celda_sucia_mas_cercana()
        print(str(self.unique_id), ": ",  pos_sucia_mas_cercana)

        vecinos = self.model.grid.get_neighbors(
                self.pos, moore=True, include_center=False)

        vecinos_con_prioridad = self.obtener_prioridad_de_vecinos(vecinos, pos_sucia_mas_cercana)
        
        for vecino in vecinos_con_prioridad:
            if not isinstance(vecino, (Mueble, RobotLimpieza)):
                self.sig_pos = vecino.pos
                return

        # print("celda sucia: ", celda_sucia_mas_cercana)
        self.sig_pos = self.random.choice(lista_de_vecinos).pos

    @staticmethod
    def buscar_celdas_sucia(lista_de_vecinos):
        # #Opción 1
        # return [vecino for vecino in lista_de_vecinos
        #                 if isinstance(vecino, Celda) and vecino.sucia]
        # #Opción 2
        celdas_sucias = list()
        for vecino in lista_de_vecinos:
            if isinstance(vecino, Celda) and vecino.sucia:
                celdas_sucias.append(vecino)
        return celdas_sucias

    def step(self):
        #Checar cuanta pila tiene
        if self.carga <= 25:
            self.mover_a_carga()

        else: 
            vecinos = self.model.grid.get_neighbors(
                self.pos, moore=True, include_center=False)

            for vecino in vecinos:
                if isinstance(vecino, (Mueble, RobotLimpieza)):
                    vecinos.remove(vecino)

            celdas_sucias = self.buscar_celdas_sucia(vecinos)

            if len(celdas_sucias) == 0:
                self.seleccionar_nueva_pos(vecinos)
            else:
                self.limpiar_una_celda(celdas_sucias)

    def mover_a_carga(self):
        # Encuentra la estación de carga más cercana y se mueve hacia ella
        estaciones_de_carga = [agent for agent in self.model.schedule.agents if isinstance(agent, Carga)]
        estacion_mas_cercana = min(estaciones_de_carga, key=lambda est: self.calc_dist(self.pos, est.pos))
        self.sig_pos = estacion_mas_cercana.pos
   
    def advance(self):
        if self.pos != self.sig_pos:
            self.movimientos += 1

        if self.carga > 0:
            self.carga -= 1
            self.model.grid.move_agent(self, self.sig_pos)

        if isinstance(self.model.grid.get_cell_list_contents(self.pos), Carga):
            self.carga += 25
            self.carga = min(self.carga, 100)


class Habitacion(Model):
    def __init__(self, M: int, N: int,
                 num_agentes: int = 5,
                 porc_celdas_sucias: float = 0.6,
                 porc_muebles: float = 0.1,
                 modo_pos_inicial: str = 'Fija',
                 ):

        super().__init__()
        self.current_id = 0

        self.num_agentes = num_agentes
        self.porc_celdas_sucias = porc_celdas_sucias
        self.porc_muebles = porc_muebles

        self.grid = MultiGrid(M, N, False)
        self.schedule = SimultaneousActivation(self)

        posiciones_disponibles = [pos for _, pos in self.grid.coord_iter()]

        
        # Posicionamiento de las estaciones de carga
        posiciones_estaciones_carga = [(0, 0), (0, N-1), (M-1, 0), (M-1, N-1)]
        for pos in posiciones_estaciones_carga:
            # Crear un identificador único a partir de las coordenadas de la celda
            unique_id = int(f"{num_agentes}0{pos[0]}0{pos[1]}")
            celda_carga = Celda(unique_id + 1, self, es_estacion_carga=True)
            self.grid.place_agent(celda_carga, pos)
            posiciones_disponibles.remove(pos)

        # Posicionamiento de muebles
        num_muebles = int(M * N * porc_muebles)
        posiciones_muebles = self.random.sample(posiciones_disponibles, k=num_muebles)

        for id, pos in enumerate(posiciones_muebles):
            mueble = Mueble(int(f"{num_agentes}0{id}") + 1, self)
            self.grid.place_agent(mueble, pos)
            posiciones_disponibles.remove(pos)

        # Posicionamiento de celdas sucias
        self.num_celdas_sucias = int(M * N * porc_celdas_sucias)
        posiciones_celdas_sucias = self.random.sample(
            posiciones_disponibles, k=self.num_celdas_sucias)

        for id, pos in enumerate(posiciones_disponibles):
            suciedad = pos in posiciones_celdas_sucias
            celda = Celda(int(f"{num_agentes}{id}") + 1, self, suciedad)
            self.grid.place_agent(celda, pos)

        # Posicionamiento de agentes robot
        if modo_pos_inicial == 'Aleatoria':
            pos_inicial_robots = self.random.sample(posiciones_disponibles, k=num_agentes)
        else:  # 'Fija'
            pos_inicial_robots = [(1, 1)] * num_agentes

        for id in range(num_agentes):
            robot = RobotLimpieza(id, self)
            self.grid.place_agent(robot, pos_inicial_robots[id])
            self.schedule.add(robot)

        self.datacollector = DataCollector(
            model_reporters={"Grid": get_grid, "Cargas": get_cargas,
                             "CeldasSucias": get_sucias},
        )

    def next_id(self):
        self.current_id += 1
        return self.current_id

    def step(self):
        self.datacollector.collect(self)

        self.schedule.step()

    def todoLimpio(self):
        for (content, x, y) in self.grid.coord_iter():
            for obj in content:
                if isinstance(obj, Celda) and obj.sucia:
                    return False
        return True


def get_grid(model: Model) -> np.ndarray:
    """
    Método para la obtención de la grid y representarla en un notebook
    :param model: Modelo (entorno)
    :return: grid
    """
    grid = np.zeros((model.grid.width, model.grid.height))
    for cell in model.grid.coord_iter():
        cell_content, pos = cell
        x, y = pos
        for obj in cell_content:
            if isinstance(obj, RobotLimpieza):
                grid[x][y] = 2
            elif isinstance(obj, Celda):
                grid[x][y] = int(obj.sucia)
    return grid


def get_cargas(model: Model):
    return [(agent.unique_id, agent.carga) for agent in model.schedule.agents]


def get_sucias(model: Model) -> int:
    """
    Método para determinar el número total de celdas sucias
    :param model: Modelo Mesa
    :return: número de celdas sucias
    """
    sum_sucias = 0
    for cell in model.grid.coord_iter():
        cell_content, pos = cell
        for obj in cell_content:
            if isinstance(obj, Celda) and obj.sucia:
                sum_sucias += 1
    return sum_sucias / model.num_celdas_sucias


def get_movimientos(agent: Agent) -> dict:
    if isinstance(agent, RobotLimpieza):
        return {agent.unique_id: agent.movimientos}
    # else:
    #    return 0
