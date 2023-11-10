from mesa.model import Model
from mesa.agent import Agent
from mesa.space import MultiGrid
from mesa.time import SimultaneousActivation
from mesa.datacollection import DataCollector

import numpy as np
import math


class Celda(Agent):
    def __init__(self, unique_id, model, suciedad: bool = False):
        super().__init__(unique_id, model)
        self.sucia = suciedad


class Mueble(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)

class Cargador(Agent):
    # Clase que carga un robot cuando este se posiciona sobre el
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)

    def cargar_robot(self, robot):
        if self.pos == robot.pos:
            # Asegurarse de que la carga no exceda el 100%
            robot.carga = min(robot.carga + 25, 100)


class RobotLimpieza(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)
        self.sig_pos = None
        self.movimientos = 0
        self.carga = 100
        self.estado = "limpiando"
        self.last_pos = []

    def limpiar_una_celda(self, lista_de_celdas_sucias):
        celda_a_limpiar = self.random.choice(lista_de_celdas_sucias)
        celda_a_limpiar.sucia = False
        self.sig_pos = celda_a_limpiar.pos

    def seleccionar_nueva_pos(self, lista_de_vecinos):
        self.sig_pos = self.random.choice(lista_de_vecinos).pos
        # new_pos = self.random.choice(lista_de_vecinos).pos
        # while new_pos in self.last_pos:
        #     new_pos = self.random.choice(lista_de_vecinos).pos
        # self.last_pos.append(new_pos)
        # self.last_pos.pop(0) if len(self.last_pos) > 5 else None
        # self.sig_pos = new_pos

    @staticmethod
    def buscar_celdas_sucia(lista_de_vecinos):
        # #Opción 1
        return [vecino for vecino in lista_de_vecinos
                        if isinstance(vecino, Celda) and vecino.sucia]
        # #Opción 2
        # celdas_sucias = list()
        # for vecino in lista_de_vecinos:
        #     if isinstance(vecino, Celda) and vecino.sucia:
        #         celdas_sucias.append(vecino)
        # return celdas_sucias
        # celdas_sucias = list()
        # for vecino in lista_de_vecinos:
        #     if isinstance(vecino, RobotLimpieza) or isinstance(vecino, Mueble) or isinstance(vecino, Cargador):
        #         celdas_sucias.append(vecino)
        # return celdas_sucias
            

    def buscar_cargador_mas_cercano(self):
        cargadores = self.model.cargadores
        distancias = [self.model.get_distance(self.pos, cargador.pos) for cargador in cargadores]
        cargador_cercano = cargadores[distancias.index(min(distancias))]
        return cargador_cercano

    def step(self):
        if self.estado == "limpiando":
            vecinos = self.model.grid.get_neighbors(
                self.pos, moore=True, include_center=False)
    
            vecinos = [vecino for vecino in vecinos if not isinstance(vecino, (Mueble, RobotLimpieza))]
    
            celdas_sucias = self.buscar_celdas_sucia(vecinos)
    
            if len(celdas_sucias) == 0:
                self.seleccionar_nueva_pos(vecinos)
            else:
                self.limpiar_una_celda(celdas_sucias)
            
            # Comprobar si necesita recargar
            if self.carga <= 25:
                self.estado = "cargando"
                cargador_cercano = self.buscar_cargador_mas_cercano()
                self.sig_pos = cargador_cercano.pos
        elif self.estado == "cargando":
            # Si ya está en el cargador, no necesita moverse
            if self.pos == self.sig_pos:
                cargador = self.model.grid.get_cell_list_contents([self.pos])[0]
                if isinstance(cargador, Cargador):
                    cargador.cargar_robot(self)
                    if self.carga >= 100:
                        self.estado = "limpiando"
            else:
                # Mover hacia el cargador más cercano
                self.seleccionar_nueva_pos([self.buscar_cargador_mas_cercano()])


    def advance(self):
        if self.pos != self.sig_pos:
            self.movimientos += 1

        if self.carga > 0:
            self.carga -= 1
            self.model.grid.move_agent(self, self.sig_pos)


class Habitacion(Model):
    def __init__(self, M: int, N: int,
                 num_agentes: int = 5,
                 porc_celdas_sucias: float = 0.6,
                 porc_muebles: float = 0.1,
                 modo_pos_inicial: str = 'Fija',
                 ):

        self.cargadores = []
        self.num_agentes = num_agentes
        self.porc_celdas_sucias = porc_celdas_sucias
        self.porc_muebles = porc_muebles

        self.grid = MultiGrid(M, N, False)
        self.schedule = SimultaneousActivation(self)

        posiciones_disponibles = [pos for _, pos in self.grid.coord_iter()]

        cargador1 = Cargador(101, self)
        self.grid.place_agent(cargador1, (0, 0))
        posiciones_disponibles.remove(cargador1.pos)
        self.cargadores.append(cargador1)  # Añadir a la lista de cargadores


        cargador2 = Cargador(102, self)
        self.grid.place_agent(cargador2, (M - 1, N - 1))
        posiciones_disponibles.remove(cargador2.pos)
        self.cargadores.append(cargador2)

        cargador3 = Cargador(103, self)
        self.grid.place_agent(cargador3, (M - 1, 0))
        posiciones_disponibles.remove(cargador3.pos)
        self.cargadores.append(cargador3)

        cargador4 = Cargador(104, self)
        self.grid.place_agent(cargador4, (0, N - 1))
        posiciones_disponibles.remove(cargador4.pos)
        self.cargadores.append(cargador4)

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

    def step(self):
        self.datacollector.collect(self)

        self.schedule.step()

    def todoLimpio(self):
        for (content, x, y) in self.grid.coord_iter():
            for obj in content:
                if isinstance(obj, Celda) and obj.sucia:
                    return False
        return True
    
    def get_distance(self, pos_1, pos_2):
        """
        Calcula la distancia euclidiana entre dos posiciones en la grilla.
        """
        x1, y1 = pos_1
        x2, y2 = pos_2
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


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
