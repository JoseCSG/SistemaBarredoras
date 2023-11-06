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
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)

    def cargar_robot(self, robot):
        if (self.pos == robot.pos):
            # Add 25% each step
            robot.carga += 25

    def step(self):
        for agent in self.model.schedule.agents:
            if isinstance(agent, RobotLimpieza):
                self.cargar_robot(agent)


class RobotLimpieza(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)
        self.sig_pos = None
        self.movimientos = 0
        self.carga = 100
        self.last_pos = []

    def necesita_cargar(self):
        return self.carga < 25

    def limpiar_una_celda(self, lista_de_celdas_sucias):
        celda_a_limpiar = self.random.choice(lista_de_celdas_sucias)
        celda_a_limpiar.sucia = False
        self.sig_pos = celda_a_limpiar.pos

    def seleccionar_nueva_pos(self, lista_de_vecinos):
        # self.sig_pos = self.random.choice(lista_de_vecinos).pos
        new_pos = self.random.choice(lista_de_vecinos).pos
        while new_pos in self.last_pos:
            new_pos = self.random.choice(lista_de_vecinos).pos
        self.last_pos.append(new_pos)
        self.sig_pos = new_pos

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
    
    def distancia_entre_posiciones(self, pos1, pos2):
        x1, y1 = pos1
        x2, y2 = pos2
        return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

    def step(self):
        vecinos = self.model.grid.get_neighbors(
            self.pos, moore=True, include_center=False)

        for vecino in vecinos:
            if isinstance(vecino, RobotLimpieza) or isinstance(vecino, Mueble):
                vecinos.remove(vecino)

        celdas_sucias = self.buscar_celdas_sucia(vecinos)

        if len(celdas_sucias) == 0:
            if self.necesita_cargar():
                # Encuentra un cargador más cercano
                cargadores = [vecino for vecino in vecinos if isinstance(vecino, Cargador)]
                if cargadores:
                    cargador_mas_cercano = min(cargadores, key=lambda c: self.distancia_entre_posiciones(self.pos, c.pos))
                    self.sig_pos = cargador_mas_cercano.pos
                    if (self.pos == cargador_mas_cercano.pos):
                        self.carga = 100
                else:
                    self.seleccionar_nueva_pos(vecinos)
            else:
                self.seleccionar_nueva_pos(vecinos)
        else:
            self.limpiar_una_celda(celdas_sucias)
        
        # vecinos = self.model.grid.get_neighbors(
        #     self.pos, moore=True, include_center=False)

        # for vecino in vecinos:
        #     if isinstance(vecino, RobotLimpieza) or isinstance(vecino, Mueble):
        #         vecinos.remove(vecino)

        # celdas_sucias = self.buscar_celdas_sucia(vecinos)

        # if len(celdas_sucias) == 0:
        #     self.seleccionar_nueva_pos(vecinos)
        # else:
        #     self.limpiar_una_celda(celdas_sucias)

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

        self.num_agentes = num_agentes
        self.porc_celdas_sucias = porc_celdas_sucias
        self.porc_muebles = porc_muebles

        self.grid = MultiGrid(M, N, False)
        self.schedule = SimultaneousActivation(self)

        posiciones_disponibles = [pos for _, pos in self.grid.coord_iter()]

        cargador1 = Cargador(1, self)
        self.grid.place_agent(cargador1, (0, 0))
        posiciones_disponibles.remove(cargador1.pos)

        cargador2 = Cargador(2, self)
        self.grid.place_agent(cargador2, (M - 1, N - 1))
        posiciones_disponibles.remove(cargador2.pos)

        cargador3 = Cargador(3, self)
        self.grid.place_agent(cargador3, (M - 1, 0))
        posiciones_disponibles.remove(cargador3.pos)

        cargador4 = Cargador(4, self)
        self.grid.place_agent(cargador4, (0, N - 1))
        posiciones_disponibles.remove(cargador4.pos)

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
