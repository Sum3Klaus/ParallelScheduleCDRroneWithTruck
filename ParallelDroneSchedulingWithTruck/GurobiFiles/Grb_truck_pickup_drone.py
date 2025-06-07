# -*- coding: utf-8 -*-
# @Time     : 2025-05-07-10:03
# @Author   : Sum3 TEO
# @E-mail   : rui3zhang@163
import math
from gurobipy import *
from InsData.Instance import Instance
from typing import Dict
from Common import get_truck_and_drone_routes, get_serve_time


class Grb(object):

    def __init__(self,
                 ins: Instance):

        self._ins = ins
        self.model = Model('parallel drone with truck')
        self.model.setParam('FuncPieces', 100)
        self.model.params.NonConvex = 2

        # self.vars: Dict[tuple, Var] = dict()
        self.X: Dict[tuple, Var] = dict()
        self.Xi: Dict[tuple, Var] = dict()
        self.Zeta: Dict[int, Var] = dict()
        self.Tau_t: Dict[tuple, Var] = dict()
        self.Tau_d: Dict[tuple, Var] = dict()
        self.Phi: Dict[tuple, Var] = dict()
        self.Q_c: Dict[int, Var] = dict()
        self.Gamma_t: Dict[tuple, Var] = dict()
        self.Gamma_d: Dict[tuple, Var] = dict()
        self.Eta_t: Dict[tuple, Var] = dict()
        self.Eta_d: Dict[tuple, Var] = dict()
        self.Z: Dict[tuple, Var] = dict()     # denote c-drone wait for truck at node i

        # power consumption
        self.averageW: Dict[tuple, Var] = dict()

        self.cons: Dict[str, Constr] = dict()

        self.obj = 0
        self.obj_1 = LinExpr()
        self.obj_2 = LinExpr()
        self.satis_var_time: Dict[tuple, Var] = dict()
        self.satis_var_t: Dict[int, Var] = dict()
        self.satis_var_d: Dict[int, Var] = dict()
        self.satis_con: Dict[tuple, Constr] = dict()

        # auxiliary variables
        self.a_v: Dict[tuple, Var] = dict()  # = (W+ω)*g
        self.a_v_to: Dict[tuple, Var] = dict()  # = v_v/2)^2 + t/k1^2
        self.a_v_to_sqrt: Dict[tuple, Var] = dict()  # = sqrt( (v_v/2)^2 + t/k1^2 )
        self.a_v_ld: Dict[tuple, Var] = dict()  # = v_v/2)^2 + t/k1^2
        self.a_v_ld_sqrt: Dict[tuple, Var] = dict()  # = sqrt( (v_v/2)^2 + t/k1^2 )
        self.a_v_pow: Dict[tuple, Var] = dict()  # = (t)^(3/2)
        self.a_v_cr: Dict[tuple, Var] = dict()  #
        self.a_v_squared: Dict[tuple, Var] = dict()  #
        self.a_v_pow2: Dict[tuple, Var] = dict()  # 3/4

        self.arriveTime: Dict[tuple, Var] = dict()

    def build_model(self):
        self._add_vars()
        self._set_obj()
        self._add_cons()

    def solve_and_print(self):
        import re
        self.model.optimize()

        # print
        if self.model.SolCount > 0:

            print('*' * 30, 'truck routing', '*' * 30)
            for x in self.X.keys():
                if self.X[x].x >= 0.95:
                    string = self.X[x].varName
                    idx = re.findall(r'\d+', string)
                    if not (int(idx[0]) == 0 and int(idx[1]) == self._ins.cusNum + 1):
                        print(f'{self.X[x].varName}={self.X[x].x}')

            routes_t, routes_d = get_truck_and_drone_routes(grb=self,
                                                            ins=self._ins)
            serve_time_truck, serve_time_drone, distances_truck, distances_drone, satisfaction = get_serve_time(
                grb=self,
                ins=self._ins,
                routes_truck=routes_t,
                routes_drone=routes_d)
            print(routes_t)
            print('*' * 30, 'truck distance', '*' * 30)
            print(distances_truck)
            print('*' * 30, 'truck serve time', '*' * 30)
            print(serve_time_truck)

            print('*' * 30, 'c-drones routing', '*' * 30)
            for xi in self.Xi.keys():
                if self.Xi[xi].x >= 0.95:
                    string = self.Xi[xi].varName
                    idx = re.findall(r'\d+', string)
                    if not (int(idx[0]) == 0 and int(idx[1]) == self._ins.cusNum + 1):
                        print(f'{self.Xi[xi].varName}={self.Xi[xi].x}')
            for c, route in routes_d.items():
                print(f'c-drone[{c}]-drone num={self.Zeta[c].x}:{route}')
            print('*' * 30, 'truck distance', '*' * 30)
            print(distances_drone)
            print('*' * 30, 'c-drones serve time', '*' * 30)
            print(serve_time_drone)

            print('*' * 30, 'c-drones endpoint', '*' * 30)
            for z in self.Z.keys():
                if self.Z[z].x >= 0.95:
                    print(f'{self.Z[z].varName}={self.Z[z].x}')

            print('*' * 30, 'customer assignment', '*' * 30)
            for eta in self.Eta_t.keys():
                if self.Eta_t[eta].x >= 0.95:
                    print(f'{self.Eta_t[eta].varName}={self.Eta_t[eta].x}')

            for eta in self.Eta_d.keys():
                if self.Eta_d[eta].x >= 0.95:
                    print(f'{self.Eta_d[eta].varName}={self.Eta_d[eta].x}')
            print('*' * 30, 'satisfaction', '*' * 30)
            print(satisfaction)

    def write_lp(self):
        self.model.write('parallel_c_drone.lp')

    def check_arc_feasible(self,
                           head_ver: int,
                           tail_ver: int,
                           t_or_d: int):
        """ t_or_d: 0 is truck, 1 is drone """
        con1 = (head_ver, tail_ver) in self._ins.graph.arcDict.keys()
        if t_or_d == 0:
            if con1 and self._ins.graph.arcDict[head_ver, tail_ver].adj_truck == 1:
                feasible = True
            else:
                feasible = False

        else:
            if con1 and self._ins.graph.arcDict[head_ver, tail_ver].adj_drone == 1:
                feasible = True
            else:
                feasible = False

        return feasible

    def _calc_power_consumption(self,
                                travel_time: float,
                                average_weight: Var):
        """  """
        v_to = self._ins.para.droneTakeoffSpeed
        v_ld = self._ins.para.droneLandSpeed
        p_to = (self._ins.para.k_1 * (self._ins.para.w_drone + self._ins.para.w_battery + average_weight) *
                self._ins.para.gravity * (v_to / 2 + math.sqrt((v_to / 2) ** 2 + ((
                                                                                          self._ins.para.w_drone + self._ins.para.w_battery + average_weight) * self._ins.para.gravity) / self._ins.para.k_2 ** 2)) + self._ins.para.c_2 * (
                        (
                                self._ins.para.w_drone + self._ins.para.w_battery + average_weight) * self._ins.para.gravity) ** (
                        3 / 2))
        p_ld = (self._ins.para.k_1 * (self._ins.para.w_drone + self._ins.para.w_battery + average_weight) *
                self._ins.para.gravity * (v_ld / 2 + math.sqrt((v_ld / 2) ** 2 + ((
                                                                                          self._ins.para.w_drone + self._ins.para.w_battery + average_weight) * self._ins.para.gravity) / self._ins.para.k_2 ** 2)) + self._ins.para.c_2 * (
                        (
                                self._ins.para.w_drone + self._ins.para.w_battery + average_weight) * self._ins.para.gravity) ** (
                        3 / 2))
        p_to_and_ld = p_to * self._ins.to_time + p_ld * self._ins.ld_time

        p_c = (self._ins.para.c_1 + self._ins.para.c_2) * (((
                                                                    self._ins.para.w_drone + self._ins.para.w_battery + average_weight) * self._ins.para.gravity - self._ins.para.c_5 * (
                                                                    self._ins.para.droneCruiseSpeed * math.cos(
                                                                self._ins.para.alpha)) ** 2) ** 2) ** (3 / 4) + (
                      self._ins.para.c_4 * (self._ins.para.droneCruiseSpeed ** 3))
        p_cruise = p_c * travel_time

        return p_to_and_ld + p_cruise

    def _add_vars(self):
        pass

    def _set_obj(self):
        pass

    def _add_cons(self):
        big_m_t = 0
        big_m_d = 0
        for arc in self._ins.graph.arcDict:
            i = arc[0]
            j = arc[1]
            if self._ins.graph.arcDict[arc].adj_truck == 1:
                big_m_t = max(
                    self._ins.graph.vertexDict[i].hardDueTime + self._ins.graph.arcDict[i, j].truckTravelTime -
                    self._ins.graph.vertexDict[i].hardReadyTime, big_m_t)
            if self._ins.graph.arcDict[arc].adj_drone == 1:
                big_m_d = max(
                    self._ins.graph.vertexDict[i].hardDueTime + self._ins.graph.arcDict[i, j].droneTravelTime -
                    self._ins.graph.vertexDict[i].hardReadyTime, big_m_d)
