# -*- coding: utf-8 -*-
# @Time     : 2025-05-07-10:03
# @Author   : Sum3 TEO
# @E-mail   : rui3zhang@163
import math
import re
from gurobipy import *
from InsData.Instance import Instance
from typing import Dict


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
        self.Z: Dict[tuple, Var] = dict()

        # power consumption
        self.averageW: Dict[tuple, Var] = dict()

        self.cons: Dict[str, Constr] = dict()

        self.obj = 0
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

    def build_model(self):
        self._add_vars()
        self._set_obj()
        self._add_cons()

    def solve_and_print(self):
        self.model.optimize()

        # print
        if self.model.SolCount > 0:
            for x in self.X.keys():
                if self.X[x].x >= 0.95:
                    string = self.X[x].varName
                    idx = re.findall(r'\d+', string)
                    if not (int(idx[0]) == 0 and int(idx[1]) == self._ins.cusNum + 1):
                        print(f'{self.X[x].varName}={self.X[x].x}')

            for xi in self.Xi.keys():
                if self.Xi[xi].x >= 0.95:
                    string = self.Xi[xi].varName
                    idx = re.findall(r'\d+', string)
                    if not (int(idx[0]) == 0 and int(idx[1]) == self._ins.cusNum + 1):
                        print(f'{self.Xi[xi].varName}={self.Xi[xi].x}')

            for z in self.Z.keys():
                if self.Z[z].x >= 0.95:
                    print(f'{self.Z[z].varName}={self.Z[z].x}')

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

    def _add_vars(self):
        for c in range(self._ins.para.droneMaxGroup):
            self.Zeta[c] = self.model.addVar(lb=0.0,
                                             ub=self._ins.para.L,
                                             obj=0,
                                             vtype=GRB.INTEGER,
                                             column=None,
                                             name='zeta_' + str(c))

        for i in range(self._ins.cusNum + 1):
            for j in range(self._ins.cusNum + 2):
                if self.check_arc_feasible(head_ver=i,
                                           tail_ver=j,
                                           t_or_d=0):
                    for t in range(self._ins.truckNum):
                        self.X[i, j, t] = self.model.addVar(lb=0.0,
                                                            ub=1.0,
                                                            obj=0,
                                                            vtype=GRB.BINARY,
                                                            column=None,
                                                            name='x_' + str(i) + '_' + str(j) + '_' + str(t))

        for i in self._ins.graph.cus_drone + [0]:
            for j in range(self._ins.cusNum + 2):
                if self.check_arc_feasible(head_ver=i,
                                           tail_ver=j,
                                           t_or_d=1):
                    for c in range(self._ins.para.droneMaxGroup):
                        self.Xi[i, j, c] = self.model.addVar(lb=0.0,
                                                             ub=1.0,
                                                             obj=0,
                                                             vtype=GRB.BINARY,
                                                             column=None,
                                                             name='xi_' + str(i) + '_' + str(j) + '_' + str(c))

        for i in range(self._ins.cusNum + 2):
            for t in range(self._ins.truckNum):
                self.Tau_t[i, t] = self.model.addVar(lb=self._ins.graph.vertexDict[i].hardReadyTime,
                                                     ub=self._ins.graph.vertexDict[i].hardDueTime,
                                                     obj=0,
                                                     vtype=GRB.CONTINUOUS,
                                                     column=None,
                                                     name='tau_t_' + str(i) + '_' + str(t))

                self.Gamma_t[i, t] = self.model.addVar(lb=0.0,
                                                       ub=self._ins.truckCapacity,
                                                       obj=0,
                                                       vtype=GRB.CONTINUOUS,
                                                       column=None,
                                                       name='gamma_t_' + str(i) + '_' + str(t))

                # for c in range(self._ins.para.droneMaxGroup):
                #     self.Z[i, c, t] = self.model.addVar(lb=0.0,
                #                                         ub=1.0,
                #                                         obj=0,
                #                                         vtype=GRB.BINARY,
                #                                         column=None,
                #                                         name='z_' + str(i) + '_' + str(c) + '_' + str(t))

            for c in range(self._ins.para.droneMaxGroup):
                self.Tau_d[i, c] = self.model.addVar(lb=self._ins.graph.vertexDict[i].hardReadyTime,
                                                     ub=self._ins.graph.vertexDict[i].hardDueTime,
                                                     obj=0,
                                                     vtype=GRB.CONTINUOUS,
                                                     column=None,
                                                     name='tau_d_' + str(i) + '_' + str(c))

                self.Gamma_d[i, c] = self.model.addVar(lb=0.0,
                                                       ub=self._ins.para.L * self._ins.para.singleQ,
                                                       obj=0,
                                                       vtype=GRB.CONTINUOUS,
                                                       column=None,
                                                       name='gamma_d_' + str(i) + '_' + str(c))

                self.Phi[i, c] = self.model.addVar(lb=0.0,
                                                   ub=self._ins.para.B,
                                                   obj=0,
                                                   vtype=GRB.CONTINUOUS,
                                                   column=None,
                                                   name='phi_' + str(i) + '_' + str(c))

                self.averageW[i, c] = self.model.addVar(lb=0.0,
                                                        ub=self._ins.para.singleQ,
                                                        obj=0,
                                                        vtype=GRB.CONTINUOUS,
                                                        column=None,
                                                        name='averageWeight_' + str(i) + '_' + str(c))
                con_name = 'average_weight_' + str(i) + '_' + str(c)
                self.cons[con_name] = self.model.addQConstr(
                    self.averageW[i, c] * self.Zeta[c] - self.Gamma_d[i, c] == 0,
                    name=con_name)

                # = (W+ω)*g
                self.a_v[i, c] = self.model.addVar(lb=0.0,
                                                   ub=(self._ins.demand_sum + self._ins.para.w_drone +
                                                       self._ins.para.w_battery) * self._ins.para.gravity,
                                                   obj=0,
                                                   vtype=GRB.CONTINUOUS,
                                                   column=None,
                                                   name='a_v_' + str(i) + '_' + str(c))

                # = v_v/2)^2 + t/k1^2
                self.a_v_to[i, c] = self.model.addVar(lb=0.0,
                                                      ub=1e9,
                                                      obj=0,
                                                      vtype=GRB.CONTINUOUS,
                                                      column=None,
                                                      name='a_v_to_' + str(i) + '_' + str(c))
                self.a_v_ld[i, c] = self.model.addVar(lb=0.0,
                                                      ub=1e9,
                                                      obj=0,
                                                      vtype=GRB.CONTINUOUS,
                                                      column=None,
                                                      name='a_v_ld_' + str(i) + '_' + str(c))

                # = sqrt( (v_v/2)^2 + t/k1^2 )
                self.a_v_to_sqrt[i, c] = self.model.addVar(lb=0.0,
                                                           ub=1e9,
                                                           obj=0,
                                                           vtype=GRB.CONTINUOUS,
                                                           column=None,
                                                           name='a_v_to_sqrt_' + str(i) + '_' + str(c))
                self.a_v_ld_sqrt[i, c] = self.model.addVar(lb=0.0,
                                                           ub=1e9,
                                                           obj=0,
                                                           vtype=GRB.CONTINUOUS,
                                                           column=None,
                                                           name='a_v_ld_sqrt_' + str(i) + '_' + str(c))
                # = (t)^(3/2)
                self.a_v_pow[i, c] = self.model.addVar(lb=0.0,
                                                       ub=1e9,
                                                       obj=0,
                                                       vtype=GRB.CONTINUOUS,
                                                       column=None,
                                                       name='a_v_pow_' + str(i) + '_' + str(c))

                self.a_v_cr[i, c] = self.model.addVar(lb=0.0,
                                                      ub=1e9,
                                                      obj=0,
                                                      vtype=GRB.CONTINUOUS,
                                                      column=None,
                                                      name='a_v_cr_' + str(i) + '_' + str(c))
                # squared

                self.a_v_squared[i, c] = self.model.addVar(lb=0.0,
                                                           ub=1e9,
                                                           obj=0,
                                                           vtype=GRB.CONTINUOUS,
                                                           column=None,
                                                           name='a_v_squared_' + str(i) + '_' + str(c))
                self.a_v_pow2[i, c] = self.model.addVar(lb=0.0,
                                                        ub=1e9,
                                                        obj=0,
                                                        vtype=GRB.CONTINUOUS,
                                                        column=None,
                                                        name='a_v_pow2_' + str(i) + '_' + str(c))
                self.model.update()
                """ auxiliary variables """
                con_name = 'a_v_' + str(i) + '_' + str(c)
                self.cons[con_name] = self.model.addConstr(self.a_v[i, c] - (
                        self._ins.para.w_drone + self._ins.para.w_battery + self.averageW[
                    i, c]) * self._ins.para.gravity == 0,
                                                           name=con_name)

                con_name = 'a_v_to_' + str(i) + '_' + str(c)
                self.cons[con_name] = self.model.addConstr(
                    self.a_v_to[i, c] - (self._ins.para.droneTakeoffSpeed / 2) ** 2 - (self.a_v[i, c] / (
                            self._ins.para.k_2 ** 2)) == 0,
                    name=con_name)
                con_name = 'a_v_ld_' + str(i) + '_' + str(c)
                self.cons[con_name] = self.model.addConstr(
                    self.a_v_ld[i, c] - (self._ins.para.droneLandSpeed / 2) ** 2 - self.a_v[i, c] / (
                            self._ins.para.k_2 ** 2) == 0,
                    name=con_name)

                con_name = 'a_v_pow_' + str(i) + '_' + str(c)
                self.cons[con_name] = self.model.addGenConstrPow(self.a_v[i, c], self.a_v_pow[i, c], 1.5,
                                                                 name=con_name)

                con_name = 'a_v_to_sqrt_' + str(i) + '_' + str(c)
                self.cons[con_name] = self.model.addGenConstrPow(self.a_v_to[i, c], self.a_v_to_sqrt[i, c], 0.5,
                                                                 name=con_name)
                con_name = 'a_v_ld_sqrt_' + str(i) + '_' + str(c)
                self.cons[con_name] = self.model.addGenConstrPow(self.a_v_ld[i, c], self.a_v_ld_sqrt[i, c], 0.5,
                                                                 name=con_name)

                con_name = 'a_v_cr_' + str(i) + '_' + str(c)
                self.cons[con_name] = self.model.addConstr(
                    self.a_v_cr[i, c] - self.a_v[i, c] - self._ins.para.c_5 * (
                            (self._ins.para.droneCruiseSpeed * math.cos(10)) ** 2) == 0,
                    name=con_name
                )
                con_name = 'a_v_squared_' + str(i) + '_' + str(c)
                self.cons[con_name] = self.model.addGenConstrPow(self.a_v_squared[i, c], self.a_v_cr[i, c], 2,
                                                                 name=con_name)
                con_name = 'a_v_pow2_' + str(i) + '_' + str(c)
                self.cons[con_name] = self.model.addGenConstrPow(self.a_v_squared[i, c], self.a_v_pow2[i, c], 0.75,
                                                                 name=con_name)

                self.Z[i, c] = self.model.addVar(lb=0.0,
                                                 ub=1.0,
                                                 obj=0,
                                                 vtype=GRB.BINARY,
                                                 column=None,
                                                 name='z_' + str(i) + '_' + str(c))

        for c in range(self._ins.para.droneMaxGroup):
            self.Q_c[c] = self.model.addVar(lb=0.0,
                                            ub=self._ins.para.L * self._ins.para.singleQ,
                                            obj=0,
                                            vtype=GRB.CONTINUOUS,
                                            column=None,
                                            name='q_c_' + str(c))

        self.model.update()

    def _set_obj(self):
        """ obj1: travel cost and drone fixed cost """
        obj_1 = LinExpr()
        obj_1_coe = list()
        obj_1_var = list()

        for i in range(self._ins.cusNum + 1):
            for j in range(self._ins.cusNum + 2):
                if self.check_arc_feasible(head_ver=i,
                                           tail_ver=j,
                                           t_or_d=0):
                    for t in range(self._ins.truckNum):
                        obj_1_coe.append(self._ins.graph.arcDict[i, j].mDistance / 1000)
                        obj_1_var.append(self.X[i, j, t])
            for j in self._ins.graph.cus_drone:
                if self.check_arc_feasible(head_ver=i,
                                           tail_ver=j,
                                           t_or_d=1):
                    for c in range(self._ins.para.droneMaxGroup):
                        obj_1_coe.append(self._ins.graph.arcDict[i, j].eDistance / 1000 * self._ins.para.cost_2)
                        obj_1_var.append(self.Xi[i, j, c])

        for c in range(self._ins.para.droneMaxGroup):
            obj_1_coe.append(self._ins.para.cost_1)
            obj_1_var.append(self.Zeta[c])
        obj_1.addTerms(obj_1_coe, obj_1_var)

        """ obj2: satisfaction, PWL """
        obj_2 = LinExpr()
        obj_2_coe = list()
        obj_2_var = list()
        for i in range(1, self._ins.cusNum + 1):
            cur_time_point = [self._ins.graph.vertexDict[i].hardReadyTime, self._ins.graph.vertexDict[i].softReadyTime,
                              self._ins.graph.vertexDict[i].softDueTime, self._ins.graph.vertexDict[i].hardDueTime]

            satis_point = [0, 1, 1, 0]

            self.satis_var_t[i] = self.model.addVar(lb=0.0,
                                                    ub=1.0,
                                                    obj=0,
                                                    vtype=GRB.INTEGER,
                                                    column=None,
                                                    name='satisfaction_t_' + str(i))

            self.satis_var_d[i] = self.model.addVar(lb=0.0,
                                                    ub=1.0,
                                                    obj=0,
                                                    vtype=GRB.INTEGER,
                                                    column=None,
                                                    name='satisfaction_d_' + str(i))

            for t in range(self._ins.truckNum):
                self.satis_con[i, t] = self.model.addGenConstrPWL(self.Tau_t[i, t], self.satis_var_t[i],
                                                                  cur_time_point, satis_point,
                                                                  name="satis_t_" + str(i) + '_' + str(t))
            obj_2_coe.append(1)
            obj_2_var.append(self.satis_var_t[i])

            for c in range(self._ins.para.droneMaxGroup):
                self.satis_con[i, c] = self.model.addGenConstrPWL(self.Tau_d[i, c], self.satis_var_d[i],
                                                                  cur_time_point, satis_point,
                                                                  name="satis_d_" + str(i) + '_' + str(c))
            obj_2_coe.append(1)
            obj_2_var.append(self.satis_var_d[i])
            obj_2.addTerms(obj_2_coe, obj_2_var)

        self.obj = obj_1 + obj_2
        self.model.setObjective(self.obj, GRB.MINIMIZE)

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

    def _add_cons(self):
        """ con.1: must serve customer """
        for i in range(1, self._ins.cusNum + 1):
            lhs = LinExpr()
            coe = list()
            var = list()

            # truck
            for j in range(self._ins.cusNum + 2):
                if self.check_arc_feasible(head_ver=i,
                                           tail_ver=j,
                                           t_or_d=0):
                    for t in range(self._ins.truckNum):
                        coe.append(1)
                        var.append(self.X[i, j, t])

            # c-drone
            if i in self._ins.graph.cus_drone:
                for j in range(self._ins.cusNum + 2):
                    if self.check_arc_feasible(head_ver=i,
                                               tail_ver=j,
                                               t_or_d=1):
                        for c in range(self._ins.para.droneMaxGroup):
                            coe.append(1)
                            var.append(self.Xi[i, j, c])

            lhs.addTerms(coe, var)
            con_name = 'con1_' + str(i)
            self.cons[con_name] = self.model.addConstr(lhs >= 1,
                                                       name=con_name)

        # new 3-d var_Z
        # for l in range(1, self._ins.cusNum + 1):
        #     lhs = LinExpr()
        #     coe = list()
        #     var = list()
        #     # truck
        #     for i in range(self._ins.cusNum + 1):
        #         if self.check_arc_feasible(head_ver=i,
        #                                    tail_ver=l,
        #                                    t_or_d=0):
        #             for t in range(self._ins.truckNum):
        #                 coe.append(1)
        #                 var.append(self.X[i, l, t])
        #     lhs.addTerms(coe, var)
        #     # c-drone
        #     lhs1 = LinExpr()
        #     coe1 = list()
        #     var1 = list()
        #     for c in range(self._ins.para.droneMaxGroup):
        #         for i in range(self._ins.cusNum + 1):
        #             if self.check_arc_feasible(head_ver=i,
        #                                        tail_ver=l,
        #                                        t_or_d=1):
        #                 coe1.append(1)
        #                 var1.append(self.Xi[i, l, c])
        #     lhs1.addTerms(coe1, var1)
        #     con_name = 'con1_' + str(l)
        #     self.cons[con_name] = self.model.addConstr(lhs + lhs1 >= 1,
        #                                                name=con_name)
        #     self.model.update()

        """ con.2: truck must start from depot """
        for t in range(self._ins.truckNum):
            lhs = LinExpr()
            coe = list()
            var = list()
            for j in range(1, self._ins.cusNum + 2):
                if self.check_arc_feasible(head_ver=0,
                                           tail_ver=j,
                                           t_or_d=0):
                    coe.append(1)
                    var.append(self.X[0, j, t])
            lhs.addTerms(coe, var)
            con_name = 'con2_' + str(t)
            self.cons[con_name] = self.model.addConstr(lhs == 1,
                                                       name=con_name)

            """ con.3: truck must return to depot """
            lhs = LinExpr()
            coe = list()
            var = list()
            for i in range(self._ins.cusNum + 1):
                if self.check_arc_feasible(head_ver=i,
                                           tail_ver=self._ins.cusNum + 1,
                                           t_or_d=0):
                    coe.append(1)
                    var.append(self.X[i, self._ins.cusNum + 1, t])
            lhs.addTerms(coe, var)
            con_name = 'con3_' + str(t)
            self.cons[con_name] = self.model.addConstr(lhs == 1,
                                                       name=con_name)

        """ con.4: c-drone must start from depot """
        for c in range(self._ins.para.droneMaxGroup):
            lhs = LinExpr()
            coe = list()
            var = list()
            for j in self._ins.graph.cus_drone + [self._ins.cusNum + 1]:
                if self.check_arc_feasible(head_ver=0,
                                           tail_ver=j,
                                           t_or_d=1):
                    coe.append(1)
                    var.append(self.Xi[0, j, c])
            lhs.addTerms(coe, var)
            con_name = 'con4_' + str(c)
            self.cons[con_name] = self.model.addConstr(lhs == 1,
                                                       name=con_name)

            """ con.5: c-drone must return to depot or a truck """
            lhs = LinExpr()
            coe = list()
            var = list()
            for i in self._ins.graph.cus_drone + [0]:
                if self.check_arc_feasible(head_ver=i,
                                           tail_ver=self._ins.cusNum + 1,
                                           t_or_d=1):
                    coe.append(1)
                    var.append(self.Xi[i, self._ins.cusNum + 1, c])

            lhs_1 = LinExpr()
            coe_1 = list()
            var_1 = list()
            for j in range(1, self._ins.cusNum + 1):
                coe_1.append(1)
                var_1.append(self.Z[j, c])
            lhs_1.addTerms(coe_1, var_1)
            lhs.addTerms(coe, var)
            con_name = 'con5_' + str(c)
            self.cons[con_name] = self.model.addConstr(lhs + lhs_1 == 1,
                                                       name=con_name)

            # 3-d var_Z
            # lhs = LinExpr()
            # coe = list()
            # var = list()
            # for i in self._ins.graph.cus_drone + [0]:
            #     if self.check_arc_feasible(head_ver=i,
            #                                tail_ver=self._ins.cusNum + 1,
            #                                t_or_d=1):
            #         coe.append(1)
            #         var.append(self.Xi[i, self._ins.cusNum + 1, c])
            #
            # lhs_1 = LinExpr()
            # coe_1 = list()
            # var_1 = list()
            # for j in range(1, self._ins.cusNum + 1):
            #     for t in range(self._ins.truckNum):
            #         coe_1.append(1)
            #         var_1.append(self.Z[j, c, t])
            # lhs_1.addTerms(coe_1, var_1)
            # lhs.addTerms(coe, var)
            # con_name = 'con5_' + str(c)
            # self.cons[con_name] = self.model.addConstr(lhs + lhs_1 == 1,
            #                                            name=con_name)

            """ con.6: c-drone start and scale """
            lhs = LinExpr()
            coe = list()
            var = list()
            for j in self._ins.graph.cus_drone:
                if self.check_arc_feasible(head_ver=0,
                                           tail_ver=j,
                                           t_or_d=1):
                    coe.append(1)
                    var.append(self.Xi[0, j, c])
            lhs.addTerms(coe, var)
            con_name = 'con6-' + str(c)
            self.cons[con_name] = self.model.addConstr(lhs - self.Zeta[c] <= 0,
                                                       name=con_name)

        """ con.7: c-drone scale ub """
        lhs = LinExpr()
        coe = list()
        var = list()
        for c in range(self._ins.para.droneMaxGroup):
            coe.append(1)
            var.append(self.Zeta[c])
        lhs.addTerms(coe, var)
        con_name = 'con7'
        self.cons[con_name] = self.model.addConstr(lhs - self._ins.para.m <= 0,
                                                   name=con_name)

        """ con.8: truck flow balance """
        for l in range(1, self._ins.cusNum + 1):
            for t in range(self._ins.truckNum):
                lhs = LinExpr()
                coe = list()
                var = list()
                for i in range(self._ins.cusNum + 1):
                    if self.check_arc_feasible(head_ver=i,
                                               tail_ver=l,
                                               t_or_d=0):
                        coe.append(1)
                        var.append(self.X[i, l, t])

                for j in range(1, self._ins.cusNum + 2):
                    if self.check_arc_feasible(head_ver=l,
                                               tail_ver=j,
                                               t_or_d=0):
                        coe.append(-1)
                        var.append(self.X[l, j, t])
                lhs.addTerms(coe, var)
                con_name = 'con8-' + str(l) + '_' + str(t)
                self.cons[con_name] = self.model.addConstr(lhs == 0,
                                                           name=con_name)

        """ con.9: drone flow balance """
        for l in self._ins.graph.cus_drone:
            for c in range(self._ins.para.droneMaxGroup):
                lhs = LinExpr()
                coe = list()
                var = list()
                for i in self._ins.graph.cus_drone + [0]:
                    if self.check_arc_feasible(head_ver=i,
                                               tail_ver=l,
                                               t_or_d=1):
                        coe.append(1)
                        var.append(self.Xi[i, l, c])

                for j in range(1, self._ins.cusNum + 2):
                    if self.check_arc_feasible(head_ver=l,
                                               tail_ver=j,
                                               t_or_d=1):
                        coe.append(-1)
                        var.append(self.Xi[l, j, c])

                # 3-d var_Z
                # for t in range(self._ins.truckNum):
                #     coe.append(-1)
                #     var.append(self.Z[l, c, t])

                lhs.addTerms(coe, var)
                con_name = 'con9-' + str(l) + '_' + str(c)
                self.cons[con_name] = self.model.addConstr(lhs - self.Z[l, c] == 0,
                                                           name=con_name)

        """ con.10: 若无人机返回货车，则无人机到达时间必须早于货车离开时间 """
        for i in self._ins.graph.cus_drone:
            for c in range(self._ins.para.droneMaxGroup):
                for t in range(self._ins.truckNum):
                    for j in range(1, self._ins.cusNum + 1):
                        if self.check_arc_feasible(head_ver=i, tail_ver=j, t_or_d=0) and self.check_arc_feasible(
                                head_ver=i, tail_ver=j, t_or_d=1):
                            con_name = 'con10-' + str(i) + '_' + str(j) + '_' + str(c) + '_' + str(t)
                            self.cons[con_name] = self.model.addConstr(
                                self.Tau_d[i, c] + self._ins.to_time + self._ins.ld_time + self._ins.graph.arcDict[
                                    i, j].droneTravelTime - self.Tau_t[j, t] - 1e6 * (1 - self.Z[j, c]) <= 0,
                                name=con_name)

        """ con.11: Drone retrieval requires truck presence """
        for c in range(self._ins.para.droneMaxGroup):
            for i in range(self._ins.cusNum + 1):
                lhs = LinExpr()
                coe = list()
                var = list()

                for j in range(1, self._ins.cusNum + 2):
                    if self.check_arc_feasible(head_ver=i,
                                               tail_ver=j,
                                               t_or_d=0):
                        for t in range(self._ins.truckNum):
                            coe.append(1)
                            var.append(self.X[i, j, t])
                lhs.addTerms(coe, var)
                con_name = 'con11-' + str(i) + '_' + str(c)
                self.cons[con_name] = self.model.addConstr(self.Z[i, c] - lhs <= 0,
                                                           name=con_name)

        # 3-d var_Z
        # for c in range(self._ins.para.droneMaxGroup):
        #     for l in range(1, self._ins.cusNum + 1):
        #         for t in range(self._ins.truckNum):
        #             lhs = LinExpr()
        #             coe = list()
        #             var = list()
        #             for i in range(1, 1, self._ins.cusNum + 1):
        #                 if self.check_arc_feasible(head_ver=i,
        #                                            tail_ver=l,
        #                                            t_or_d=0):
        #                     coe.append(1)
        #                     var.append(self.X[i, l, t])
        #             lhs.addTerms(coe, var)
        #             con_name = 'con11-' + str(l) + '_' + str(c) + '_' + str(t)
        #             self.cons[con_name] = self.model.addConstr(self.Z[l, c, t] - lhs <= 0,
        #                                                        name=con_name)


        """ con.12: time continuous truck """
        for i in range(self._ins.cusNum + 1):
            for j in range(self._ins.cusNum + 2):
                if self.check_arc_feasible(head_ver=i,
                                           tail_ver=j,
                                           t_or_d=0):
                    for t in range(self._ins.truckNum):
                        con_name = 'con12-' + str(i) + '_' + str(j) + '_' + str(t)
                        self.cons[con_name] = self.model.addConstr(
                            self.Tau_t[i, t] + self._ins.para.serviceTime + self._ins.graph.arcDict[
                                i, j].truckTravelTime - self._ins.time_bgM * (1 - self.X[i, j, t]) - self.Tau_t[
                                j, t] <= 0,
                            name=con_name)

        """ con.13: time continuous drone """
        for i in self._ins.graph.cus_drone + [0]:
            for j in range(self._ins.cusNum + 2):
                if self.check_arc_feasible(head_ver=i,
                                           tail_ver=j,
                                           t_or_d=1):
                    for c in range(self._ins.para.droneMaxGroup):
                        con_name = 'con13-' + str(i) + '_' + str(j) + '_' + str(c)
                        self.cons[con_name] = self.model.addConstr(
                            self.Tau_d[i, c] + self._ins.para.serviceTime + self._ins.to_time + self._ins.ld_time +
                            self._ins.graph.arcDict[i, j].droneTravelTime - self._ins.time_bgM * (
                                    1 - self.Xi[i, j, c]) - self.Tau_d[j, c] <= 0,
                            name=con_name)

        """ con.14: load continuous truck """
        for i in range(self._ins.cusNum + 1):
            for j in range(self._ins.cusNum + 2):
                if self.check_arc_feasible(head_ver=i,
                                           tail_ver=j,
                                           t_or_d=0):
                    for t in range(self._ins.truckNum):
                        con_name = 'con14-' + str(i) + '_' + str(j) + '_' + str(t)
                        self.cons[con_name] = self.model.addConstr(
                            self.Gamma_t[i, t] + self._ins.graph.vertexDict[j].demand - self._ins.truckCapacity * (
                                    1 - self.X[i, j, t]) - self.Gamma_t[j, t] <= 0,
                            name=con_name)

        """ con.15: load of c-drone """
        for i in range(self._ins.cusNum + 2):
            for c in range(self._ins.para.droneMaxGroup):
                con_name = 'con15-' + str(i) + '_' + str(c)
                self.cons[con_name] = self.model.addConstr(
                    self.Gamma_d[i, c] - self.Zeta[c] * self._ins.para.singleQ <= 0,
                    name=con_name)

        """ con.16: load continuous drone """
        for i in self._ins.graph.cus_drone + [0]:
            for j in range(self._ins.cusNum + 2):
                if self.check_arc_feasible(head_ver=i,
                                           tail_ver=j,
                                           t_or_d=1):
                    for c in range(self._ins.para.droneMaxGroup):
                        con_name = 'con16-' + str(i) + '_' + str(j) + '_' + str(c)
                        self.cons[con_name] = self.model.addConstr(
                            self.Gamma_d[i, c] + self._ins.graph.vertexDict[
                                j].demand - self.Zeta[c] * self._ins.para.singleQ * (1 - self.Xi[i, j, c]) -
                            self.Gamma_d[j, c] <= 0,
                            name=con_name)

                        """ con.17: power consumption """
                        con_name = 'con17-' + str(i) + '_' + str(j) + '_' + str(c)
                        power_to = self._ins.para.k_1 * self.a_v[i, c] * (
                                (self._ins.para.droneTakeoffSpeed / 2) + self.a_v_to_sqrt[i, c]) + (
                                           self._ins.para.c_2 * self.a_v_pow[i, c])
                        power_ld = self._ins.para.k_1 * self.a_v[i, c] * (
                                (self._ins.para.droneLandSpeed / 2) + self.a_v_ld[i, c]) + (
                                           self._ins.para.c_2 * self.a_v_pow[i, c])
                        power_cruise = (self._ins.para.c_1 + self._ins.para.c_2) * self.a_v_pow2[
                            i, c] + self._ins.para.c_4 * self._ins.para.droneCruiseSpeed ** 3
                        self.cons[con_name] = self.model.addConstr(
                            self.Phi[i, c] + power_to + power_ld + power_cruise - self.Phi[j, c] - 1e15 * (
                                    1 - self.Xi[i, j, c]) <= 0,
                            name=con_name
                        )

        """ con.18: each customer only can be served by one truck """
        for j in range(1, self._ins.cusNum + 1):
            lhs = LinExpr()
            coe = list()
            var = list()
            for i in range(self._ins.cusNum + 1):
                if self.check_arc_feasible(head_ver=i,
                                           tail_ver=j,
                                           t_or_d=0):
                    for t in range(self._ins.truckNum):
                        coe.append(1)
                        var.append(self.X[i, j, t])

            lhs.addTerms(coe, var)
            con_name = 'con18_' + str(j)
            self.cons[con_name] = self.model.addConstr(
                lhs <= 1,
                name=con_name
            )

        """ con.19: each customer only can be served by one c-drone """
        for j in self._ins.graph.cus_drone:
            lhs = LinExpr()
            coe = list()
            var = list()
            for i in self._ins.graph.cus_drone + [0]:
                if self.check_arc_feasible(head_ver=i,
                                           tail_ver=j,
                                           t_or_d=1):
                    for c in range(self._ins.para.droneMaxGroup):
                        coe.append(1)
                        var.append(self.Xi[i, j, c])

            # for t in range(self._ins.truckNum):
            #     for c in range(self._ins.para.droneMaxGroup):
            #         coe.append(-1)
            #         var.append(self.Z[j, c, t])

            lhs.addTerms(coe, var)
            con_name = 'con19_' + str(j)
            self.cons[con_name] = self.model.addConstr(
                lhs <= 1,
                name=con_name
            )
