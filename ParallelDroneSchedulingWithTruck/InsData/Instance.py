# -*- coding: utf-8 -*-
# @Time     : 2025-05-07-10:05
# @Author   : Sum3 TEO
# @E-mail   : rui3zhang@163
import re
from InsData.Vertex import Vertex
from InsData.Paramerters import Parameters
from InsData.Graph import Graph


class Instance(object):

    def __init__(self,
                 para: Parameters):

        self.para = para
        self.truckNum = 0
        self.truckCapacity = 0
        self.droneNum = 0
        self.cusNum = 0
        self.demand_sum = 0

        self.graph = Graph(ins=self)
        self.to_time, self.ld_time = self.calc_to_ld_time()
        self.to_and_ld_consumption = 0
        self.bigM = -float('inf')
        self.time_bgM = -float('inf')

    def read_data(self,
                  filename,
                  cus_num):

        self.cusNum = cus_num
        f = open(filename, 'r')
        lines = f.readlines()

        cnt = 0

        for line in lines:
            cnt += 1
            if cnt == 5:
                line = line[:-1].strip()  # '  5                200\n'
                str_arr = re.split(r" +", line)  # ['5', '200']
                self.truckNum = int(str_arr[0])
                self.truckCapacity = int(str_arr[1])

            elif (cnt >= 10) and (cnt <= 10 + cus_num):
                #    ' 0      40         50          0          0       1236          0\n'
                line = line[:-1].strip()
                str_arr = re.split(r" +",
                                   line)

                vertex = Vertex(vertex_id=int(str_arr[0]),
                                x_coord=int(str_arr[1]),
                                y_coord=int(str_arr[2]),
                                demand=int(str_arr[3]),
                                # ready_time_h=int(str_arr[4]) * 60,
                                # due_time_h=int(str_arr[5]) * 60,
                                # ready_time_s=(int(str_arr[4]) + 0.15 * (int(str_arr[5]) - int(str_arr[4]))) * 60,
                                # due_time_s=(int(str_arr[5]) - 0.55 * (int(str_arr[5]) - int(str_arr[4]))) * 60
                                ready_time_h=int(str_arr[4]),
                                due_time_h=int(str_arr[5]),
                                ready_time_s=(int(str_arr[4]) + 0.15 * (int(str_arr[5]) - int(str_arr[4]))),
                                due_time_s=(int(str_arr[5]) - 0.55 * (int(str_arr[5]) - int(str_arr[4])))
                                )
                self.demand_sum += vertex.demand
                self.graph.add_vertex(vertex=vertex)

                if self.time_bgM < vertex.hardDueTime:
                    self.time_bgM = vertex.hardDueTime

        # self.graph.vertexDict[15].hardReadyTime = self.graph.vertexDict[0].hardReadyTime
        # self.graph.vertexDict[15].hardDueTime = self.graph.vertexDict[0].hardDueTime
        # self.graph.vertexDict[1].hardReadyTime = self.graph.vertexDict[0].hardReadyTime
        # self.graph.vertexDict[1].hardDueTime = self.graph.vertexDict[0].hardDueTime

        # add a virtual depot node for the convenience of modelling
        virtual_depot = self.graph.vertexDict[0].copy_vertex()
        # virtual_depot.hardDueTime *= 3
        virtual_depot.ID = self.cusNum + 1
        self.graph.add_vertex(vertex=virtual_depot)

        """ cala arc and process infeasible arcs """
        self.graph.calc_arc()
        self.graph.calc_max_ride_time()
        self.graph.preprocess()

    def calc_to_ld_time(self):
        return round(self.para.h / self.para.droneTakeoffSpeed, 3), round(self.para.h / self.para.droneLandSpeed, 3)
