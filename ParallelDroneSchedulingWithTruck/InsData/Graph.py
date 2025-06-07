# -*- coding: utf-8 -*-
# @Time     : 2025-05-07-10:10
# @Author   : Sum3 TEO
# @E-mail   : rui3zhang@163
from typing import Dict
from math import *
from InsData.Vertex import Vertex
from InsData.Arc import Arc
from Common import clac_euclidean_dis, clac_manhattan_dis


class Graph(object):

    def __init__(self,
                 ins):
        self._ins = ins
        self.vertexDict: Dict[int, Vertex] = dict()
        self.arcDict: Dict[tuple, Arc] = dict()
        self.cus_drone = None

    def add_vertex(self, vertex):
        """
        add a single vertex into the graph
        :param vertex:
        :return:
        """
        self.vertexDict[vertex.ID] = vertex

    def add_arc(self,
                arc: Arc):
        """
        add a single edge into the graph
        :param arc:
        :return:
        """
        self.arcDict[arc.head_vertex, arc.tail_vertex] = arc

        # update the successors and predecessors accordingly
        self.vertexDict[arc.head_vertex].successors.append(arc.tail_vertex)
        self.vertexDict[arc.tail_vertex].predecessors.append(arc.head_vertex)

    def calc_arc(self):
        """ compute the distance matrix of the customers """
        self.cus_drone = list(range(1, self._ins.cusNum + 1))
        for i in range(len(self.vertexDict) - 1):
            if self.vertexDict[i].demand > self._ins.para.L * self._ins.para.singleQ:
                try:
                    self.cus_drone.remove(i)
                except ValueError:
                    pass  # 元素不存在，什么也不做

            for j in range(1, len(self.vertexDict)):
                # if (i != j) and not (i == 0 and j == self._ins.cusNum + 1):
                if i != j:
                    temp_euclidean_dis = clac_euclidean_dis(x_1=self.vertexDict[i].xCoord,
                                                            y_1=self.vertexDict[i].yCoord,
                                                            x_2=self.vertexDict[j].xCoord,
                                                            y_2=self.vertexDict[j].yCoord
                                                            )
                    # temp_euclidean_dis *= 1e2

                    temp_manhattan_dis = clac_manhattan_dis(x_1=self.vertexDict[i].xCoord,
                                                            y_1=self.vertexDict[i].yCoord,
                                                            x_2=self.vertexDict[j].xCoord,
                                                            y_2=self.vertexDict[j].yCoord
                                                            )
                    # temp_manhattan_dis *= 1e2

                    arc = Arc(head_vertex=i,
                              tail_vertex=j,
                              euclidean_distance=temp_euclidean_dis,
                              manhattan_distance=temp_manhattan_dis,
                              travel_time_truck=round(temp_manhattan_dis / self._ins.para.truckSpeed, 3),
                              travel_time_drone=round(temp_euclidean_dis / self._ins.para.droneCruiseSpeed, 3))

                    self.add_arc(arc=arc)

        for i in range(len(self.vertexDict)):
            if (len(self.vertexDict) - 1, i) in self.arcDict.keys():
                del self.arcDict[len(self.vertexDict) - 1, i]

            if (i, 0) in self.arcDict.keys():
                del self.arcDict[i, 0]

    def preprocess(self):
        """
        delete those infeasible arcs
        :return:
        """
        to_ld_time = self._ins.to_time + self._ins.ld_time
        for i in range(len(self.vertexDict)):
            if (0, i) in self.arcDict.keys() and i != self._ins.cusNum + 1:
                # cannot serve
                cond1 = (self.vertexDict[0].hardReadyTime + self.arcDict[0, i].truckTravelTime >
                         self.vertexDict[len(self.vertexDict) - 1].hardDueTime)
                cond2 = (self.vertexDict[0].hardReadyTime + to_ld_time + self.arcDict[
                    i, len(self.vertexDict) - 1].droneTravelTime >
                         self.vertexDict[len(self.vertexDict) - 1].hardDueTime)
                if cond1 and cond2:
                    print("the calculating example is false")
            for j in range(len(self.vertexDict)):
                if (i, j) in self.arcDict.keys():
                    service_time = self._ins.para.serviceTime if i != 0 else 0
                    con_truck = self.vertexDict[i].hardReadyTime + service_time + self.arcDict[i, j].truckTravelTime > \
                                self.vertexDict[j].hardDueTime
                    # 1. cannot serve, 2. exceed max ride time
                    con_drone = (self.vertexDict[i].hardReadyTime + to_ld_time + service_time + self.arcDict[
                        i, j].droneTravelTime > self.vertexDict[j].hardDueTime) or (
                                        self.arcDict[i, j].droneTravelTime > self._ins.para.ride_time_max)
                    # if i == 0:
                    #     print(con_drone)
                    if i == j:
                        self.arcDict[i, j].adj_truck = 0
                        self.arcDict[i, j].adj_drone = 0

                    elif con_truck is True and con_drone is False:
                        self.arcDict[i, j].adj_truck = 0
                        self.arcDict[i, j].adj_drone = 1

                    elif con_drone is True and con_truck is False:
                        self.arcDict[i, j].adj_truck = 1
                        self.arcDict[i, j].adj_drone = 0

                    else:
                        self.arcDict[i, j].adj_truck = 1
                        self.arcDict[i, j].adj_drone = 1

                    """ calc bigM """
                    if self.arcDict[i, j].adj_truck == 1:
                        self._ins.bigM = max(
                            self.vertexDict[i].hardReadyTime + self.arcDict[i, j].truckTravelTime - self.vertexDict[
                                i].hardReadyTime,
                            self._ins.bigM
                        )

                    if self.vertexDict[i].demand + self.vertexDict[j].demand > self._ins.truckCapacity:
                        self.arcDict[i, j].adj_truck = 0
                    if self.vertexDict[i].demand + self.vertexDict[j].demand > self._ins.para.L * self._ins.para.singleQ:
                        self.arcDict[i, j].adj_drone = 0

    def calc_max_ride_time(self):
        """ calc square """
        x_min = float('inf')
        x_max = -float('inf')
        y_min = float('inf')
        y_max = -float('inf')

        for ver in self.vertexDict.keys():
            if self.vertexDict[ver].xCoord > x_max:
                x_max = self.vertexDict[ver].xCoord
            if self.vertexDict[ver].xCoord < x_min:
                x_min = self.vertexDict[ver].xCoord
            if self.vertexDict[ver].yCoord > y_max:
                y_max = self.vertexDict[ver].yCoord
            if self.vertexDict[ver].yCoord < y_min:
                y_min = self.vertexDict[ver].yCoord

        # diagonal = sqrt((x_max - x_min) ** 2 + (y_max - y_min) ** 2) * 1e2
        diagonal = sqrt((x_max - x_min) ** 2 + (y_max - y_min) ** 2)
        self._ins.para.ride_time_max = round(0.5 * diagonal, 2)