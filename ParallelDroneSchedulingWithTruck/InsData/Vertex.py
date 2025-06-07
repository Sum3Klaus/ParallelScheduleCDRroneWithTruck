# -*- coding: utf-8 -*-
# @Time     : 2025-05-07-10:05
# @Author   : Sum3 TEO
# @E-mail   : rui3zhang@163


class Vertex(object):

    def __repr__(self):
        time_windows = f'[{self.hardReadyTime},[{self.softReadyTime},{self.softDueTime}],{self.hardDueTime}]'
        return f'Vertex:{self.ID},demand={self.demand},{time_windows}'

    def __init__(self,
                 vertex_id=0,
                 demand=0,
                 x_coord=0,
                 y_coord=0,
                 ready_time_h=0,
                 due_time_h=1240,
                 ready_time_s=0,
                 due_time_s=1240
                 ):
        self.ID = vertex_id
        self.demand = demand
        self.xCoord = x_coord
        self.yCoord = y_coord
        self.hardReadyTime = ready_time_h
        self.hardDueTime = due_time_h
        self.softReadyTime = ready_time_s
        self.softDueTime = due_time_s

        self.predecessors = []
        self.successors = []

    def copy_vertex(self):
        vertex_copy = Vertex(vertex_id=self.ID,
                             demand=self.demand,
                             x_coord=self.xCoord,
                             y_coord=self.yCoord,
                             ready_time_h=self.hardReadyTime,
                             due_time_h=self.hardDueTime,
                             ready_time_s=self.softReadyTime,
                             due_time_s=self.softDueTime
                             )

        vertex_copy.predecessors = self.predecessors
        vertex_copy.successors = self.successors

        return vertex_copy