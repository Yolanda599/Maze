from tkinter import Button
from tkinter import Tk
from tkinter import Canvas

import numpy as np


class Maze(object):

    def __init__(self):

        self.blockcolorIndex = 0
        self.blockcolor = ['black', 'green', 'red', 'yellow']  # 障碍颜色为黑色、起点绿色 终点红色、路径黄色
        self.mapStatus = np.ones((15, 15), dtype=int)  # 地图状态数组15×15 1表示无障碍 0表示有障碍
        self.startPoint = 'start'  # 起点
        self.endPoint = 'end'  # 终点

        self.selectedStart = False  # 是否选了起点，默认未选择起点
        self.selectedEnd = False  # 是否选了终点，默认未选择终点

        self.openList = []  # open表
        self.closeList = []  # close表
        self.isOK = False  # 是否已经结束

        self.route = []  # 路径列表
        # 界面设计
        self.root = Tk()
        self.root.title('基于A*算法的迷宫游戏')
        self.root.geometry("800x800+300+0")
        self.btn_maze = Button(self.root, text="生成迷宫", command=self.selectmaze)
        self.btn_maze.pack()
        self.btn_start = Button(self.root, text="选择起点", command=self.selectstart)
        self.btn_start.pack()
        self.btn_end = Button(self.root, text="选择终点", command=self.selectend)
        self.btn_end.pack()
        self.btn_find = Button(self.root, text="开始寻路", command=self.selectfind)
        self.btn_find.pack()
        self.btn_restart = Button(self.root, text="重新开始", command=self.selectrestart)
        self.btn_restart.pack()
        self.canvas = Canvas(self.root, width=500, height=500, bg="white")#页面布局：500*500，白底
        self.canvas.pack()
        for i in range(1, 17):
            self.canvas.create_line(30, 30 * i, 480, 30 * i,dash=(4, 4))  # 横线
            self.canvas.create_line(30 * i, 30, 30 * i, 480,dash=(4, 4))  # 竖线
        self.canvas.bind("<Button-1>", self.drawMapBlock)
        self.root.mainloop()

    # 按钮对应具体操作
    def selectrestart(self):
        self.mapStatus = np.ones((15, 15), dtype=int)  # 地图状态数组15×15，1表示无障碍，0表示障碍
        self.startPoint = 'start'
        self.endPoint = 'end'
        self.selectedStart = False  # 是否选了起点 默认否
        self.selectedEnd = False  # 是否选了终点 默认否
        self.openList = []  # open表
        self.closeList = []  # close表
        self.isOK = False  # 是否已经结束
        self.route = []
        self.canvas.destroy()
        self.canvas = Canvas(self.root, width=500, height=500, bg="white")
        self.canvas.pack()

       #重新开始界面重新生成
        for i in range(1, 17):
            self.canvas.create_line(30, 30 * i, 480, 30 * i,dash=(4, 4))  # 横线
            self.canvas.create_line(30 * i, 30, 30 * i, 480,dash=(4, 4))  # 竖线
        self.canvas.bind("<Button-1>", self.drawMapBlock)

    def selectmaze(self):
        self.blockcolorIndex = 0  # 迷宫障碍物颜色：黑

    def selectstart(self):
        if not self.selectedStart:
            self.blockcolorIndex = 1  # 起点颜色：绿
        else:
            self.blockcolorIndex = 0  # 未选择菜单情况下默认为设计迷宫操作，填充黑色

    def selectend(self):
        if not self.selectedEnd:
            self.blockcolorIndex = 2  # 终点颜色：红
        else:
            self.blockcolorIndex = 0

    def selectfind(self):
        self.blockcolorIndex = 3  # 寻找出的最短路径的颜色：黄
        self.Astar()
        self.route.pop(-1)
        self.route.pop(0)
        for i in self.route:
            self.canvas.create_rectangle((i.x + 1) * 30, (i.y + 1) * 30, (i.x + 2) * 30, (i.y + 2) * 30, fill='yellow')


    def Astar(self):
        # 将起点放到open表中
        self.openList.append(self.startPoint)
        while (not self.isOK):
            # 先检查终点是否在open表中，若有则结束
            if self.inOpenList(self.endPoint) != -1:  # 在open表中，程序结束
                self.isOK = True  #
                self.end = self.openList[self.inOpenList(self.endPoint)]
                self.route.append(self.end)
                self.te = self.end
                while (self.te.parentPoint != 0):
                    self.te = self.te.parentPoint
                    self.route.append(self.te)
            else:
                self.sortOpenList()  # 将估值最小的节点放在index = 0
                current_min = self.openList[0]  # 估值最小节点
                self.openList.pop(0)
                self.closeList.append(current_min)
                # 设current_min节点，并放到open 表
                if current_min.x - 1 >= 0:  # 没有越界
                    if (self.mapStatus[current_min.y][current_min.x - 1]) != 0:  # 无障碍点,可前行路径
                        self.temp1 = mapPoint(current_min.x - 1, current_min.y, current_min.distanceStart + 1,
                                              self.endPoint.x, self.endPoint.y, current_min)
                        if self.inOpenList(self.temp1) != -1:  # open表存在相同的节点
                            if self.temp1.evaluate() < self.openList[self.inOpenList(self.temp1)].evaluate():
                                self.openList[self.inOpenList(self.temp1)] = self.temp1
                        elif self.inCloseList(self.temp1) != -1:  # 否则查看close表是否存在相同的节点，若存在进行if判断
                            if self.temp1.evaluate() < self.closeList[self.inCloseList(self.temp1)].evaluate():
                                self.closeList[self.inCloseList(self.temp1)] = self.temp1
                        else:  # open、close表都不存在 temp1
                            self.openList.append(self.temp1)

                if current_min.x + 1 < 15:
                    if (self.mapStatus[current_min.y][current_min.x + 1]) != 0:  # 无障碍点,可前行路径
                        self.temp2 = mapPoint(current_min.x + 1, current_min.y, current_min.distanceStart + 1,
                                              self.endPoint.x, self.endPoint.y, current_min)
                        if self.inOpenList(self.temp2) != -1:  # open表存在相同的节点
                            if self.temp2.evaluate() < self.openList[self.inOpenList(self.temp2)].evaluate():
                                self.openList[self.inOpenList(self.temp2)] = self.temp2
                        elif self.inCloseList(self.temp2) != -1:  # 否则，查看close表是否存在相同的节点（存在）
                            if self.temp2.evaluate() < self.closeList[self.inCloseList(self.temp2)].evaluate():
                                self.closeList[self.inCloseList(self.temp2)] = self.temp2
                        else:
                            self.openList.append(self.temp2)

                if current_min.y - 1 >= 0:
                    if (self.mapStatus[current_min.y - 1][current_min.x]) != 0:  # 无障碍点,可前行路径
                        self.temp3 = mapPoint(current_min.x, current_min.y - 1, current_min.distanceStart + 1,
                                              self.endPoint.x, self.endPoint.y, current_min)
                        if self.inOpenList(self.temp3) != -1:  # open表中存在相同的节点
                            if self.temp3.evaluate() < self.openList[self.inOpenList(self.temp3)].evaluate():
                                self.openList[self.inOpenList(self.temp3)] = self.temp3
                        elif self.inCloseList(self.temp3) != -1:  # 否则，查看close表是否存在相同的节点（存在）
                            if self.temp3.evaluate() < self.closeList[self.inCloseList(self.temp3)].evaluate():
                                self.closeList[self.inCloseList(self.temp3)] = self.temp3
                        else:
                            self.openList.append(self.temp3)

                if current_min.y + 1 < 15:
                    if (self.mapStatus[current_min.y + 1][current_min.x]) != 0:  # 无障碍点,可前行路径
                        self.temp4 = mapPoint(current_min.x, current_min.y + 1, current_min.distanceStart + 1,
                                              self.endPoint.x, self.endPoint.y, current_min)

                        if self.inOpenList(self.temp4) != -1:  # open表存在相同的节点
                            if self.temp4.evaluate() < self.openList[self.inOpenList(self.temp4)].evaluate():
                                self.openList[self.inOpenList(self.temp4)] = self.temp4
                        elif self.inCloseList(self.temp4) != -1:  # 否则查看close表是否存在相同的节点（存在）
                            if self.temp4.evaluate() < self.closeList[self.inCloseList(self.temp4)].evaluate():
                                self.closeList[self.inCloseList(self.temp4)] = self.temp4
                        else:
                            self.openList.append(self.temp4)
    #作障碍物
    def drawMapBlock(self, event):
        x, y = event.x, event.y
        if (30 <= x <= 480) and (30 <= y <= 480):
            i = int((x // 30) - 1)
            j = int((y // 30) - 1)
            # 记录下起止点，保证不能选择多个起点或者多个终点
            if self.blockcolorIndex == 1 and not self.selectedStart:
                self.startPoint = mapPoint(i, j, 0, 0, 0, 0)
                self.selectedStart = True
                self.canvas.create_rectangle((i + 1) * 30, (j + 1) * 30, (i + 2) * 30, (j + 2) * 30,
                                             fill=self.blockcolor[self.blockcolorIndex])
                self.blockcolorIndex = 0
            elif self.blockcolorIndex == 2 and not self.selectedEnd:
                self.endPoint = mapPoint(i, j, 0, 0, 0, 0)
                self.selectedEnd = True
                self.canvas.create_rectangle((i + 1) * 30, (j + 1) * 30, (i + 2) * 30, (j + 2) * 30,
                                             fill=self.blockcolor[self.blockcolorIndex])
                self.blockcolorIndex = 0
            else:
                self.canvas.create_rectangle((i + 1) * 30, (j + 1) * 30, (i + 2) * 30, (j + 2) * 30,
                                             fill=self.blockcolor[self.blockcolorIndex])
                self.mapStatus[j][i] = self.blockcolorIndex

    # 检查终点是否在open表中
    def endInOpenList(self):
        for i in self.openList:
            if self.endPoint[0] == i.x and self.endPoint[1] == i.y:
                return True
        return False

    # 将节点加入open表前，检查该节点是否在open表中
    def inOpenList(self, p1):
        for i in range(0, len(self.openList)):
            if p1.isEq(self.openList[i]):
                return i
        return -1

    # 将节点加入open表前，检查该节点是否在close表中
    def inCloseList(self, p1):
        for i in range(0, len(self.closeList)):
            if p1.isEq(self.closeList[i]):
                return i  # 若在返回索引
        return -1  # 不在返回-1

    # 将估值最小的排在 index = 0
    def sortOpenList(self):
        if len(self.openList) > 0:
            if len(self.openList) > 1:
                for i in range(1, len(self.openList)):
                    if self.openList[i].evaluate() < self.openList[0].evaluate():
                        self.t = self.openList[0]
                        self.openList[0] = self.openList[i]
                        self.openList[i] = self.t


class mapPoint(object):
    def __init__(self, x, y, distanceStart, endX, endY, parentPoint):
        self.x = x
        self.y = y
        self.distanceStart = distanceStart
        self.endX = endX
        self.endY = endY
        self.parentPoint = parentPoint  # 前一个节点

    def evaluate(self):   #估值函数
        return self.distanceStart + abs(self.x - self.endX) + abs(self.y - self.endY)

    def isEq(self, point):
        if point.x == self.x and point.y == self.y:
            return True
        else:
            return False


def main():
    Maze()


if __name__ == '__main__':
    main()