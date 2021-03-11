
import random
import math
import time
import os
import cv2
import numpy as np
from scipy.spatial import cKDTree


class Node:
    """
    Node class for dijkstra / A* search algorithm
    """

    def __init__(self, x, y, cost, parent_index):
        self.x = x
        self.y = y
        self.cost = cost
        self.parent_index = parent_index

    def __str__(self):
        return "x: " + str(self.x) + ", y: " + str(self.y) + "," + str(self.cost) + "," + str(self.parent_index)



class PRM:
    """
    PRM Planner

    """

    def __init__(self, image, res, sx, sy, gx, gy, robotSize, NSample=500, edgeFromeOneSamplePoint=20, maxEdgeLength=10.0):
        self._image = image
        self._res = res

        self._sx = sx
        self._sy = sy
        self._gx = gx
        self._gy = gy
        self._robotSize = robotSize

        self._edgeFromeOneSamplePoint = edgeFromeOneSamplePoint
        self._NSample = NSample
        self._maxEdgeLength = maxEdgeLength

    def __str__(self):
        return "Prm planner with parameters : \nNumber Sample : " + str(self._NSample) + ", Maximum edge from one point : " + str(self._edgeFromeOneSamplePoint) + ", Maximum edge length : " + str(self._maxEdgeLength)

    def getObstaclesFromImage(self):
        ox = []
        oy = []
        for i in range(len(self._image)):
            for j in range(len(self._image[i])):
                if self._image[i][j] == 255:
                    # Conversion from openCV frame to real world frame
                    oy.append((len(self._image) - i)/self._res)
                    ox.append(j/self._res)
        return ox, oy

    def startPlanner(self, aStar = True, verbiose=True):

        start = time.clock()
        ox, oy = self.getObstaclesFromImage()
        if(verbiose):
            print(" Time get obstacle: " + str(time.clock()- start))

        start = time.clock()
        obstacle_kd_tree = cKDTree(np.vstack((ox, oy)).T)
        if(verbiose):
            print(" Time get cdkTree: " + str(time.clock()- start))

        start = time.clock()
        sample_x, sample_y = self.generateSamplePoints(
            ox, oy, obstacle_kd_tree)
        if(verbiose):
            print(" Time generate Sample Point: " + str(time.clock()- start))

        start = time.clock()
        road_map = self.generateRoadMap(
            sample_x, sample_y, obstacle_kd_tree)
        if(verbiose):
            print(" Time generate road map: " + str(time.clock()- start))
        
        start = time.clock()
        rx, ry = self.AStar(road_map, sample_x, sample_y) if aStar else self.dijkstra(road_map, sample_x, sample_y)
        if(verbiose):
            print(" Time plan: " + str(time.clock()- start))

        return rx, ry

    # def isCollision(self, sx, sy, gx, gy, obstacle_kd_tree):
    #     x = sx
    #     y = sy
    #     dx = gx - sx
    #     dy = gy - sy
    #     yaw = math.atan2(dy, dx)
    #     d = math.hypot(dx, dy)

    #     if d > self._maxEdgeLength:
    #         return True  # not checking if collision or not

    #     D = self._robotSize
    #     n_step = round(d / D)

    #     for i in range(int(n_step)):
    #         dist, _ = obstacle_kd_tree.query([x, y])
    #         if dist <= self._robotSize:
    #             return True  # it's a collision
    #         x += D * math.cos(yaw)
    #         y += D * math.sin(yaw)

    #     # goal point check
    #     dist, _ = obstacle_kd_tree.query([self._gx, self._gy])
    #     if dist <= self._robotSize:
    #         return True  # it's a collision

    #     return False  # it's not a collision

    
    def isCollision(self, sx, sy, gx, gy, obstacle_kd_tree):
        x = sx
        y = sy
        dx = gx - sx
        dy = gy - sy
        d = dx + dy

        if d > self._maxEdgeLength:
            return True  # not checking if collision or not
        
        yaw = math.atan2(dy, dx)
        n_step = round(d / self._robotSize)

        for i in range(int(n_step)):
            dist, _ = obstacle_kd_tree.query([x, y])
            if dist <= self._robotSize:
                return True  # it's a collision
            x += self._robotSize * math.cos(yaw)
            y += self._robotSize * math.sin(yaw)

        # goal point check
        dist, _ = obstacle_kd_tree.query([self._gx, self._gy])
        if dist <= self._robotSize:
            return True  # it's a collision

        return False  # it's not a collision

    def generateRoadMap(self, sample_x, sample_y, obstacle_kd_tree):

        print("")
        print("")
        start = time.clock()
        sample_kd_tree = cKDTree(np.vstack((sample_x, sample_y)).T)
        road_map = []
        nSample = len(sample_x)
        nSample2 = 20

        print("Kd tree size / len")
        print(sample_kd_tree)
        print(sample_kd_tree.size)
        print("")
        print(" Time init roadMap: " + str(time.clock()- start))

        for (i, ix, iy) in zip(range(nSample), sample_x, sample_y):

            dists, indexes = sample_kd_tree.query([ix, iy], k=self._edgeFromeOneSamplePoint, n_jobs=-1)
            edge_id = []

            for y in range(1, len(indexes)):
                nx = sample_x[indexes[y]]
                ny = sample_y[indexes[y]]

                if not self.isCollision(ix, iy, nx, ny, obstacle_kd_tree):
                    edge_id.append(indexes[y])

            road_map.append(edge_id)

        return road_map

    def dijkstra(self, road_map, sample_x, sample_y, verbiose=True):

        goal_node = Node(self._gx, self._gy, 0.0, -1)
        start_node = Node(self._sx, self._sy, 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[len(road_map) - 2] = start_node
        path_found = True

        while True:
            if not open_set:
                path_found = False
                break

            c_id = min(open_set, key=lambda o: open_set[o].cost)
            current = open_set[c_id]

            if c_id == (len(road_map) - 1):
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break
            

            # Remove the item from the open set and add it to the closed set
            del open_set[c_id]
            closed_set[c_id] = current

            # expand search grid based on motion model
            for i in range(len(road_map[c_id])):
                n_id = road_map[c_id][i]
                dx = sample_x[n_id] - current.x
                dy = sample_y[n_id] - current.y
                d = math.hypot(dx, dy)
                node = Node(sample_x[n_id], sample_y[n_id],
                            current.cost + d, c_id)

                if n_id in closed_set:
                    continue
                # Otherwise if it is already in the open set
                if n_id in open_set:
                    if open_set[n_id].cost > node.cost:
                        open_set[n_id].cost = node.cost
                        open_set[n_id].parent_index = c_id
                else:
                    open_set[n_id] = node

        if path_found is False:
            if(verbiose):
                print("Path not found")
            return [], []
        else:
            if(verbiose):
                print("Path found")
        # generate final course
        rx, ry = [goal_node.x], [goal_node.y]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(n.x)
            ry.append(n.y)
            parent_index = n.parent_index

        return rx, ry


    @staticmethod
    def calcHeuristic(n1, n2):
        w = 1.0  # weight of heuristic
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d

    def AStar(self, road_map, sample_x, sample_y, verbiose=True):

        goal_node = Node(self._gx, self._gy, 0.0, -1)
        start_node = Node(self._sx, self._sy, 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[len(road_map) - 2] = start_node
        path_found = True

        while True:
            if not open_set:
                path_found = False
                break

            c_id = min(open_set, key=lambda o: open_set[o].cost + self.calcHeuristic(goal_node, open_set[o]))
            current = open_set[c_id]

            if c_id == (len(road_map) - 1):
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            # Remove the item from the open set and add it to the closed set
            del open_set[c_id]
            closed_set[c_id] = current

            # expand search grid based on motion model
            for i in range(len(road_map[c_id])):
                n_id = road_map[c_id][i]
                dx = sample_x[n_id] - current.x
                dy = sample_y[n_id] - current.y
                d = math.hypot(dx, dy)
                node = Node(sample_x[n_id], sample_y[n_id],
                            current.cost + d, c_id)

                if n_id in closed_set:
                    continue
                # Otherwise if it is already in the open set
                if n_id in open_set:
                    if open_set[n_id].cost > node.cost:
                        open_set[n_id].cost = node.cost
                        open_set[n_id].parent_index = c_id
                else:
                    open_set[n_id] = node

        if path_found is False:
            if(verbiose):
                print("Path not found")
            return [], []
        else:
            if( verbiose):
                print("Path found")
        # generate final course
        rx, ry = [goal_node.x], [goal_node.y]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(n.x)
            ry.append(n.y)
            parent_index = n.parent_index
        

        return rx, ry



    def generateSamplePoints(self, ox, oy, obstacle_kd_tree):
        max_x = max(ox)
        max_y = max(oy)
        min_x = min(ox)
        min_y = min(oy)

        sample_x, sample_y = [], []

        while len(sample_x) <= self._NSample:
            tx = (random.random() * (max_x - min_x)) + min_x
            ty = (random.random() * (max_y - min_y)) + min_y

            dist, index = obstacle_kd_tree.query([tx, ty])

            if dist >= self._robotSize:
                sample_x.append(tx)
                sample_y.append(ty)

        sample_x.append(self._sx)
        sample_y.append(self._sy)

        sample_x.append(self._gx)
        sample_y.append(self._gy)

        return sample_x, sample_y

    def pointMapToImg(self, xMap, yMap):
        xPixel = int(xMap*self._res)
        yPixel = int(len(self._image) - int(yMap*self._res))
        return (xPixel, yPixel)

    def saveToVideo(self, rx, ry, converToGif=False, startIconSize=2, circleSize=2, lineSize=2):
        # RGB conversion
        imageStart = cv2.cvtColor(self._image, cv2.COLOR_GRAY2BGR)

        #imageStart = self._image

        height, width, channel = imageStart.shape

        fourcc = cv2.VideoWriter_fourcc(*'MP42')
        video = cv2.VideoWriter('./output.avi', fourcc, 2, (width, height))

        # Start and end rectangle (green and red)
        # StartIconSize define the size of each rectangle in pixel
        cv2.rectangle(imageStart,
                      self.pointMapToImg(
                          self._sx-startIconSize/self._res, self._sy-startIconSize/self._res),
                      self.pointMapToImg(
                          self._sx+startIconSize/self._res, self._sy+startIconSize/self._res),
                      (0, 255, 0), -1)

        cv2.rectangle(imageStart,
                      self.pointMapToImg(self._gx-startIconSize/self._res, self._gy -
                                         startIconSize/self._res),
                      self.pointMapToImg(self._gx+startIconSize/self._res, self._gy +
                                         startIconSize/self._res),
                      (0, 0, 255), -1)

        # Wiriting first image multiple time in order to have a start "wait"
        for i in range(6):  # 3 seconds of wait
            video.write(imageStart)

        if(rx != None):
            imageSubsequent = imageStart
            for i in range(len(rx)-2, 0, -1):
                cv2.circle(imageStart,
                           self.pointMapToImg(rx[i], ry[i]),
                           circleSize,
                           (255, 0, 0), -1)
                cv2.line(imageStart,
                         self.pointMapToImg(
                             rx[i+1], ry[i+1]),
                         self.pointMapToImg(rx[i], ry[i]),
                         (255, 0, 0), 2)
                video.write(imageSubsequent)

            i = 0
            cv2.line(imageStart,
                     self.pointMapToImg(rx[i+1], ry[i+1]),
                     self.pointMapToImg(rx[i], ry[i]),
                     (255, 0, 0), lineSize)
            video.write(imageSubsequent)

        video.release()

        # Convert to gif
        # https://engineering.giphy.com/how-to-make-gifs-with-ffmpeg/
        if(converToGif):
            filePathIn = "output.avi"
            filePathOut = "output.gif"
            command = "ffmpeg -y -i " + \
                str(filePathIn) + \
                " -filter_complex \"[0:v] split [a][b];[a] palettegen [p];[b][p] paletteuse\" " + str(
                    filePathOut)
            os.system(command)


if __name__ == "__main__":

    img = cv2.imread(__file__.replace("PRM.py", "map.png"), 0)
    res = 20.0
    sx = 16.6
    sy = 1.6
    gx = 265.0 / 20  # [m]
    gy = 220.0 / 20  # [m]
    robotSize = 0.160 / 2  # [m]

    import time
    test = PRM(img, res, sx, sy, gx, gy, robotSize)

    # start = time.clock()
    # print("AStar starting")
    # print(" ")
    # rx, ry = test.startPlanner()
    #print("Elapsed time 1: " + str(time.clock()- start))

    start = time.clock()
    print("Dijkstra starting")
    print(" ")
    rx, ry = test.startPlanner(False)
    #print("Elapsed time 2: " + str(time.clock()- start))
    #test.saveToVideo(rx, ry, True)
