#!/usr/bin/env python
import rospy
import numpy as np
import math
import time
import random
import Queue

import std_msgs.msg
import geometry_msgs.msg
import nav_msgs.msg
import sensor_msgs.msg
import visualization_msgs.msg

class GraphBuilder():

    def __init__(self):
        rospy.init_node('graph_builder')
        random.seed()


        self.new_point_dist_thresh = 0.45
        self.dist_thresh = 0.35
        self.check_thresh = 0.25
        self.num_waypoints = 150
        self.nn = 3


        self.occ_thresh = 90
        
        self.processed = False

        self.map = None
        self.meta = None

        self.occupied = []
        self.waypoints = [(4, 0), (8, -4), (8, 0)]
        self.edges = []

        self.planning_points = Queue.Queue()
        self.planning_edges = []


        self.map_sub = rospy.Subscriber('/map', nav_msgs.msg.OccupancyGrid, callback=self.MapCallback)
        self.occ_pub = rospy.Publisher('/occupied_points', visualization_msgs.msg.MarkerArray, queue_size=5)
        self.wp_pub = rospy.Publisher('/waypoints', visualization_msgs.msg.MarkerArray, queue_size=5)
        self.edge_pub = rospy.Publisher('/edges', visualization_msgs.msg.MarkerArray, queue_size=5)
        self.clear_pub = rospy.Publisher('/visualization_marker', visualization_msgs.msg.Marker, queue_size=5)

    def MapCallback(self, msg):
        if self.processed:
            return

        self.map = msg
        self.meta = msg.info

    def ProcessMap(self):
        self.ClearRViz()
        print("Clearing")
        time.sleep(1)
        self.GetOccupiedPoints()
        print("Got Occupied Points")
        self.GetWaypoints()
        print("Got Waypoints")
        self.GetEdges()
        self.ViewEdges()
        print("Got Edges")
        self.PreProcessPlan()
        print("PreProcessed")
        self.processed = True

    def ClearRViz(self):
        marker = visualization_msgs.msg.Marker()
        marker.header.frame_id = '/my_frame'
        marker.header.stamp = rospy.Time.now()
        marker.ns = 'clear_points'
        marker.id = 0
        marker.type = visualization_msgs.msg.Marker.SPHERE
        marker.action = 3

        self.clear_pub.publish(marker)

    def GetGridIndex(self, i, j):
        # i is row, j is column; 0 indexed
        x = i * self.meta.width + j
        return int(x)

    def GridToPoint(self, i, j):
        # i is row (y), j is column (x)
        x = j * self.meta.resolution + self.meta.origin.position.x
        y = i * self.meta.resolution + self.meta.origin.position.y

        return x, y

    def GetOccupiedPoints(self):
        msg = visualization_msgs.msg.MarkerArray()
        mark_index = 0
        stamp = rospy.Time.now()

        scale = geometry_msgs.msg.Vector3()
        scale.x = 0.05
        scale.y = 0.05
        scale.z = 0.05

        color = std_msgs.msg.ColorRGBA()
        color.r = 1
        color.g = 0
        color.b = 0
        color.a = 1.0

        for j in range(0, self.meta.width):
            for i in range(0, self.meta.height):

                index = self.GetGridIndex(i, j)

                #print(self.map.data[index])
                if self.map.data[index] > self.occ_thresh:
                    x, y = self.GridToPoint(i, j)

                    self.occupied.append((x,y))
                    #print("%f %f" % (x, y))

                    marker = visualization_msgs.msg.Marker()
                    marker.header.frame_id = '/my_frame'
                    marker.header.stamp = stamp
                    marker.ns = 'occ_points'
                    marker.id = mark_index
                    marker.type = visualization_msgs.msg.Marker.SPHERE
                    marker.action = visualization_msgs.msg.Marker.ADD

                    pose = geometry_msgs.msg.Pose()
                    pose.position.x = x
                    pose.position.y = y
                    pose.position.z = 0.0
                    pose.orientation.w = 1.0
                    marker.pose = pose

                    
                    marker.scale = scale         
                    marker.color = color

                    #self.occ_pubx.publish(marker)

                    msg.markers.append(marker)
                    mark_index = mark_index + 1

        self.occ_pub.publish(msg)
        print("Published Occ'd points")

    def GetWaypoints(self):

        num_waypoints = 0
        while num_waypoints < self.num_waypoints:

            i = random.randint(0, self.meta.height)
            j = random.randint(0, self.meta.width)

            x, y = self.GridToPoint(i, j)

            if self.CheckCollision(x, y):

                # Check to see if too close to another point
                too_close = False
                for pt in self.waypoints:
                    if self.SquareDist((x,y), pt) < self.new_point_dist_thresh*self.new_point_dist_thresh:
                        too_close = True
                        break

                if not too_close:
                    self.waypoints.append((x,y))
                    num_waypoints = num_waypoints + 1
                    print("Added waypoint %i" % num_waypoints)


        msg = visualization_msgs.msg.MarkerArray()
        mark_index = 0
        stamp = rospy.Time.now()

        scale = geometry_msgs.msg.Vector3()
        scale.x = 0.05
        scale.y = 0.05
        scale.z = 0.05

        color = std_msgs.msg.ColorRGBA()
        color.r = 0
        color.g = 0
        color.b = 1
        color.a = 1.0

        for wp in self.waypoints:
            marker = visualization_msgs.msg.Marker()
            marker.header.frame_id = '/my_frame'
            marker.header.stamp = stamp
            marker.ns = 'waypoints'
            marker.id = mark_index
            marker.type = visualization_msgs.msg.Marker.SPHERE
            marker.action = visualization_msgs.msg.Marker.ADD

            pose = geometry_msgs.msg.Pose()
            pose.position.x = wp[0]
            pose.position.y = wp[1]
            pose.position.z = 0.05
            pose.orientation.w = 1.0
            marker.pose = pose

            
            marker.scale = scale         
            marker.color = color

            #self.occ_pubx.publish(marker)

            msg.markers.append(marker)
            mark_index = mark_index + 1
        self.wp_pub.publish(msg)

    def GetEdges(self):

        start_index = -1
        for start in self.waypoints:
            start_index = start_index + 1

            neighbours = []

            for iteration in range(0, self.nn):
                min_dist = 999
                min_index = -1

                end_index = -1
                for end in self.waypoints:
                    end_index = end_index + 1

                    if end not in neighbours and end is not start:
                        dist = self.SquareDist(start, end)
                        if dist < min_dist:
                            min_dist = dist
                            min_index = end_index

                    else:
                        pass # Points is already checked or is start

                end = self.waypoints[min_index]
                neighbours.append(end)

                if self.CheckLine(start, end):
                    self.edges.append((start_index, min_index))
                    #print("Added Edge")
            '''
            print("Start: ")
            print(self.waypoints[start_index])
            print("Neighbours: ")
            print(neighbours)
            '''

    def ViewEdges(self):
        msg = visualization_msgs.msg.MarkerArray()
        mark_index = 0
        stamp = rospy.Time.now()

        scale = geometry_msgs.msg.Vector3()
        scale.x = 0.02
        scale.y = 0.02
        scale.z = 0.02

        color = std_msgs.msg.ColorRGBA()
        color.r = 1
        color.g = 1
        color.b = 1
        color.a = 1.0

        for edge in self.edges:

            start = self.waypoints[edge[0]]
            end = self.waypoints[edge[1]]

            marker = visualization_msgs.msg.Marker()
            marker.header.frame_id = '/my_frame'
            marker.header.stamp = stamp
            marker.ns = 'edges'
            marker.id = mark_index
            marker.type = visualization_msgs.msg.Marker.LINE_LIST
            marker.action = visualization_msgs.msg.Marker.ADD

            pose1 = geometry_msgs.msg.Point()
            pose1.x = start[0]
            pose1.y = start[1]
            pose1.z = 0.05

            pose2 = geometry_msgs.msg.Point()
            pose2.x = end[0]
            pose2.y = end[1]
            pose2.z = 0.05

            marker.points.append(pose1)
            marker.points.append(pose2)

            
            marker.scale = scale         
            marker.color = color

            #self.occ_pubx.publish(marker)

            msg.markers.append(marker)
            mark_index = mark_index + 1

        self.edge_pub.publish(msg)

    def SquareDist(self, p1, p2):
        dx = p1[0] - p2[0]
        dy = p1[1] - p2[1]

        return dx*dx + dy*dy

    def CheckLine(self, p1, p2):

        dx = p1[0] - p2[0]
        dy = p1[1] - p2[1]
        dist = math.sqrt(dx*dx + dy*dy)

        if dist != 0:
            alpha = np.arange(0, 1, self.check_thresh)
        else:
            alpha = [1]



        for a in alpha:
            x = p1[0] + (p2[0] - p1[0])*a
            y = p1[1] + (p2[1] - p1[1])*a

            if not self.CheckCollision(x, y):
                return False

        return True


    def CheckCollision(self, x, y):

        for pt in self.occupied:
            dx = pt[0] - x
            dy = pt[1] - y

            dist = dx*dx + dy*dy

            if dist < self.dist_thresh*self.dist_thresh:
                return False
        
        return True

    def PreProcessPlan(self):

        index = 0

        for pt in self.waypoints:
            self.planning_points.put((index, pt, -1, 0))

        self.planning_edges = self.edges

    def GetClosestPlanningPoint(self, point):
        min_dist = 999
        min_index = -1

        for pt in self.planning_points:
            dist = self.SquareDist(point, pt[1])
            
            if dist < min_dist:
                min_dist = dist
                min_index = pt[0]

        return min_index

    def AStar(self, start, end):

        start_ind = self.GetClosestPlanningPoint(start)
        end_ind = self.GetClosestPlanningPoint(end)


        open_set = []
        closed_set = []

        open_set.append(self.planning_points[start_ind])

        best_ind = start_ind

        while best_index is not end_ind:
            pass


    def GetNeighbours(self, ind):




    def main(self):
        while not rospy.is_shutdown():
            time.sleep(0.2)

            if (self.map is not None) and (not self.processed):
                self.ProcessMap()

if __name__ == "__main__":
    
    gb = GraphBuilder()
    gb.main()