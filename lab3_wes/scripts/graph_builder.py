#!/usr/bin/env python
from __future__ import print_function

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


class Vertex(object):
    def __init__(self, index, prevVertex, distToPrev):
        self.index = index
        self.prevVertex = prevVertex
        self.distToPrev = distToPrev
    


class GraphBuilder():

    def __init__(self):
        print("Entered")
        rospy.init_node('graph_builder')
        random.seed()
        print("Rest of Enter")

        # Parameters to change graph
        # These are for simulation; Set new parameters for experiment
        self.new_point_dist_thresh = 0.45
        self.dist_thresh = 0.35
        self.check_thresh = 0.25
        self.num_waypoints = 150
        self.nn = 3


        self.occ_thresh = 90
        
        self.processed = False

        self.map = None
        self.meta = None

        #Start and Goal
        self.startPoint = (0,0)
        self.endPoint = (0,0)
        self.goalChanged = False

        # Graph elements
        self.occupied = []
        self.waypoints = [(4, 0), (8, -4), (8, 0)]
        self.edges = []

        # For planning
        self.planning_points = []
        self.planning_edges = []


        # Communication
        self.occ_pub = rospy.Publisher('/occupied_points', visualization_msgs.msg.MarkerArray, queue_size=5)
        self.wp_pub = rospy.Publisher('/waypoints', visualization_msgs.msg.MarkerArray, queue_size=5)
        self.edge_pub = rospy.Publisher('/edges', visualization_msgs.msg.MarkerArray, queue_size=5)
        self.clear_pub = rospy.Publisher('/visualization_marker', visualization_msgs.msg.Marker, queue_size=5)
        # for visualization
        self.plan_pub = rospy.Publisher('/plan', visualization_msgs.msg.Marker, queue_size=5)
        # For plan path
        self.plan_array_pub = rospy.Publisher('/plan_array', geometry_msgs.msg.PoseArray, queue_size=5)

        self.map_sub = rospy.Subscriber('/map', nav_msgs.msg.OccupancyGrid, callback=self.MapCallback)
        self.goal_sub = rospy.Subscriber('/goal', geometry_msgs.msg.PoseArray, callback=self.GoalCallback)

        print("Initialized")


    def MapCallback(self, msg):
        if self.processed:
            return

        print("New Map")
        # Store map data locally
        self.map = msg
        self.meta = msg.info

        for i in range(0, len(self.waypoints)):
            #self.waypoints[i] = (self.waypoints[i][0] + self.meta.origin.position.x, self.waypoints[i][1] + self.meta.origin.position.y)
            pass
    
    def GoalCallback(self, msg):
        print("Goal New Goal")

        point1 = (msg.poses[0].position.x, msg.poses[0].position.y)
        point2 = (msg.poses[1].position.x, msg.poses[1].position.y)

        # Change goal and current pos to start a new plan
        self.goalChanged = True
        self.startPoint = point1
        self.endPoint = point2

        print(self.startPoint)
        print(self.endPoint)


    def ProcessMap(self):
        # Go through steps to create the graph, and get ready for planning
        self.ClearRViz()
        print("Clearing")
        time.sleep(1)
        self.GetOccupiedPoints()
        print("Got Occupied Points")
        self.ViewWaypoints(ns='goal', z=0.3, s=0.2)
        self.GetWaypoints()
        self.ViewWaypoints(ns='waypoints', z=0.05, s=0.05)
        print("Got Waypoints")
        self.GetEdges()
        self.ViewEdges()
        print("Got Edges")
        self.PreProcessPlan()
        print("PreProcessed")
        self.processed = True

    def ClearRViz(self):
        # Call this to try and get rid of other PRM map markers on rviz, from prev runs
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
        # Build a visualization array
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

        # Go through map by indices
        for j in range(0, self.meta.width):
            for i in range(0, self.meta.height):

                index = self.GetGridIndex(i, j)

                #print(self.map.data[index])
                if self.map.data[index] > self.occ_thresh:

                    # find pos of occupied point, and add to list of points
                    x, y = self.GridToPoint(i, j)
                    self.occupied.append((x,y))
                    #print("%f %f" % (x, y))

                    # Make a marker for each point
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

        # Create a set number of waypoints
        num_waypoints = 0
        while num_waypoints < self.num_waypoints:

            i = random.randint(0, self.meta.height)
            j = random.randint(0, self.meta.width)

            # Coords of  random point
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


    def ViewWaypoints(self, ns='waypoints', z=0.05, s=0.05):
        # create Viz msg
        msg = visualization_msgs.msg.MarkerArray()
        mark_index = 0
        stamp = rospy.Time.now()

        scale = geometry_msgs.msg.Vector3()
        scale.x = s
        scale.y = s
        scale.z = s

        color = std_msgs.msg.ColorRGBA()
        color.r = 0
        color.g = 0
        color.b = 1
        color.a = 1.0

        # Add each waypoint (node in graph) to vis message via a marker
        for wp in self.waypoints:
            marker = visualization_msgs.msg.Marker()
            marker.header.frame_id = '/my_frame'
            marker.header.stamp = stamp
            marker.ns = ns
            marker.id = mark_index
            marker.type = visualization_msgs.msg.Marker.SPHERE
            marker.action = visualization_msgs.msg.Marker.ADD

            pose = geometry_msgs.msg.Pose()
            pose.position.x = wp[0]
            pose.position.y = wp[1]
            pose.position.z = z
            pose.orientation.w = 1.0
            marker.pose = pose

            
            marker.scale = scale         
            marker.color = color

            #self.occ_pubx.publish(marker)

            msg.markers.append(marker)
            mark_index = mark_index + 1
        self.wp_pub.publish(msg)

    def GetEdges(self):

        # Attempt to create edges from each waypoint/node
        start_index = -1
        for start in self.waypoints:
            start_index = start_index + 1

            neighbours = []

            # Attempt to create an edge a certain number of times
            for iteration in range(0, self.nn):
                min_dist = 999
                min_index = -1

                end_index = -1
                # Go through list of all other waypoints
                for end in self.waypoints:
                    end_index = end_index + 1

                    # Check to see if htis edges has been created, or was a failure before
                    if end not in neighbours and end is not start:
                        dist = self.SquareDist(start, end)
                        if dist < min_dist:
                            # Point is closest point, and valid for an edge; Track
                            min_dist = dist
                            min_index = end_index

                    else:
                        pass # Points is already checked or is start

                # Mark final point, and add to list of points hwich have been checked
                end = self.waypoints[min_index]
                neighbours.append(end)

                # Check for collisions in edge
                if self.CheckLine(start, end):
                    self.edges.append((start_index, min_index)) # Store indices of each point of the edge
                    #print("Added Edge")
            '''
            print("Start: ")
            print(self.waypoints[start_index])
            print("Neighbours: ")
            print(neighbours)
            '''

    def ViewEdges(self):
        # Create messgae to view
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

        # Create a marker with 2 points per marker (start, end) for each edge
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

        # First, see if points are the same
        dx = p1[0] - p2[0]
        dy = p1[1] - p2[1]
        dist = math.sqrt(dx*dx + dy*dy)

        if dist != 0:
            alpha = np.arange(0, 1, self.check_thresh)
        else:
            alpha = [1]


        # Interpolate positions between points; check each for distance from collisions
        for a in alpha:
            x = p1[0] + (p2[0] - p1[0])*a
            y = p1[1] + (p2[1] - p1[1])*a

            if not self.CheckCollision(x, y):
                return False

        return True


    def CheckCollision(self, x, y):

        # Iterate through all occupied points; check distance from all
        # Exit when only 1 occ point too close is found
        for pt in self.occupied:
            dx = pt[0] - x
            dy = pt[1] - y

            dist = dx*dx + dy*dy

            if dist < self.dist_thresh*self.dist_thresh:
                return False
        
        return True

    def PreProcessPlan(self):
        # Not sure if still used/needed
        # Intended to create data types to be stored for search
        index = 0

        for pt in self.waypoints:
            # index of point, (x,y), back_trace_index, cost
            self.planning_points.append((index, pt, -1, 0))

        self.planning_edges = self.edges

    def GetClosestPlanningPoint(self, point):
        min_dist = 999
        min_index = -1

        index=0
        for pt in self.planning_points:
            dist = self.SquareDist(point, pt[1])
            
            if dist < min_dist:
                min_dist = dist
                min_index = index

            index = index + 1

        return min_index
 

    # return true if found
    def findIndexIn4ValArray(self, planning_points, indexToFind):

        for (ind, pt, prevInd, cost) in planning_points:
            if ind == indexToFind:
                return True

        return False


    # *******************************
    # @param starting point coordinates
    # @param goal point coordinates 
    # @returns the Optimal Path list as integer vertex indices
    # *******************************
    def AStar(self, start, end):
        print("Starting A*")

        # Get integer vertex indices for start and goal
        start_ind = self.GetClosestPlanningPoint(start)
        end_ind = self.GetClosestPlanningPoint(end)

        # Get approximated coordinates for start and goal
        _ind0, pt0, prevInd0, _cost0 = self.planning_points[start_ind]
        _ind3, pt3, prevInd3, _cost3 = self.planning_points[end_ind]

        # List of points to avoid duplicate traversal
        closed_set = []

        # *********************************
        # Initializing AStar Algorithm
        curr_vertex = Vertex(start_ind, None, 0)
        # Distance to reach current vertex from start vertex
        curr_dist = 0 
        curr_queueCost = self.distanceBetweenPoints(pt3, pt0)

        pQueue = Queue.PriorityQueue()
        pQueue.put((curr_queueCost, curr_vertex, curr_dist))
        closed_set.append(curr_vertex)

        print("AStar Initialized\n")
        print("AStar traversing from (%.2f,%.2f)[%d] to (%.2f,%.2f)[%d]\n" % (pt0[0], pt0[1], start_ind, pt3[0], pt3[1], end_ind))

        for _ in range(0, 100000):

            # Retrieve current vertex info from priority queue
            curr_queueCost, curr_vertex, curr_dist = pQueue.get()
            curr_ind = curr_vertex.index

            if curr_ind == end_ind:
                break

            # Generate the neighbour vertices
            neighbourNodes = self.GetNeighbours(curr_ind)
            
            # Get the current vertex euler coordinate
            _ind1, pt1, _prevInd1, _cost1 = self.planning_points[curr_ind]

            # For each neighbouring vertex
            for neighbourNode in neighbourNodes:
                # Get the current neighbour vertex euler coordinate
                _ind2, pt2, _prevInd2, _cost2 = self.planning_points[neighbourNode]

                # If aleady calculated, skip
                if neighbourNode in closed_set:
                    continue

                # AStar Heuristics Calculations
                distToGoal = self.distanceBetweenPoints(pt3, pt2)

                heuristicOfNeighbour = distToGoal
                distToNeighbour = self.distanceBetweenPoints(pt2, pt1)

                # Add neighbour to priority queue
                next_queueCost = heuristicOfNeighbour + distToNeighbour + curr_dist
                next_vertex = Vertex(neighbourNode, curr_vertex, distToNeighbour)
                next_dist = distToNeighbour + curr_dist
                pQueue.put((next_queueCost, next_vertex, next_dist))

                # Put in closed_set, mark as traversed
                closed_set.append(neighbourNode)

            print("AStar vertex processed: [%d]" % curr_ind)
            ## End For
            
        ### End While

        if curr_ind != end_ind:
            print("Path could not be found")

        # Find the optimal path through back trace
        optimalPath = []
        optimalPathCost = 0

        opt_curr_vertex = curr_vertex
        while opt_curr_vertex is not None:

            optimalPath.insert(0, opt_curr_vertex.index)
            optimalPathCost += opt_curr_vertex.distToPrev

            opt_curr_vertex = opt_curr_vertex.prevVertex
        
        # Print out Optimal Path
        print("\n")
        print("-------------------------------------------------")
        print('\x1b[6;30;42m' + "Found Optimal Path with cost of [%d]" % optimalPathCost + '\x1b[0m')
        print("OptimalPath: [")
        for index in optimalPath:
            print("%d, " % index, end='')
        print("]")
        print("-------------------------------------------------")
        print("\n\n")

        self.goalChanged = False
        return optimalPath


    # ******************************************
    # @param first point euler coordinates
    # @param second point euler coordinates
    # @returns euler distance between two points
    # ******************************************
    def distanceBetweenPoints(self, pt1, pt2):
        distToGoal_x = pt2[0] - pt1[0]
        distToGoal_y = pt2[1] - pt1[1]

        distToGoal = math.sqrt(distToGoal_x**2 + distToGoal_y**2)
        return distToGoal
    

    # ******************************************
    # @param index of a vertex on the graph
    # @returns list of all neighbour vertex indices
    # ******************************************
    def GetNeighbours(self, ind):

        neighbourNodes = []
        # search through all the edges linked to the current point
        for x, y in self.planning_edges:

            if x == ind:
                neighbourNodes.append(y)
            elif y == ind:
                neighbourNodes.append(x)

        return neighbourNodes

    # *****************************************
    # Publishes optimal path visualization markers for rviz
    # @param optimal list of vertex to traverse on graph
    # *****************************************
    def VisualizePlan(self, path):
        msg = visualization_msgs.msg.Marker()
        mark_index = 0
        stamp = rospy.Time.now()

        scale = geometry_msgs.msg.Vector3()
        scale.x = 0.05
        scale.y = 0.05
        scale.z = 0.05

        color = std_msgs.msg.ColorRGBA()
        color.r = 0
        color.g = 1
        color.b = 0
        color.a = 1.0

        msg.header.frame_id = '/my_frame'
        msg.header.stamp = stamp
        msg.ns = 'plan'
        msg.id = mark_index
        msg.type = visualization_msgs.msg.Marker.LINE_STRIP
        msg.action = visualization_msgs.msg.Marker.ADD
        msg.color = color
        msg.scale = scale
        msg.pose.orientation.w = 1

        for p in path:
            wp = self.planning_points[p]

            point = geometry_msgs.msg.Point()
            point.x = wp[1][0]
            point.y = wp[1][1]
            point.z = 0.2

            msg.points.append(point)



        self.plan_pub.publish(msg)

    # *****************************************
    # Publishes optimal path as a geometry_msgs.msg.PoseArray
    # @param optimal list of vertex to traverse on graph
    # *****************************************
    def PublishPath(self, path):

        pose_array_msg = geometry_msgs.msg.PoseArray()
        pose_array = []
        for p in path:
            pt = self.planning_points[p][1]
            curr_pose = geometry_msgs.msg.Pose()
            curr_pose.position.x = pt[0]
            curr_pose.position.y = pt[1]

            pose_array.append(curr_pose)

        pose_array_msg.poses = pose_array

        self.plan_array_pub.publish(pose_array_msg)
        


    def main(self):
        while not rospy.is_shutdown():
            time.sleep(0.2)

            if (self.map is not None) and (not self.processed):
                self.ProcessMap()
                
                print("Pre-Processed; Starting A*")

            if self.goalChanged and self.processed:
                self.PreProcessPlan()
                path = self.AStar(self.startPoint, self.endPoint)
                self.VisualizePlan(path)
                
                self.PublishPath(path)
                print('Visualized Plan')

if __name__ == "__main__":
    
    gb = GraphBuilder()
    gb.main()