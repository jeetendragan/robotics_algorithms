#!/usr/bin/env python
import math
import rospy
import tf
import random
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from sensor_msgs.msg import LaserScan
import sys
from visualization_msgs.msg import Marker
from tf.transformations import euler_from_quaternion
import time


########################################################################################
#For instructors
"""
The lines are being published to the topic line_publisher
Points are being published to the topic point_publisher
"""
########################################################################################


class Vector:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __init__(self, sourcePoint, destPoint):
        self.x = destPoint.x - sourcePoint.x
        self.y = destPoint.y - sourcePoint.y

    def getAngle(self):
        return math.atan2(self.y, self.x)
        
    def getLength(self):
        xsq = math.pow(self.x, 2)
        ysq = math.pow(self.y, 2)
        return math.sqrt(xsq + ysq)

    def sameDirection(self, otherVect):
        thisAngle = self.getAngle()
        otherAngle = otherVect.getAngle()
        if abs(thisAngle - otherAngle) < 0.15:
            return True
        else:
            return False

class Perception:
    def __init__(self):
        rospy.init_node("perception")
        
        self.startLoc = Point(-8, -2, 0)
        self.goalLoc = Point(4.5, 9, 0)
        self.orignalPathVect = Vector(self.startLoc, self.goalLoc)
        self.inGoalSeek = True
        self.allLineEndPoints = []
        self.visionRadius = 2
        self.visionAngle = math.pi / 3

        rospy.Subscriber('/base_scan', LaserScan, self.laser_input_callback)
        rospy.Subscriber('/odom', Odometry, self.location_call_back)
        self.point_publisher = rospy.Publisher("/point_publisher", Marker, queue_size = 1)
        self.line_publisher = rospy.Publisher("/line_publisher", Marker, queue_size = 1)
        self.global_publisher_lines = rospy.Publisher('/global_publisher_lines', Marker, queue_size = 1)
        self.global_publisher_points = rospy.Publisher('/global_publisher_points', Marker, queue_size = 1)
        self.robot_front_vision_publisher = rospy.Publisher("/robot_front_vision_publisher", Marker, queue_size = 1)
        #self.global_publisher_intr_lines = rospy.Publisher('/global_publisher_intr_lines', Marker, queue_size = 1)
        self.vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.spin()

    def laser_input_callback(self, data):
        message = Marker()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = '/odom'
        message.type = message.POINTS
        message.action = message.ADD
        message.lifetime = rospy.Duration(10)
        message.scale.x = .1
        message.scale.y = .1
        message.color.a = 1.0
        message.color.r = 1.0
        self.points = self.get_cartesian_coords(data)
        self.points.append(Point(1,0,0))
        message.points = self.points
        self.point_publisher.publish(message)

        if len(self.points) == 0:
            print("No obstacles closeby")
            return
        
        lineMessage = Marker()
        lineMessage.header.stamp = rospy.Time.now()
        lineMessage.header.frame_id = '/odom'
        lineMessage.type = lineMessage.LINE_LIST
        lineMessage.action = lineMessage.ADD
        lineMessage.lifetime = rospy.Duration(10)
        lineMessage.scale.x = .1
        lineMessage.color.a = 1.0
        lineMessage.color.r = 0.0
        lineMessage.color.g = 1.0
        lineMessage.color.b = 1.0
        self.allLineEndPoints = self.getLines(self.points)
        onlyPoints = []
        print("Lines retrived: "+str(len(self.allLineEndPoints)))
        for lineEndPoints in self.allLineEndPoints:
            onlyPoints.append(lineEndPoints[0])
            onlyPoints.append(lineEndPoints[1])
        lineMessage.points = onlyPoints
        self.line_publisher.publish(lineMessage)

        """vel_msg = Twist()
        vel_msg.linear.x = 2
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
        self.vel_publisher.publish(vel_msg)"""

    def convertLineEndPointsToList(self, lines):
        onlyPoints = []
        for lineEndPoints in lines:
            onlyPoints.append(lineEndPoints[0])
            onlyPoints.append(lineEndPoints[1])
        return onlyPoints
    
    def get_rotation(self, odoMsg):
        orientation_q = odoMsg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(orientation_list)

        dx = self.goalLoc.x - odoMsg.pose.pose.position.x
        dy = self.goalLoc.y - odoMsg.pose.pose.position.y
        self.target_angle = math.atan2(dy, dx)

    def getClosestPointOnLine(A, B, P):
        a_to_p = [P.x - A.x, P.y - A.y]     
        a_to_b = [B.x - A.x, B.y - A.y]    
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          
        atb2 = a_to_b[0]**2 + a_to_b[1]**2

        atp_dot_atb = a_to_p[0]*a_to_b[0] + a_to_p[1]*a_to_b[1]

        t = atp_dot_atb / atb2

        return Point(A.x + a_to_b[0]*t, A.y + a_to_b[1]*t, 0)
    
    def getObstaclesInRobotVision(self):
        obstacles = []
        for linePoints in self.allLineEndPoints:
            if self.isLineInFront(linePoints):
                obstacles.append(linePoints)
        return obstacles

    def isLineInFront(self, linePoints):
        rightBound = self.visionRadius * math.cos(self.visionAngle)
        leftBound = -rightBound
        return self.doLineSegmentsOverlap1D(leftBound, rightBound, linePoints[0].y, linePoints[1].y)

    def doLineSegmentsOverlap1D(self, leftBound, rightBound, y1, y2):
        if y2 <= leftBound or y1 >= rightBound:
            return False
        else:
            return True

    def location_call_back(self, data):
        currentLoc = data.pose.pose.position
        currLoCStr = str(round(currentLoc.x,2)) + ", "+str(round(currentLoc.y,2))+", "+str(round(currentLoc.z,2))
        distBet = round(self.getDistBet(self.goalLoc, currentLoc),2)
        print("CurrentLoc: "+currLoCStr+", DistToGoal: "+str(distBet)+", AtGoal: "+str(distBet < 0.1))

        # get any obstacle in the robot vision
        #obstacles = self.getObstaclesInRobotVision()
        #obstaclePoints = self.convertLineEndPointsToList(obstacles)
        #self.publishRobotFrontVision(obstaclePoints)
        
        obstacle = self.getLineClosestToRobot(currentLoc)
        if obstacle == None:
            print("No obstacles in vision")
            return
        p1 = obstacle[0]
        p2 = obstacle[1]
        robotYaw = self.getRobotYaw(data)
        robotYawInDeg = robotYaw*180 / math.pi
        print("Robot Yaw: "+str(robotYaw)+", Deg: "+str(robotYawInDeg))
        p1St = str(round(p1.x,2)) + ", "+str(round(p1.y,2)) +" ---> "+str(round(p2.x,2)) + ", "+str(round(p2.y,2))
        obstInGlobal = self.convertLineToGlobalFromRobotOdo(obstacle[0], obstacle[1], data)
        self.publishGlobalPoints(obstInGlobal, currentLoc)
        p1 = obstInGlobal[0]
        p2 = obstInGlobal[1]
        p1Stg = str(round(p1.x,2)) + ", "+str(round(p1.y,2)) +" ---> "+str(round(p2.x,2)) + ", "+str(round(p2.y,2))
        print("Local: "+p1St)
        print("Global: "+p1Stg)

        return
        pointsInGlobal = self.convertPointsInRobotSystemToGlobal(self.points, data)
        self.publishGlobalPoints(pointsInGlobal, currentLoc)
        # convert the lines and the robot location into the global frame(currently in robot frame), and publish it to /global_publish
        """linesInGlobal = self.convertLinesToGlobalFromRobotOdo(data)
        points = self.convertLineEndPointsToList(linesInGlobal)
        #points.append(currentLoc)
        self.publishGlobalPoints(points, currentLoc)"""
        
        if self.atGoal(currentLoc):
            return

        if self.inGoalSeek:
            print("####### IN GOAL SEEK #########")
            lineInRobotSystem = self.getLineClosestToRobot(currentLoc)
            if lineInRobotSystem != None:
                ##lineInGlobal = self.convertLineToGlobalFromRobotOdo(lineInRobotSystem[0], lineInRobotSystem[1], data)
                ##ineSegmentToGoal = [currentLoc, self.goalLoc]
                #self.publishIntersectionInfo(lineInGlobal, lineSegmentToGoal)
                ##obstLine = str(round(lineInGlobal[0].x,2)) + ", "+str(round(lineInGlobal[0].y,2)) +" ---> "+str(round(lineInGlobal[1].x,2)) + ", "+str(round(lineInGlobal[1].y,2))
                ##vectToGoal = str(round(lineSegmentToGoal[0].x,2)) + ", "+str(round(lineSegmentToGoal[0].y,2)) +" ---> "+str(round(lineSegmentToGoal[1].x,2)) + ", "+str(round(lineSegmentToGoal[1].y,2))
                print("Obstacle encountered in goal seek")
                #print("Obstacle: "+obstLine)
                #print("VEctToGoal: "+vectToGoal)
                #if self.doIntersect(lineSegmentToGoal[0], lineSegmentToGoal[1], lineInGlobal[0], lineInGlobal[1]):
                #    print("There is an obstacle and the DP intersects with it..Changing state to wall follow")
                #    self.inGoalSeek = False
                #else:
                #print("The Current path does not intersect with obstacle, stauing in goal seek")
                self.inGoalSeek = False
                print("Moving to wall follow")
                ############################
                #This is the goal seek state
                ############################
                """print("\t In Goal Seek state, towards the goal-------")
                vel_msg = Twist()
                self.get_rotation(data)
                target_rad = self.target_angle 
                print("\tTarget rad: "+str(target_rad)+", Current Yaw: "+str(self.yaw))
                if abs(target_rad - self.yaw) > 0.01:
                    print("\tRotating...")
                    vel_msg.angular.z = 0.5*(target_rad - self.yaw)
                    self.vel_publisher.publish(vel_msg)
                else:
                    print("\tMoving forward...")
                    vel_msg.linear.x = 0.5
                    self.vel_publisher.publish(vel_msg)"""
            else:
                ############################
                #This is the goal seek state
                ############################
                print("\t In Goal Seek state, towards the goal")
                vel_msg = Twist()
                self.get_rotation(data)
                target_rad = self.target_angle 
                print("\tTarget rad: "+str(target_rad)+", Current Yaw: "+str(self.yaw))
                if abs(target_rad - self.yaw) > 0.01:
                    print("\tRotating...")
                    vel_msg.angular.z = 0.5*(target_rad - self.yaw)
                    self.vel_publisher.publish(vel_msg)
                else:
                    print("\tMoving forward...")
                    vel_msg.linear.x = 0.5
                    self.vel_publisher.publish(vel_msg)
        else:
            print("############ IN WALL FOLLOW #######")
            # wall following state
            currToGoal = Vector(currentLoc, self.goalLoc)
            lineInRobotSystem = self.getLineClosestToRobot(currentLoc)
            if self.orignalPathVect.sameDirection(currToGoal) and lineInRobotSystem == None:
                self.inGoalSeek = True
                print("Transitioning to goal seek from wall follow")
            else:
                ###################################################################
                # wall hugging algorithm - keep moving parallel to the closest wall
                ###################################################################
                lineInRobotSystem = self.getLineClosestToRobot(currentLoc)
                if lineInRobotSystem == None:
                    print("Robot is not close to any line. No movement..")
                    self.inGoalSeek = True
                    return
                #lineInGlobalSystem = self.convertLineToGlobalFromRobotOdo(lineInRobotSystem[0], lineInRobotSystem[1], data)
                lineInRobotSystem = self.sortPoints(lineInRobotSystem)
                lineVect = Vector(lineInRobotSystem[0], lineInRobotSystem[1])
                lineAngle = lineVect.getAngle()
                p1 = str(round(lineInRobotSystem[0].x,3)) +", "+str(round(lineInRobotSystem[0].y,3))
                p2 = str(round(lineInRobotSystem[1].x,3)) +", "+str(round(lineInRobotSystem[1].y,3))
                print("Line coords: "+p1+" --> "+p2)
                absAngl = lineAngle - math.pi/2
                absInDeg = round(180 * absAngl/math.pi,3)
                print("Line angle: "+str(lineAngle)+", Deg: "+str(180*lineAngle/math.pi)+", ABS: "+str(absAngl)+", "+str(absInDeg))
                #robotAngle = self.getRobotYaw(data)
                vel_msg = Twist()
                if (abs(lineAngle - math.pi) > 0.15):
                    # rotate the robot so that its yaw matches with the lineAngle
                    rotateBy = 0.05*(lineAngle-math.pi)
                    vel_msg.linear.x = 0
                    vel_msg.angular.z = rotateBy
                    print("Rotating robot by : "+str(rotateBy))
                else:
                    # move the robot in a straight line
                    vel_msg.linear.x = 0.5
                    print("Moving straight...")
                self.vel_publisher.publish(vel_msg)

        #time.sleep(1)
        
        """pointClosestToRobotOnObst = self.getClosestPointOnLine(lineInRobotSystem[0], lineInRobotSystem[1], currentLoc)
        vectorToObs = Vector(currentLoc, pointClosestToRobotOnObst)
        if vectorToObs.getLength() > 0.7:
            # rotate the robot with an angle such that robot matches with the vector connecting the point
            angle = vectorToObs.getAngle()
        else:"""
        """if self.inGoalSeek:
            obstacleLinePoints = self.getObstacleInViewDuringGoalSeek(currentLoc)
            if obstacleLinePoints == None:
                # keep moving towards the goal
                vel_msg = Twist()
                vel_msg.linear.x = 1
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0
                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                vel_msg.angular.z = 0
                self.vel_publisher.publish(vel_msg)
            else:
                # need to stop and figure out the direction of the wall

        else: # in wall follow state
            obstacleLinePoints = self.getObstacleInViewDuringGoalSeek(currentLoc)
            if obstacleLinePoints == None:
                # switch to goal seek
            else:
                # keep moving in the direction of the wall""" 

    def convertPointsInRobotSystemToGlobal(self, points, robotOdo):
        pointsInGlobal = []
        robotLoc = robotOdo.pose.pose.position
        robotYaw = self.getRobotYaw(robotOdo)
        for point in points:
            startX = point.x * math.cos(robotYaw) - point.y * math.sin(robotYaw) + robotLoc.x
            startY = point.x * math.sin(robotYaw) + point.y * math.cos(robotYaw) + robotLoc.y
            newStart = Point(startX, startY, 0)
            pointsInGlobal.append(newStart)
        return pointsInGlobal

    def publishGlobalPoints(self, points, robotLoc):
        lineMessage = Marker()
        lineMessage.header.stamp = rospy.Time.now()
        lineMessage.header.frame_id = '/map'
        lineMessage.type = lineMessage.LINE_LIST
        lineMessage.action = lineMessage.ADD
        lineMessage.lifetime = rospy.Duration(10)
        lineMessage.scale.x = .1
        #lineMessage.scale.y = .1
        lineMessage.color.a = 1.0
        lineMessage.color.r = 0.0
        lineMessage.color.g = 1.0
        lineMessage.color.b = 0.0
        lineMessage.points = points
        self.global_publisher_lines.publish(lineMessage)

        lineMessage = Marker()
        lineMessage.header.stamp = rospy.Time.now()
        lineMessage.header.frame_id = '/map'
        lineMessage.type = lineMessage.POINTS
        lineMessage.action = lineMessage.ADD
        lineMessage.lifetime = rospy.Duration(10)
        lineMessage.scale.x = .1
        lineMessage.scale.y = .1
        lineMessage.color.a = 1.0
        lineMessage.color.r = 1.0
        lineMessage.color.g = 0.0
        lineMessage.color.b = 0.0
        lineMessage.points = [robotLoc, Point(0,0,0), Point(1, 0, 0)]
        self.global_publisher_points.publish(lineMessage)

    def publishRobotFrontVision(self, obstaclePoints):
        lineMessage = Marker()
        lineMessage.header.stamp = rospy.Time.now()
        lineMessage.header.frame_id = '/odom'
        lineMessage.type = lineMessage.LINE_LIST
        lineMessage.action = lineMessage.ADD
        lineMessage.lifetime = rospy.Duration(10)
        lineMessage.scale.x = .2
        lineMessage.color.a = 1.0
        lineMessage.color.r = 0.0
        lineMessage.color.g = 1.0
        lineMessage.color.b = 0.0
        lineMessage.points = obstaclePoints
        self.robot_front_vision_publisher.publish(lineMessage)
    
    def publishIntersectionInfo(self, line1, line2):
        lineMessage = Marker()
        lineMessage.header.stamp = rospy.Time.now()
        lineMessage.header.frame_id = '/map'
        lineMessage.type = lineMessage.LINE_LIST
        lineMessage.action = lineMessage.ADD
        lineMessage.lifetime = rospy.Duration(10)
        lineMessage.scale.x = .1
        lineMessage.color.a = 1.0
        lineMessage.color.r = 0.0
        lineMessage.color.g = 0.0
        lineMessage.color.b = 1.0
        points = self.convertLineEndPointsToList([line1, line2]) 
        lineMessage.points = points
        self.global_publisher_intr_lines.publish(lineMessage)
        
    def getRobotYaw(self, robotOdo):
        orientation_q = robotOdo.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        return yaw

    def sortPoints(self, points):
        bestPoint1 = points[0]
        bestPoint2 = points[1]
        dx = abs(bestPoint1.x - bestPoint2.x)
        isVertical = True
        if dx < 0.15:
            # line is vertical - find a point that has the lowest y, and the heighest y values
            isVertical = True
        else:
            # line is horizontal - find a point that has the lowest x, and heightest x values
            isVertical = False
        
        smallestPoint = Point(sys.maxsize, sys.maxsize, 0)
        largestPoint = Point(-1*2000000, -200000, 0)
        for point in points:
            if isVertical:
                if point.y < smallestPoint.y:
                    smallestPoint = point
                if point.y > largestPoint.y:
                    largestPoint = point
            else:
                if point.x < smallestPoint.x:
                    smallestPoint = point
                if point.x > largestPoint.x:
                    largestPoint = point
        if isVertical:
            return [largestPoint, smallestPoint]
        else:
            return [smallestPoint, largestPoint]
        
    def convertLinesToGlobalFromRobotOdo(self, robotOdo):
        if self.allLineEndPoints == None or len(self.allLineEndPoints) == 0:
            return None

        robotLoc = robotOdo.pose.pose.position
        robotYaw = self.getRobotYaw(robotOdo)

        linesInGlobal = []
        for lineEndPoints in self.allLineEndPoints:
            lineEndPointsGlobal = []
            start = lineEndPoints[0]
            end = lineEndPoints[1]
            startX = start.x * math.cos(robotYaw) - start.y * math.sin(robotYaw) + robotLoc.x
            startY = start.y * math.cos(robotYaw) + start.x * math.sin(robotYaw) + robotLoc.y
            newStart = Point(startX, startY, 0)

            endX = end.x * math.cos(robotYaw) - end.y * math.sin(robotYaw) + robotLoc.x
            endY = end.y * math.cos(robotYaw) + end.x * math.sin(robotYaw) + robotLoc.y
            newEnd = Point(endX, endY, 0)

            lineEndPointsGlobal.append(newStart)
            lineEndPointsGlobal.append(newEnd)
            linesInGlobal.append(lineEndPointsGlobal)
    
        return linesInGlobal

    def convertLineToGlobalWithoutRotation(self, start, end, robotOdo):
        robotLoc = robotOdo.pose.pose.position
        robotYaw = self.getRobotYaw(robotOdo)
        startX = start.x + robotLoc.x
        startY = start.y + robotLoc.y
        newStart = Point(startX, startY, 0)

        endX = end.x  + robotLoc.x
        endY = end.y  + robotLoc.y
        newEnd = Point(endX, endY, 0)
        return [newStart, newEnd]
    

    def convertLineToGlobalFromRobotOdo(self, start, end, robotOdo):
        robotLoc = robotOdo.pose.pose.position
        robotYaw = self.getRobotYaw(robotOdo)
        startX = start.x * math.cos(robotYaw) - start.y * math.sin(robotYaw) + robotLoc.x
        startY = start.x * math.sin(robotYaw) + start.y * math.cos(robotYaw) + robotLoc.y
        newStart = Point(startX, startY, 0)

        endX = end.x * math.cos(robotYaw) - end.y * math.sin(robotYaw) + robotLoc.x
        endY = end.x * math.sin(robotYaw) + end.y * math.cos(robotYaw) + robotLoc.y
        newEnd = Point(endX, endY, 0)
        return [newStart, newEnd]
    
    def getLineClosestToRobot(self, currentLoc):
        if self.allLineEndPoints == None or len(self.allLineEndPoints) == 0:
            return None
        closestPointDist = sys.maxsize
        closestLinePoints = None
        for lineEndPoints in self.allLineEndPoints:
            dist = self.getDistanceTo(lineEndPoints[0], lineEndPoints[1], currentLoc)
            if dist < closestPointDist:
                closestPointDist = dist
                closestLinePoints = lineEndPoints
        
        return closestLinePoints

    def getObstacleInViewDuringGoalSeek(self, currentLoc):
        if self.allLineEndPoints == None or len(self.allLineEndPoints) == 0:
            return None
        
        for lineEndPoints in self.allLineEndPoints:
            intersectionPoint = self.doLineSegmentsIntersect(self.goalLoc, self.currentLoc, lineEndPoints[0], lineEndPoints[1])
            if intersectionPoint != None:
                return lineEndPoints
        
        return None

    def doLineSegmentsIntersect(self, line1Point1, line1Point2, line2Point1, line2Point2):
        intersectPoint = self.doLinesIntersect(line1Point1, line1Point2, line2Point1, line2Point2)
        if intersectPoint == None:
            return None
        
        # does the line segment fall within the bounds of the line
        line1MinX = min(line1Point1.x, line1Point2.x)
        line1MinY = min(line1Point1.y, line1Point2.y)
        line1MaxX = max(line1Point1.x, line1Point2.x)
        line1MaxY = max(line1Point1.y, line1Point2.y)

        line2MinX = min(line1Point1.x, line1Point2.x)
        line2MinY = min(line1Point1.y, line1Point2.y)
        line2MaxX = max(line1Point1.x, line1Point2.x)
        line2MaxY = max(line1Point1.y, line1Point2.y)

        inLine1 = line1MinX <= intersectPoint.x and intersectPoint.x <= line1MaxX and line1MinY <= intersectPoint.y and intersectPoint.y <= line1MaxY
        inLine2 = line2MinX <= intersectPoint.x and intersectPoint.x <= line2MaxX and line2MinY <= intersectPoint.y and intersectPoint.y <= line2MaxY

        if inLine1 and inLine2:
            return intersectPoint
        else:
            return None

    def onSegment(self, p, q, r): 
        if ( (q.x <= max(p.x, r.x)) and (q.x >= min(p.x, r.x)) and 
            (q.y <= max(p.y, r.y)) and (q.y >= min(p.y, r.y))): 
            return True
        return False

    def doLinesIntersect(self, line1Point1, line1Point2, line2Point1, line2Point2):
        xdiff = (line1Point1.x - line1Point2.x, line2Point1.x - line2Point2.x)
        ydiff = (line1Point1.y - line1Point2.y, line2Point1.y - line2Point2.y)

        def det(a, b):
            return a[0] * b[1] - a[1] * b[0]

        div = det(xdiff, ydiff)
        if div == 0:
            return None

        d = (det(*line1), det(*line2))
        x = det(d, xdiff) / div
        y = det(d, ydiff) / div
        return Point(x, y, 0)

    def orientation(self, p, q, r):         
        val = (float(q.y - p.y) * (r.x - q.x)) - (float(q.x - p.x) * (r.y - q.y)) 
        if (val > 0): 
            # Clockwise orientation 
            return 1
        elif (val < 0): 
            # Counterclockwise orientation 
            return 2
        else: 
            # Colinear orientation 
            return 0

    def doIntersect(self, p1 ,q1 ,p2 ,q2):
        o1 = self.orientation(p1, q1, p2) 
        o2 = self.orientation(p1, q1, q2) 
        o3 = self.orientation(p2, q2, p1) 
        o4 = self.orientation(p2, q2, q1) 
    
        # General case 
        if ((o1 != o2) and (o3 != o4)): 
            return True
    
        # Special Cases 
    
        # p1 , q1 and p2 are colinear and p2 lies on segment p1q1 
        if ((o1 == 0) and self.onSegment(p1, p2, q1)): 
            return True
    
        # p1 , q1 and q2 are colinear and q2 lies on segment p1q1 
        if ((o2 == 0) and self.onSegment(p1, q2, q1)): 
            return True
    
        # p2 , q2 and p1 are colinear and p1 lies on segment p2q2 
        if ((o3 == 0) and self.onSegment(p2, p1, q2)): 
            return True
    
        # p2 , q2 and q1 are colinear and q1 lies on segment p2q2 
        if ((o4 == 0) and self.onSegment(p2, q1, q2)): 
            return True
    
        # If none of the cases 
        return False

    def atGoal(self, currentLocationPoint):
        dist = self.getDistBet(self.goalLoc, currentLocationPoint)
        return dist < 0.1

    def getDistBet(self, point1, point2):
        dx = point1.x - point2.x
        dy = point1.y - point2.y
        return math.sqrt(math.pow(dx, 2) + math.pow(dy, 2))

    def get_cartesian_coords(self, data):
        # data.ranges gives the distance to an obstacle for an angle
        # the max angle(degree) is 90, and the min angle is -90. The step is 0.50 degrees.
        # the max angle(radian) is +1.57 and min is -1.57. THe step is 0.0087
        ranges = data.ranges
        currentAngle = data.angle_max
        stepSize = data.angle_increment
        points = []
        for dist in ranges:
            if dist > self.visionRadius:
                currentAngle -= stepSize
                continue
            xCoord = dist * math.sin(currentAngle)
            yCoord = dist * math.cos(currentAngle)
            point = Point(xCoord, yCoord, 0)
            points.append(point)
            currentAngle -= stepSize
        return points

    def getLines(self, points):
        leftOverPoints = []
        for point in points:
            leftOverPoints.append(point)
        lines = []
        for i in range(2):
            if len(leftOverPoints) <= 1:
                print("No points left!")
                break
            (outliers, bestPoint1, bestPoint2) = self.ransack(leftOverPoints, 25, 0.1, 10)
            if bestPoint1 == None:
                continue
            lines.append((bestPoint1, bestPoint2))
            leftOverPoints = outliers
        return lines

    def ransack(self, points, th_tryKLines, th_inlinerDistance, th_minInlinerPointCount):
        bestInners = []
        bestOuters = []
        bestPoint1 = None
        bestPoint2 = None
        for tryI in range(1, th_tryKLines):
            pt1Idx = random.randint(0, len(points)-1)
            pt2Inx = self.getPointInRangeWithout(0, len(points) - 1, pt1Idx)

            point1 = points[pt1Idx]
            point2 = points[pt2Inx]

            (inners, outers) = self.getInnersFor(point1, point2, points, th_inlinerDistance)
            if len(inners) > th_minInlinerPointCount:
                # atleast have 10 points in a line
                if len(inners) > len(bestInners):
                    bestInners = inners
                    bestOuters = outers
                    bestPoint1 = point1
                    bestPoint2 = point2
        
        # for the line with the best fit, get the extereme points on the line
        if bestPoint1 != None:
            (bestPoint1, bestPoint2) = self.getExteremePointsOnLine(bestPoint1, bestPoint2, bestInners)

        return (bestOuters, bestPoint1, bestPoint2)

    def getPointInRangeWithout(self, min, max, excluding):
        while True:
            num = random.randint(min, max)
            if num == excluding:
                continue
            return num

    def getExteremePointsOnLine(self, bestPoint1, bestPoint2, bestInners):
        # is the line vertical?
        dx = abs(bestPoint1.x - bestPoint2.x)
        isVertical = True
        if dx < 0.1:
            # line is vertical - find a point that has the lowest y, and the heighest y values
            isVertical = True
        else:
            # line is horizontal - find a point that has the lowest x, and heightest x values
            isVertical = False
        
        smallestPoint = Point(sys.maxsize, sys.maxsize, 0)
        largestPoint = Point(-1*2000000, -200000, 0)
        for point in bestInners:

            if point.y < smallestPoint.y:
                smallestPoint = point
            if point.y > largestPoint.y:
                largestPoint = point

        return (smallestPoint, largestPoint)
    
    def getInnersFor(self, point1, point2, points, th_inlinerDistance):
        inners = []
        outers = []
        for point in points:
            dist = self.getDistanceTo(point1, point2, point)
            if dist <= th_inlinerDistance:
                inners.append(point)
            else:
                outers.append(point)
        return (inners, outers)


    def getDistanceTo(self, point1, point2, anyPoint):
        x1 = point1.x
        y1 = point1.y
        x2 = point2.x
        y2 = point2.y
        x0 = anyPoint.x
        y0 = anyPoint.y

        num = abs((y2 - y1) * x0 - (x2 - x1) * y0 + x2*y1 - y2*x1)
        den = math.sqrt(math.pow(y2 - y1, 2) + math.pow(x2 - x1, 2))
        dist = num / den
        return dist



if __name__ == "__main__":
    perceptionNode = Perception()