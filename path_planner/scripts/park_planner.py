#!/usr/bin/env python

import rospy, math
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg
from geometry_msgs.msg import PolygonStamped, Point32

def makebox(ax, ay, bx, by, cx, cy, dx, dy):

  pub = rospy.Publisher('/test_optim_node/obstacles', ObstacleArrayMsg, queue_size=1) # /park_planner/lines
  #pub = rospy.Publisher('/p3dx/move_base/TebLocalPlannerROS/obstacles', ObstacleArrayMsg, queue_size=1)
  rospy.init_node("park_planner")


  obstacle_msg = ObstacleArrayMsg() 
  obstacle_msg.header.stamp = rospy.Time.now()
  obstacle_msg.header.frame_id = "odom" # CHANGE HERE: odom/mapobstacle_msg.obstacles.append(ObstacleMsg())
  obstacle_msg.obstacles.append(ObstacleMsg())
  obstacle_msg.obstacles.append(ObstacleMsg())
  obstacle_msg.obstacles.append(ObstacleMsg())
  obstacle_msg.obstacles[0].id = 1
  obstacle_msg.obstacles[1].id = 2
  obstacle_msg.obstacles[2].id = 3
  A = Point32()
  A.x = ax
  A.y = ay
  B = Point32()
  B.x = bx
  B.y = by
  C = Point32()
  C.x = cx
  C.y = cy
  D = Point32()
  D.x = dx
  D.y = dy
  
  obstacle_msg.obstacles[0].polygon.points = [A, B]
  obstacle_msg.obstacles[1].polygon.points = [B, C]
  obstacle_msg.obstacles[2].polygon.points = [C, D]
  #obstacle_msg.obstacles[0].polygon.points = [v1, v2, v3, v4]

  r = rospy.Rate(10) # 10hz
  t = 0.0
  while not rospy.is_shutdown():
    
    """# Vary y component of the point obstacle
    obstacle_msg.obstacles[0].polygon.points[0].y = 1*math.sin(t)
    t = t + 0.1"""
    
    pub.publish(obstacle_msg)
    
    r.sleep()



if __name__ == '__main__': 
  try:
    makebox(0,0,0,1,1,1,1,0)
  except rospy.ROSInterruptException:
    pass
