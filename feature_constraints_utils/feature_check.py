#!/usr/bin/env python

# Listen to the constraints published and verify their meaning.
# publishes an error if they do not make sense

import roslib
roslib.load_manifest('robohow_common_msgs')
import rospy

from robohow_common_msgs.msg import ConstraintConfig, Constraint, Feature

roslib.load_manifest('robohow_common_tools')
#from robohow_sot.utils import *

"""
Check the if a constraint has a meaning.
"""
def checkConstraintCoherence(c):
  if (len(pos_lo) != len(pos_hi)):
    rospy.logwarn('The size of the control command '+c.controller_id +' is incorrect')
  for i in range(i, len(pos_lo)):
    if pos_lo[i] > pos_up[i]:
      rospy.logwarn('The size of the control command '+c.controller_id +' is incorrect')


"""
Check the if a constraint has a meaning.
"""
def checkConstraintCoherence(c):

  LINE=0    #
  PLANE=1   #
  POINT=2   #
  VERSOR=3  #

  toolType  = c.tool_feature.type
  worldType = c.world_feature.type

  if c.function == 1: # angle
    # plane / plane
    if  (toolType == PLANE  and worldType == PLANE)  \
     or (toolType == PLANE  and worldType == VERSOR) \
     or (toolType == VERSOR and worldType == PLANE)  \
     or (toolType == VERSOR and worldType == VERSOR):
     return
    else:
      rospy.logwarn('Unable to compute the angle between ' + toolType  + '/' + worldType)

  if c.function == 2: # distance
    # line / line
    if   (toolType == LINE  and worldType == LINE ) \
      or (toolType == POINT and worldType == PLANE) \
      or (toolType == PLANE and worldType == POINT) \
      or (toolType == LINE  and worldType == POINT) \
      or (toolType == POINT and worldType == LINE)  \
      or (toolType == POINT and worldType == POINT):
      return
    else :
      rospy.logwarn('Unable to compute the angle between ' + toolType + '/' + worldType)

  if operation == 3: #'position'
    # point / point
    if (toolType == POINT and worldType == POINT):
      return
    else:
      rospy.logwarn('Unable to compute the angle between ' + toolType  + '/' + worldType)

  if operation == 4: # pointing_at
    return 


"""
ConstraintListener listens to the constraints message sent, 
and verifies that the given informations are correct
"""
class ConstraintListener:
  def __init__(self):
    # Create the node
    rospy.init_node('constraint_checker', anonymous=True)

    # Subscribe to the constraint publisher
    rospy.Subscriber("/constraint_config", ConstraintConfig, self.callback)
    rospy.spin()

  def callback(self, data):
    # check the constraint
    for c in data.constraints:
      checkConstraintCoherence(c)

# Start the listener
if __name__ == '__main__':
    ConstraintListener ()

