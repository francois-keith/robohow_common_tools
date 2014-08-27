#!/usr/bin/env python

# Listen to the constraints published and verify their meaning.
# publishes an error if they do not make sense

import roslib
import rospy

from robohow_common_msgs.msg import GaussianMixtureModel, GaussianDistribution,\
  MotionPhase, MotionModel

"""
Check
"""
def checkGaussianDistribution(gd):
  m = len(gd.mean)
  c = len(gd.cov)
  if c != m*m:
    rospy.logwarn('cov does not have the right size.')

def checkCDS(g):
  if g.type == 'CDS':
    if len(g.described_by_GMM) != 3:
      rospy.logwarn('CDS vectors do not have the expected size (3): ' + len(g.described_by_GMM))
    else:
      nStatesMaster=len(g.described_by_GMM[0].gaussian_dist)
      nVarMaster=len(g.described_by_GMM[0].gaussian_dist[0].mean)

      nStatesSlave=len(g.described_by_GMM[1].gaussian_dist)
      nVarSlave   =len(g.described_by_GMM[1].gaussian_dist[0].mean)

      nStatesCpl=len(g.described_by_GMM[2].gaussian_dist)
      nVarCpl   =len(g.described_by_GMM[2].gaussian_dist[0].mean)

      if nVarMaster != nVarCpl:
        rospy.logwarn('Master and coupling sizes do not correpond ' + 
          str(nVarMaster) + ' != ' + str(nVarCpl))
      if nVarSlave != nStatesCpl:
        rospy.logwarn('Slave and coupling sizes do not correpond ' + 
          str(nStatesSlave) + ' != ' + str(nStatesCpl))

      # check the GaussianDistribution
      for gd in (g.described_by_GMM[0].gaussian_dist):
        checkGaussianDistribution(gd)
      for gd in (g.described_by_GMM[1].gaussian_dist):
        checkGaussianDistribution(gd)
      for gd in (g.described_by_GMM[2].gaussian_dist):
        checkGaussianDistribution(gd)

def checkMotionModel(g):
  if g.type != 'CDS' and g.type != 'GMR':
    rospy.logwarn('Unknown gaussian model type ' + g.type)
  if g.type == 'CDS':
    checkCDS(g)

"""
ConstraintListener listens to the constraints message sent, 
and verifies that the given informations are correct
"""
class MotionModelListener:
  def __init__(self):
    # Create the node
    rospy.init_node('gmm_model_checker', anonymous=True)

    # Subscribe to the constraint publisher
    rospy.Subscriber("/gmm_model", MotionModel, self.callback)
    rospy.spin()  

  def callback(self, data):
    # check the constraint
    checkMotionModel(data)

# Start the listener
if __name__ == '__main__':
    MotionModelListener ()

