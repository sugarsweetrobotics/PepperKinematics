#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -*- Python -*-

"""
 @file ManipulatorCommonInterface_MiddleLevel_idl_examplefile.py
 @brief Python example implementations generated from ManipulatorCommonInterface_MiddleLevel.idl
 @date $Date$


"""

import omniORB
from omniORB import CORBA, PortableServer
import JARA_ARM, JARA_ARM__POA
import numpy as np

import  pepper_kinematics as pk
from pepper_kinematics import forward_kinematics as fk
from pepper_kinematics import inverse_kinematics as ik

import ssr, ssr__POA

class ManipulatorCommonInterface_Middle_i (JARA_ARM__POA.ManipulatorCommonInterface_Middle):
    """
    @class ManipulatorCommonInterface_Middle_i
    Example class implementing IDL interface JARA_ARM.ManipulatorCommonInterface_Middle
    """

    class InvalidArgException(Exception):
        def __init__(self):
            return

    def __init__(self, right_left, motion):
        """
        @brief standard constructor
        Initialise member variables here
        """
        self._right_left = right_left
        if not( right_left == 'right' or right_left == 'left' ):
            raise InvalidArgException()
        self._motion = motion
        if self._right_left == 'right':
            self._tags = pk.right_arm_tags
            self._handTag = 'RHand'
        else:
            self._tags = pk.left_arm_tags
            self._handTag = 'LHand'
            
        pass

    # RETURN_ID closeGripper()
    def closeGripper(self):
        # raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        self._motion._ptr().closeHand(self._handTag)
        result = JARA_ARM.RETURN_ID(JARA_ARM.OK, "")
        return result
        
        # *** Implement me
        # Must return: result

    # RETURN_ID getBaseOffset(out HgMatrix offset)
    def getBaseOffset(self):
        raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result, offset

    # RETURN_ID getFeedbackPosCartesian(out CarPosWithElbow pos)
    def getFeedbackPosCartesian(self):
        #raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        self._angles = self._motion._ptr().getAngles(ssr.StringArray(self._tags), True)
        angle_sets = self._angles.data
        matrix = fk.calc_fk_and_jacob_matrix(angle_sets, False, self._right_left == 'right')
        print matrix
        pos = JARA_ARM.CarPosWithElbow([[matrix[0,0],matrix[0,1],matrix[0,2],matrix[0,3]], [matrix[1,0],matrix[1,1],matrix[1,2],matrix[1,3]], [matrix[2,0],matrix[2,1],matrix[2,2],matrix[2,3]]], 0.0, 0)
        result = JARA_ARM.RETURN_ID(JARA_ARM.OK, "")
        return result, pos
        # *** Implement me
        # Must return: result, pos

    # RETURN_ID getMaxSpeedCartesian(out CartesianSpeed speed)
    def getMaxSpeedCartesian(self):
        raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result, speed

    # RETURN_ID getMaxSpeedJoint(out DoubleSeq speed)
    def getMaxSpeedJoint(self):
        raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result, speed

    # RETURN_ID getMinAccelTimeCartesian(out double aclTime)
    def getMinAccelTimeCartesian(self):
        raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result, aclTime

    # RETURN_ID getMinAccelTimeJoint(out double aclTime)
    def getMinAccelTimeJoint(self):
        raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result, aclTime

    # RETURN_ID getSoftLimitCartesian(out LimitValue xLimit, out LimitValue yLimit, out LimitValue zLimit)
    def getSoftLimitCartesian(self):
        raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result, xLimit, yLimit, zLimit

    # RETURN_ID moveGripper(in ULONG angleRatio)
    def moveGripper(self, angleRatio):
        raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result

    # RETURN_ID moveLinearCartesianAbs(in CarPosWithElbow carPoint)
    def moveLinearCartesianAbs(self, carPoint):
        raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result

    # RETURN_ID moveLinearCartesianRel(in CarPosWithElbow carPoint)
    def moveLinearCartesianRel(self, carPoint):
        raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result

    # RETURN_ID movePTPCartesianAbs(in CarPosWithElbow carPoint)
    def movePTPCartesianAbs(self, carPoint):
        #self._angles = self._motion._ptr().getAngles(ssr.StringArray(self._tags), True)
        #angle_sets = self._angles.data
        #pos, ori = fk.calc_fk_and_jacob(angle_sets, False, self._right_left == 'right')
        target_pos = np.array([carPoint.carPos[0][3], carPoint.carPos[1][3], carPoint.carPos[2][3], 1])
        target_ori = None
        epsilon = 0.1
        target_angles = ik.calc_inv_pos(angle_sets, target_pos, target_ori, epsilon, right=self._right_left == 'right')
        angles = [a for a in target_angles]
        self._motion._ptr().setAngles(ssr.StringArray(self._tags), ssr.FloatArray(angles), 1.0)
        result = JARA_ARM.RETURN_ID(JARA_ARM.OK, "")
        return result
        #raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result

    # RETURN_ID movePTPCartesianRel(in CarPosWithElbow carPoint)
    def movePTPCartesianRel(self, carPoint):
        self._angles = self._motion._ptr().getAngles(ssr.StringArray(self._tags), True)
        angle_sets = self._angles.data
        pos, ori = fk.calc_fk_and_jacob(angle_sets, False, self._right_left == 'right')
        target_pos = np.array([pos[0] + carPoint.carPos[0][3], pos[1] + carPoint.carPos[1][3], pos[2] + carPoint.carPos[2][3], 1])
        target_ori = None
        epsilon = 0.1
        target_angles = ik.calc_inv_pos(angle_sets, target_pos, target_ori, epsilon, right=self._right_left == 'right')
        angles = [a for a in target_angles]
        self._motion._ptr().setAngles(ssr.StringArray(self._tags), ssr.FloatArray(angles), 1.0)
        result = JARA_ARM.RETURN_ID(JARA_ARM.OK, "")
        return result
        #raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result

    # RETURN_ID movePTPJointAbs(in JointPos jointPoints)
    def movePTPJointAbs(self, jointPoints):
        self._motion._ptr().setAngles(ssr.StringArray(self._tags), ssr.FloatArray(jointPoints.pos), 1.0)
        result = JARA_ARM.RETURN_ID(JARA_ARM.OK, "")
        return result
        #raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result

    # RETURN_ID movePTPJointRel(in JointPos jointPoints)
    def movePTPJointRel(self, jointPoints):
        self._angles = self._motion._ptr().getAngles(ssr.StringArray(self._tags), True)
        pos_ = [0] * 5
        for i in range(0, 5):
            pos_[i] = self._angles[i] + jointPoints.pos[i]
        self._motion._ptr().setAngles(ssr.StringArray(self._tags), ssr.FloatArray(pos_), 1.0)
        result = JARA_ARM.RETURN_ID(JARA_ARM.OK, "")
        return result
        
        #raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result

    # RETURN_ID openGripper()
    def openGripper(self):
        self._motion._ptr().openHand(self._handTag)
        result = JARA_ARM.RETURN_ID(JARA_ARM.OK, "")
        return result

        #raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result

    # RETURN_ID pause()
    def pause(self):
        raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result

    # RETURN_ID resume()
    def resume(self):
        raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result

    # RETURN_ID stop()
    def stop(self):
        raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result

    # RETURN_ID setAccelTimeCartesian(in double aclTime)
    def setAccelTimeCartesian(self, aclTime):
        raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result

    # RETURN_ID setAccelTimeJoint(in double aclTime)
    def setAccelTimeJoint(self, aclTime):
        raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result

    # RETURN_ID setBaseOffset(in HgMatrix offset)
    def setBaseOffset(self, offset):
        raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result

    # RETURN_ID setControlPointOffset(in HgMatrix offset)
    def setControlPointOffset(self, offset):
        raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result

    # RETURN_ID setMaxSpeedCartesian(in CartesianSpeed speed)
    def setMaxSpeedCartesian(self, speed):
        raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result

    # RETURN_ID setMaxSpeedJoint(in DoubleSeq speed)
    def setMaxSpeedJoint(self, speed):
        raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result

    # RETURN_ID setMinAccelTimeCartesian(in double aclTime)
    def setMinAccelTimeCartesian(self, aclTime):
        raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result

    # RETURN_ID setMinAccelTimeJoint(in double aclTime)
    def setMinAccelTimeJoint(self, aclTime):
        raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result

    # RETURN_ID setSoftLimitCartesian(in LimitValue xLimit, in LimitValue yLimit, in LimitValue zLimit)
    def setSoftLimitCartesian(self, xLimit, yLimit, zLimit):
        raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result

    # RETURN_ID setSpeedCartesian(in ULONG spdRatio)
    def setSpeedCartesian(self, spdRatio):
        raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result

    # RETURN_ID setSpeedJoint(in ULONG spdRatio)
    def setSpeedJoint(self, spdRatio):
        raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result

    # RETURN_ID moveCircularCartesianAbs(in CarPosWithElbow carPointR, in CarPosWithElbow carPointT)
    def moveCircularCartesianAbs(self, carPointR, carPointT):
        raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result

    # RETURN_ID moveCircularCartesianRel(in CarPosWithElbow carPointR, in CarPosWithElbow carPointT)
    def moveCircularCartesianRel(self, carPointR, carPointT):
        raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result

    # RETURN_ID setHome(in JointPos jointPoint)
    def setHome(self, jointPoint):
        raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result

    # RETURN_ID getHome(out JointPos jointPoint)
    def getHome(self):
        raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result, jointPoint

    # RETURN_ID goHome()
    def goHome(self):
        raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result


if __name__ == "__main__":
    import sys
    
    # Initialise the ORB
    orb = CORBA.ORB_init(sys.argv)
    
    # As an example, we activate an object in the Root POA
    poa = orb.resolve_initial_references("RootPOA")

    # Create an instance of a servant class
    servant = ManipulatorCommonInterface_Middle_i()

    # Activate it in the Root POA
    poa.activate_object(servant)

    # Get the object reference to the object
    objref = servant._this()
    
    # Print a stringified IOR for it
    print orb.object_to_string(objref)

    # Activate the Root POA's manager
    poa._get_the_POAManager().activate()

    # Run the ORB, blocking this thread
    orb.run()

