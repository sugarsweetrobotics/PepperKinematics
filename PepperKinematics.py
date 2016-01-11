#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -*- Python -*-

"""
 @file PepperKinematics.py
 @brief Pepper Kinematics RT-component
 @date $Date$


"""
import sys
import time
sys.path.append(".")

# Import RTM module
import RTC
import OpenRTM_aist

import ManipulatorCommonInterface_MiddleLevel_idl
import ManipulatorCommonInterface_Common_idl
import NAO_idl

# Import Service implementation class
# <rtc-template block="service_impl">
from ManipulatorCommonInterface_MiddleLevel_idl_example import *
from ManipulatorCommonInterface_Common_idl_example import *

# </rtc-template>

# Import Service stub modules
# <rtc-template block="consumer_import">
import JARA_ARM, JARA_ARM__POA
import JARA_ARM, JARA_ARM__POA
import ssr, ssr__POA
import ssr, ssr__POA
import ssr, ssr__POA
import ssr, ssr__POA
import ssr, ssr__POA
import ssr, ssr__POA


# </rtc-template>


# This module's spesification
# <rtc-template block="module_spec">
pepperkinematics_spec = ["implementation_id", "PepperKinematics", 
		 "type_name",         "PepperKinematics", 
		 "description",       "Pepper Kinematics RT-component", 
		 "version",           "1.0.0", 
		 "vendor",            "Sugar Sweet Robotics", 
		 "category",          "Manipulato", 
		 "activity_type",     "STATIC", 
		 "max_instance",      "1", 
		 "language",          "Python", 
		 "lang_type",         "SCRIPT",
		 ""]
# </rtc-template>

##
# @class PepperKinematics
# @brief Pepper Kinematics RT-component
# 
# Pepper Manipulator Kinematics RT-component. When use this, install
#	pepper_kinematics module through pip.
#	To install,
#	#pip install pepper_kinematics
# 
# 
class PepperKinematics(OpenRTM_aist.DataFlowComponentBase):
	
	##
	# @brief constructor
	# @param manager Maneger Object
	# 
	def __init__(self, manager):
		OpenRTM_aist.DataFlowComponentBase.__init__(self, manager)


		"""
		"""
		self._rightManipMiddlePort = OpenRTM_aist.CorbaPort("rightManipMiddle")
		"""
		"""
		self._rightManipCommonPort = OpenRTM_aist.CorbaPort("rightManipCommon")
		"""
		"""
		self._leftManipMiddlePort = OpenRTM_aist.CorbaPort("leftManipMiddle")
		"""
		"""
		self._leftManipCommonPort = OpenRTM_aist.CorbaPort("leftManipCommon")
		"""
		"""
		self._motionPort = OpenRTM_aist.CorbaPort("motion")

		"""
		"""
		self._motion = OpenRTM_aist.CorbaConsumer(interfaceType=ssr.ALMotion)

		"""
		"""
		self._rightManipMiddle = ManipulatorCommonInterface_Middle_i('right', self._motion)
		"""
		"""
		self._rightManipCommon = ManipulatorCommonInterface_Common_i('right', self._motion)
		"""
		"""
		self._leftManipMiddle = ManipulatorCommonInterface_Middle_i('left', self._motion)
		"""
		"""
		self._leftManipCommon = ManipulatorCommonInterface_Common_i('left', self._motion)
		


		# initialize of configuration-data.
		# <rtc-template block="init_conf_param">
		
		# </rtc-template>


		 
	##
	#
	# The initialize action (on CREATED->ALIVE transition)
	# formaer rtc_init_entry() 
	# 
	# @return RTC::ReturnCode_t
	# 
	#
	def onInitialize(self):
		# Bind variables and configuration variable
		
		# Set InPort buffers
		
		# Set OutPort buffers
		
		# Set service provider to Ports
		self._rightManipMiddlePort.registerProvider("JARA_ARM_ManipulatorCommonInterface_Middle", "JARA_ARM::ManipulatorCommonInterface_Middle", self._rightManipMiddle)
		self._rightManipCommonPort.registerProvider("JARA_ARM_ManipulatorCommonInterface_Common", "JARA_ARM::ManipulatorCommonInterface_Common", self._rightManipCommon)

		self._leftManipMiddlePort.registerProvider("JARA_ARM_ManipulatorCommonInterface_Middle", "JARA_ARM::ManipulatorCommonInterface_Middle", self._leftManipMiddle)
		self._leftManipCommonPort.registerProvider("JARA_ARM_ManipulatorCommonInterface_Common", "JARA_ARM::ManipulatorCommonInterface_Common", self._leftManipCommon)
		
		# Set service consumers to Ports
		self._motionPort.registerConsumer("ALMotion", "ssr::ALMotion", self._motion)
		
		# Set CORBA Service Ports
		self.addPort(self._rightManipMiddlePort)
		self.addPort(self._rightManipCommonPort)
		self.addPort(self._leftManipMiddlePort)
		self.addPort(self._leftManipCommonPort)
		self.addPort(self._motionPort)
		
		return RTC.RTC_OK
	
	#	##
	#	# 
	#	# The finalize action (on ALIVE->END transition)
	#	# formaer rtc_exiting_entry()
	#	# 
	#	# @return RTC::ReturnCode_t
	#
	#	# 
	#def onFinalize(self, ec_id):
	#
	#	return RTC.RTC_OK
	
	#	##
	#	#
	#	# The startup action when ExecutionContext startup
	#	# former rtc_starting_entry()
	#	# 
	#	# @param ec_id target ExecutionContext Id
	#	#
	#	# @return RTC::ReturnCode_t
	#	#
	#	#
	#def onStartup(self, ec_id):
	#
	#	return RTC.RTC_OK
	
	#	##
	#	#
	#	# The shutdown action when ExecutionContext stop
	#	# former rtc_stopping_entry()
	#	#
	#	# @param ec_id target ExecutionContext Id
	#	#
	#	# @return RTC::ReturnCode_t
	#	#
	#	#
	#def onShutdown(self, ec_id):
	#
	#	return RTC.RTC_OK
	
	#	##
	#	#
	#	# The activated action (Active state entry action)
	#	# former rtc_active_entry()
	#	#
	#	# @param ec_id target ExecutionContext Id
	#	# 
	#	# @return RTC::ReturnCode_t
	#	#
	#	#
	#def onActivated(self, ec_id):
	#
	#	return RTC.RTC_OK
	
	#	##
	#	#
	#	# The deactivated action (Active state exit action)
	#	# former rtc_active_exit()
	#	#
	#	# @param ec_id target ExecutionContext Id
	#	#
	#	# @return RTC::ReturnCode_t
	#	#
	#	#
	#def onDeactivated(self, ec_id):
	#
	#	return RTC.RTC_OK
	
	#	##
	#	#
	#	# The execution action that is invoked periodically
	#	# former rtc_active_do()
	#	#
	#	# @param ec_id target ExecutionContext Id
	#	#
	#	# @return RTC::ReturnCode_t
	#	#
	#	#
	#def onExecute(self, ec_id):
	#
	#	return RTC.RTC_OK
	
	#	##
	#	#
	#	# The aborting action when main logic error occurred.
	#	# former rtc_aborting_entry()
	#	#
	#	# @param ec_id target ExecutionContext Id
	#	#
	#	# @return RTC::ReturnCode_t
	#	#
	#	#
	#def onAborting(self, ec_id):
	#
	#	return RTC.RTC_OK
	
	#	##
	#	#
	#	# The error action in ERROR state
	#	# former rtc_error_do()
	#	#
	#	# @param ec_id target ExecutionContext Id
	#	#
	#	# @return RTC::ReturnCode_t
	#	#
	#	#
	#def onError(self, ec_id):
	#
	#	return RTC.RTC_OK
	
	#	##
	#	#
	#	# The reset action that is invoked resetting
	#	# This is same but different the former rtc_init_entry()
	#	#
	#	# @param ec_id target ExecutionContext Id
	#	#
	#	# @return RTC::ReturnCode_t
	#	#
	#	#
	#def onReset(self, ec_id):
	#
	#	return RTC.RTC_OK
	
	#	##
	#	#
	#	# The state update action that is invoked after onExecute() action
	#	# no corresponding operation exists in OpenRTm-aist-0.2.0
	#	#
	#	# @param ec_id target ExecutionContext Id
	#	#
	#	# @return RTC::ReturnCode_t
	#	#

	#	#
	#def onStateUpdate(self, ec_id):
	#
	#	return RTC.RTC_OK
	
	#	##
	#	#
	#	# The action that is invoked when execution context's rate is changed
	#	# no corresponding operation exists in OpenRTm-aist-0.2.0
	#	#
	#	# @param ec_id target ExecutionContext Id
	#	#
	#	# @return RTC::ReturnCode_t
	#	#
	#	#
	#def onRateChanged(self, ec_id):
	#
	#	return RTC.RTC_OK
	



def PepperKinematicsInit(manager):
    profile = OpenRTM_aist.Properties(defaults_str=pepperkinematics_spec)
    manager.registerFactory(profile,
                            PepperKinematics,
                            OpenRTM_aist.Delete)

def MyModuleInit(manager):
    PepperKinematicsInit(manager)

    # Create a component
    comp = manager.createComponent("PepperKinematics")

def main():
	mgr = OpenRTM_aist.Manager.init(sys.argv)
	mgr.setModuleInitProc(MyModuleInit)
	mgr.activateManager()
	mgr.runManager()

if __name__ == "__main__":
	main()

