#!/usr/bin/env python

import rospy
import random

from python_qt_binding.QtCore import pyqtSlot
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtCore import Signal
from python_qt_binding.QtGui import QFont
from python_qt_binding.QtWidgets import QApplication
from python_qt_binding.QtWidgets import QHBoxLayout
from python_qt_binding.QtWidgets import QLabel
from python_qt_binding.QtWidgets import QLineEdit
from python_qt_binding.QtWidgets import QPushButton
from python_qt_binding.QtWidgets import QSlider
from python_qt_binding.QtWidgets import QVBoxLayout
from python_qt_binding.QtWidgets import QGridLayout
from python_qt_binding.QtWidgets import QScrollArea
from python_qt_binding.QtWidgets import QSpinBox
from python_qt_binding.QtWidgets import QWidget

import xml.dom.minidom
from sensor_msgs.msg import JointState
from math import pi
from threading import Thread
import sys
import signal
import math

from naoqi import ALProxy
from naoqi import ALBroker
from naoqi import ALModule
from std_msgs.msg import String, Float64MultiArray
#from nao_pkg.msg import Float64Array, HeadPitch
from sensor_msgs.msg import JointState
from functools import partial
import numpy as np
from optparse import OptionParser
NAO_IP = "nao.local"
from math import pi
# Global variable to store the HumanGreeter module instance
memory = None
import json
import argparse
import motion
import almath

RANGE = 10000


def get_param(name, value=None):
    private = "~%s" % name
    if rospy.has_param(private):
        return rospy.get_param(private)
    elif rospy.has_param(name):
        return rospy.get_param(name)
    else:
        return value


class naoTalker():
    def init_collada(self, robot):
        robot = robot.getElementsByTagName('kinematics_model')[0].getElementsByTagName('technique_common')[0]
        for child in robot.childNodes:
            if child.nodeType is child.TEXT_NODE:
                continue
            if child.localName == 'joint':
                name = child.getAttribute('name')
                if child.getElementsByTagName('revolute'):
                    joint = child.getElementsByTagName('revolute')[0]
                else:
                    rospy.logwarn("Unknown joint type %s", child)
                    continue

                if joint:
                    limit = joint.getElementsByTagName('limits')[0]
                    minval = float(limit.getElementsByTagName('min')[0].childNodes[0].nodeValue)
                    maxval = float(limit.getElementsByTagName('max')[0].childNodes[0].nodeValue)
                if minval == maxval:  # this is fixed joint
                    continue

                self.joint_list.append(name)
                joint = {'min':minval*pi/180.0, 'max':maxval*pi/180.0, 'zero':0, 'position':0, 'velocity':0, 'effort':0}
                self.free_joints[name] = joint

    def init_urdf(self, robot):
        robot = robot.getElementsByTagName('robot')[0]
        # Find all non-fixed joints
        for child in robot.childNodes:
            if child.nodeType is child.TEXT_NODE:
                continue
            if child.localName == 'joint':
                jtype = child.getAttribute('type')
                if jtype == 'fixed' or jtype == 'floating':
                    continue
                name = child.getAttribute('name')
                #limits = self.motion.getLimits(name)
                self.joint_list.append(name)
                if jtype == 'continuous':
                    minval = -pi
                    maxval = pi
                else:
                    
                    try:
                        #questa parte puo essere sostituita con la funzione 
                        limit = child.getElementsByTagName('limit')[0]
                        minval = float(limit.getAttribute('lower'))
                        maxval = float(limit.getAttribute('upper'))
                        #minval=limits[0]
                        #maxval=limits[1]
                    except:
                        rospy.logwarn("%s is not fixed, nor continuous, but limits are not specified!" % name)
                        continue
                """
                safety_tags = child.getElementsByTagName('safety_controller')
                if self.use_small and len(safety_tags) == 1:
                    tag = safety_tags[0]
                    if tag.hasAttribute('soft_lower_limit'):
                        minval = max(minval, float(tag.getAttribute('soft_lower_limit')))
                    if tag.hasAttribute('soft_upper_limit'):
                        maxval = min(maxval, float(tag.getAttribute('soft_upper_limit')))
                """
                mimic_tags = child.getElementsByTagName('mimic')
                if self.use_mimic and len(mimic_tags) == 1:
                    tag = mimic_tags[0]
                    entry = {'parent': tag.getAttribute('joint')}
                    if tag.hasAttribute('multiplier'):
                        entry['factor'] = float(tag.getAttribute('multiplier'))
                    if tag.hasAttribute('offset'):
                        entry['offset'] = float(tag.getAttribute('offset'))

                    self.dependent_joints[name] = entry
                    continue

                if name in self.dependent_joints:
                    continue

                if self.zeros and name in self.zeros:
                    zeroval = self.zeros[name]
                elif minval > 0 or maxval < 0:
                    zeroval = (maxval + minval)/2
                else:
                    zeroval = 0

                joint = {'min': minval, 'max': maxval, 'zero': zeroval}
                if self.pub_def_positions:
                    joint['position'] = zeroval
                if self.pub_def_vels:
                    joint['velocity'] = 0.0
                if self.pub_def_efforts:
                    joint['effort'] = 0.0

                if jtype == 'continuous':
                    joint['continuous'] = True
                self.free_joints[name] = joint

    def __init__(self):
        description = get_param('robot_description')
        
        self.free_joints = {}
        self.joint_list = [] # for maintaining the original order of the joints
        self.dependent_joints = get_param("dependent_joints", {})
        self.use_mimic = get_param('use_mimic_tags', True)
        self.use_small = get_param('use_smallest_joint_limits', True)

        self.zeros = get_param("zeros")

        self.pub_def_positions = get_param("publish_default_positions", True)
        self.pub_def_vels = get_param("publish_default_velocities", False)
        self.pub_def_efforts = get_param("publish_default_efforts", False)

        self.connectNaoQi()
        self.initModules()

        robot = xml.dom.minidom.parseString(description)
        if robot.getElementsByTagName('COLLADA'):
            self.init_collada(robot)
        else:
            self.init_urdf(robot)

        """
        use_gui = get_param("use_gui", False)

        if use_gui:
            num_rows = get_param("num_rows", 0)
            self.app = QApplication(sys.argv)
            self.gui = JointStatePublisherGui("Joint State Publisher", self, num_rows)
            self.gui.show()
        else:
            self.gui = None
        """
        

        
        self.pub = rospy.Publisher('joint_states', JointState, queue_size=5)


    def source_cb(self, msg):
        for i in range(len(msg.name)):
            name = msg.name[i]
            if name not in self.free_joints:
                continue

            if msg.position:
                position = msg.position[i]
            else:
                position = None
            if msg.velocity:
                velocity = msg.velocity[i]
            else:
                velocity = None
            if msg.effort:
                effort = msg.effort[i]
            else:
                effort = None

            joint = self.free_joints[name]
            if position is not None:
                joint['position'] = position
            if velocity is not None:
                joint['velocity'] = velocity
            if effort is not None:
                joint['effort'] = effort

        if self.gui is not None:
            # signal instead of directly calling the update_sliders method, to switch to the QThread
            self.gui.sliderUpdateTrigger.emit()

    def createJson(self, data,nome):
        output_data={}
        jointName = self.motion.getBodyNames("Body")
        
        #for i in range(len(data)):
            #output_data[jointName[i]] = data[i]
        
        with open('/home/giovanna/catkin_ws/src/nao_pkg/JSON/'+nome+'.txt', 'w') as outfile:
            json.dump(data , outfile)


    def loop(self):
        hz = get_param("rate", 10)  # 10hz
        r = rospy.Rate(hz)

        delta = get_param("delta", 0.0)

        nomiGiunti = self.motion.getBodyNames("Body")
        nomiGiuntiExtra = ['RFinger13', 'RFinger12', 'LFinger21', 'LFinger13','LFinger11', 
                    'RFinger22', 'LFinger22', 'RFinger21', 'LFinger12', 'RFinger23', 'RFinger11', 
                    'LFinger23', 'LThumb1', 'RThumb1', 'RThumb2', 'LThumb2']
        nomiGiuntiCompleti = nomiGiunti + nomiGiuntiExtra

        statoGiunti = self.motion.getAngles("Body", False)
        statoGiuntiExtra = len(nomiGiuntiExtra) * [0.0]
        statoGiuntiCompleti = statoGiunti + statoGiuntiExtra
        
        self.reorder(nomiGiuntiCompleti, statoGiuntiCompleti)
        statoPrecedenteGiuntiCompleti = statoGiuntiCompleti
        
        # Publish Joint States
        while not rospy.is_shutdown():
            
            statoGiunti = self.motion.getAngles("Body", False)
            statoGiuntiExtra = len(nomiGiuntiExtra) * [0.0]
            statoGiuntiCompleti = statoGiunti + statoGiuntiExtra
            statoGiuntiCompleti = self.reorder(nomiGiuntiCompleti, statoGiuntiCompleti)
            #statoPrecedenteGiuntiCompleti = statoGiuntiCompleti
            
            if self.compaire(statoGiuntiCompleti, statoPrecedenteGiuntiCompleti):

                msg = JointState()
                msg.header.stamp = rospy.Time.now()

                self.createJson(self.dependent_joints, "dependJoints")
                self.createJson(self.free_joints, "freeJoints")
                self.createJson(self.joint_list, "jointList")
        
                #statoGiunti = self.motion.getAngles("Body", False)
                #statoGiuntiExtra = len(nomiGiuntiExtra) * [0.0]
                #statoGiuntiCompleti = statoGiunti + statoGiuntiExtra
                #giunti = self.reorder(nomiGiuntiCompleti, statoGiuntiCompleti)

                statoPrecedenteGiuntiCompleti = statoGiuntiCompleti
                #positionForUnity = self.reorderForUnity(self.joint_list, statoGiuntiCompleti)
                msg.name = self.reorderForUnity(self.joint_list) #self.joint_list
                msg.position = self.reorderForUnity(statoGiuntiCompleti) #statoGiuntiCompleti
                msg.velocity = len(self.joint_list) * [0.0]
                msg.effort = len(self.joint_list) * [0.0]

                self.pub.publish(msg)

                """
                # Initialize to 0 msg.position, msg.velocity, and msg.effort.
                has_position = len(self.dependent_joints.items()) > 0
                has_velocity = False
                has_effort = False
                for name, joint in self.free_joints.items():
                    if not has_position and 'position' in joint:
                        has_position = True
                    if not has_velocity and 'velocity' in joint:
                        has_velocity = True
                    if not has_effort and 'effort' in joint:
                        has_effort = True
                num_joints = (len(self.free_joints.items()) +
                              len(self.dependent_joints.items()))
                if has_position:
                    msg.position = num_joints * [0.0]
                if has_velocity:
                    msg.velocity = num_joints * [0.0]
                if has_effort:
                    msg.effort = num_joints * [0.0]

                # ---
                """
        
            

                """
                for i, name in enumerate(self.joint_list):
                    msg.name.append(str(name))
                    joint = None

                    # Add Free Joint
                    if name in self.free_joints:
                        joint = self.free_joints[name]
                        varQualcosa = self.motion.getAngles(self.joint_list[i],False)
                        self.createJson(varQualcosa, "ok")
                        #joint['position']=self.motion.getAngles(joint[name],False)
                        factor = 1
                        offset = 0
                    # Add Dependent Joint
                    elif name in self.dependent_joints:
                        param = self.dependent_joints[name]
                        parent = param['parent']
                        factor = param.get('factor', 1)
                        offset = param.get('offset', 0)
                        # Handle recursive mimic chain
                        recursive_mimic_chain_joints = [name]
                        while parent in self.dependent_joints:
                            if parent in recursive_mimic_chain_joints:
                                error_message = "Found an infinite recursive mimic chain"
                                rospy.logerr("%s: [%s, %s]", error_message, ', '.join(recursive_mimic_chain_joints), parent)
                                sys.exit(-1)
                            recursive_mimic_chain_joints.append(parent)
                            param = self.dependent_joints[parent]
                            parent = param['parent']
                            offset += factor * param.get('offset', 0)
                            factor *= param.get('factor', 1)
                        joint = self.free_joints[parent]

                    if has_position and 'position' in joint:
                        msg.position[i] = joint['position'] * factor + offset
                    if has_velocity and 'velocity' in joint:
                        msg.velocity[i] = joint['velocity'] * factor
                    if has_effort and 'effort' in joint:
                        msg.effort[i] = joint['effort']

                if msg.name or msg.position or msg.velocity or msg.effort:
                    # Only publish non-empty messages
                    self.createJson( str(msg), "msg")
                    self.pub.publish(msg)
                """
                try:
                    r.sleep()
                except rospy.exceptions.ROSTimeMovedBackwardsException:
                    pass
    
    def compaire(self, val1, val2):
        different = False
        for i in range(0, len(val1)-1):
            if val1[i] != val2[i]:
                different = True
                return different
        return different


    def update(self, delta):
        for name, joint in self.free_joints.iteritems():
            forward = joint.get('forward', True)
            if forward:
                joint['position'] += delta
                if joint['position'] > joint['max']:
                    if joint.get('continuous', False):
                        joint['position'] = joint['min']
                    else:
                        joint['position'] = joint['max']
                        joint['forward'] = not forward
            else:
                joint['position'] -= delta
                if joint['position'] < joint['min']:
                    joint['position'] = joint['min']
                    joint['forward'] = not forward

    def reorder(self, nomiGiuntiCompleti, statoGiuntiCompleti):
        #riordinare statoGiuntiCompleti, associati a nomiGiuntiCompleti, in base a joint_list
        #ritorna lo stato ordinato e completo
        jointsList = []
        new_position=[]
        for i in range(0, len(self.joint_list)):           
            indice = nomiGiuntiCompleti.index(self.joint_list[i])
            jointsList.append(statoGiuntiCompleti[indice])

        """
        new_position1=[]
        for i in range(0,1):
            new_position1.append(statoGiuntiCompleti[i])

        for i in range(8,19):
            new_position1.append(statoGiuntiCompleti[i])

        for i in range(2,7):
            new_position1.append(statoGiuntiCompleti[i])

        for i in range(20,len(statoGiuntiCompleti)):
            new_position1.append(statoGiuntiCompleti[i])

        self.createJson(jointsList, "new")
        self.createJson(str(self.joint_list+new_position1), "new1")
        """
        return jointsList


    def reorderForUnity(self, val):
        newValues=[]
        for i in range(0,20):
            newValues.append(val[i])
        newValues.append(val[28])
        newValues.append(val[32])
        newValues.append(val[37])
        newValues.append(val[30])
        newValues.append(val[34])
        newValues.append(val[29])
        newValues.append(val[38])
        newValues.append(val[41])
        newValues.append(val[20])
        newValues.append(val[21])
        newValues.append(val[22])
        newValues.append(val[23])
        newValues.append(val[24])
        newValues.append(val[25])
        newValues.append(val[33])
        newValues.append(val[31])
        newValues.append(val[35])
        newValues.append(val[36])
        newValues.append(val[27])
        newValues.append(val[26])
        newValues.append(val[39])
        newValues.append(val[40])
        return newValues

    def initModules(self):
        global memory
        self.motion   = ALProxy("ALMotion")        
        self.posture  = ALProxy("ALRobotPosture")        
        self.tts      = ALProxy("ALTextToSpeech")
        memory = ALProxy("ALMemory")


    def connectNaoQi(self):
        try:
            parser = OptionParser()
            parser.add_option("--pip",
                help="Parent broker port. The IP address or your robot",
                dest="pip")
            parser.add_option("--pport",
                help="Parent broker port. The port NAOqi is listening to",
                dest="pport",
                type="int")
            parser.set_defaults(
                pip="127.0.0.1",
                pport=9559)
            parser.add_option("--robot_description",
                help="Parent broker port. The IP address or your robot",
                dest="name")
            (opts, args_) = parser.parse_args()
            self.pip   = get_param('pip', None)
            self.pport = get_param('pport', None)
            self.description = opts.name
            myBroker = ALBroker("myBroker","0.0.0.0",0,self.pip, self.pport)
        
            
        except KeyboardInterrupt:
            print "Interrupted by user, shutting down"
            myBroker.shutdown()
            sys.exit(0)




if __name__ == '__main__':
    try:
        rospy.init_node('nao_talker', anonymous=True)
        nao = naoTalker()
        nao.loop()
        

    except rospy.ROSInterruptException:
        pass
