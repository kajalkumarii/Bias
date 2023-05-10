# import system libraries
import math
import random
import numpy as np
import pandas as pd
import time

# use special conventions ('roslib.load_manifest(...)'
# to import python ros-packages packages
import roslib
roslib.load_manifest('rospy')
import rospy

# osg_utils contains utility functions for manipulating osg files in the display server
roslib.load_manifest('flyvr')
import flyvr.osg_utils as osg_utils

# to make writing experiements easier we provide serveral libraries, including a common
# base class from which all experiments should derive
roslib.load_manifest('fishvr')
import fishvr.experiment
import fishvr.rosutil


class intrinsicBiasExperiment(fishvr.experiment.Experiment):

    # the base class takes care of a number of things for you invisibly including
    # command line parsing, recording experimental metadata, recording experimental
    # configuration for reproducibility, etc. You must override the following functions
    #
    #  * condition_switched()
    #  * loop()
    #

    def __init__(self, args):
        # first chain up to the base class to allow it to parse the command line arguments
        fishvr.experiment.Experiment.__init__(self, args, state=('osgNodeX1', 'osgNodeY1', 'osgNodeX2','osgNodeX2',\
                                                                    'timing', 'fishx', 'fishy', 'fishz',\
                                                                    'fish_theta', 'zHeight', 'pathRadius',\
                                                                        'currentStim','currentDirection', 'currentStimTrial',\
                                                                        'noStimDurPreExp', 'noStimDurPostExp',\
                                                                        'noStimDurPreExpComplete', 'noStimDurPostExpComplete',\
                                                                            'allStimComplete','stimTrialComplete', 'stimTrialInitiated',\
                                                                                    'interStimComplete', 'interStimInitiated',\
                                                                                        'interStimFRAME', 'stimTrialFRAME','rho_fish',\
                                                                                        'speed', 'rotSpeed', 'angle', 'dt','fishHeading'
                                                                                                        ))

        # state has a very important meaning, it describes things which should be saved in the
        # resulting experimental log csv file. This log file is a magic object to which you assign
        # values

        # initilaise some variables which will be used later to hold confiuguration state
        self._osg_path = ''
        self._node_name1 = ''
        self._node_name2 = ''
        self._node_bowl = ''
        self._anim_name = ''
        self._should_animate = False
        self.counter = 0
        self.positions = []

        # StimulusOSGController makes it easier to control things in osg files
        self._osg_model = osg_utils.StimulusOSGController()

    # this function is called after every condition switch. At this point the self.condition
    # object (dict like) contains all those values specified in the configuration yaml file
    # for the current conditon. If you want to know the current condition name, you can
    # use self.condition.name
    def condition_switched(self):
        # when using a filesystem path, make sure to use expand_ros_path(), this saves you some
        # trouble and lets you use ros-relative paths '($find xxx)'
        path = self.condition['osg_filename']
        self._osg_path = fishvr.rosutil.expand_ros_path(path)
        self._osg_model.load_osg(self._osg_path)

        # take some values you defined in the configuration yaml and store them for later use
        self._anim_name = self.condition['animation_name']
        self._node_name1 = self.condition['node_name1']
        self._node_name2 = self.condition['node_name2']
        self._node_bowl = self.condition['node_bowl']

        self._should_animate = self.condition['should_animate']

    # this function is called when there is no current active object being experiemented on, but
    # the tracking system has detected a new object that might be worthwhile performing a trail on

    # add D's filter/object ID/frame number diff

    def should_lock_on(self, obj):
        # for example only lock onto fish < 15cm (in x,y) from the centre of the arena
        r2 = obj.position.x**2 + obj.position.y**2
        return r2 < (0.15**2)


    def hide_node(self, node_name):
        self._osg_model.move_node(node_name, hidden=True)

    def show_node(self, node_name):
        self._osg_model.move_node(node_name, hidden=False)


    def move_in_circle(self, pathRadius, angle, rotSpeed, dt):
        zHeight = -0.03  # height of the center of the circle above the table
        # pathRadius = 0.08  # radius of the circle the arm will move in
        # rotSpeed = np.float64(0.025 * np.pi)  # speed of rotation
        osgNodeX1 = self.centerX + pathRadius * np.cos(angle)  # x position of the node
        osgNodeY1 = self.centerY + pathRadius * np.sin(angle)  # y position of the node
        angle += rotSpeed * self.currentDirection * dt
        orientation = angle + (np.pi/2 * self.currentDirection)
        self._osg_model.move_node(self._node_name1, x=osgNodeX1, y=osgNodeY1, z=zHeight, orientation_z= orientation)
        self.hide_node(self._node_name2)

    def rho_fish(self, fishx, fishy):
        rho_fish = np.sqrt(fishx**2+fishy**2)
        return rho_fish
    
    def move_in_diverging_path(self, angle, rotSpeed, dt, centerX, centerY):
        if not self.positions:
            return

        fishx, fishy = self.positions[0]

        zHeight = -0.03
        self.currentDirection = self.stimDirections[self.currentStimTrial]

        fishHeading = np.arctan2(fishy, fishx)

        osgNodeX1 = centerX + 0.05 * np.cos(-(np.pi / 2 - fishHeading))
        osgNodeY1 = centerY + 0.05 * np.sin(-(np.pi / 2 - fishHeading))
        osgNodeX2 = centerX + 0.05 * np.cos(-(np.pi / 2 - fishHeading + np.pi / 3))
        osgNodeY2 = centerY + 0.05 * np.sin(-(np.pi / 2 - fishHeading + np.pi / 3))

        self._osg_model.move_node(self._node_name1, x=osgNodeX1, y=osgNodeY1, z=zHeight, orientation_z=fishHeading)
        self._osg_model.move_node(self._node_name2, x=osgNodeX2, y=osgNodeY2, z=zHeight, orientation_z=fishHeading)

        angle += rotSpeed * self.currentDirection * dt

        centerX = fishx
        centerY = fishy
        zHeight = -0.03

        osgNodeX1 = centerX + 0.05 * np.cos(-(np.pi / 2 - angle))
        osgNodeY1 = centerY + 0.05 * np.sin(-(np.pi / 2 - angle))
        osgNodeX2 = centerX + 0.05 * np.cos(-(np.pi / 2 - angle + np.pi / 3))
        osgNodeY2 = centerY + 0.05 * np.sin(-(np.pi / 2 - angle + np.pi / 3))

        self._osg_model.move_node(self._node_name1, x=osgNodeX1, y=osgNodeY1, z=zHeight)
        self._osg_model.move_node(self._node_name2, x=osgNodeX2, y=osgNodeY2, z=zHeight)




    # this is the main function that is called after the node is constructed. you can do anything
    # you wish in here, but typically this is where you would dynamically change the virtual
    # environment, for example in response to the tracked object position
    def loop(self):

        # initilize iteration & timing
        i = 0
        Time0 = time.time()
        fishx=0
        fishy=0
        fishz=0
        osgNodeX1 = 0
        osgNodeY1 = 0
        osgNodeX2 = 0
        osgNodeY2 = 0
        
        self.centerX = 0
        self.centerY = 0
        zHeight = -0.03
        pathRadius = 0
        
        # speed= 0
        # rotSpeed = 0
        rotSpeed = 0 #0.625 rad/s / 100 fps
        angle= 0
        
        dt = 0.01
        # fishHeading = 0
        fps = 100
        r = rospy.Rate(fps)
        
        self.noStimDurPreExp = 2* fps
        self.noStimDurPostExp = 5* fps
        self.noStimDurPreExpComplete = False
        self.noStimDurPostExpComplete = False
        self.noStimDurPreExpFRAME = 0
        self.noStimDurPostExpFRAME = 0
        self.allStimComplete = False
       
        self.stimTrialDur = 6*fps
        self.interStimDur = 3*fps
        total_distance = 0

        self.stimTrialCount = 20
        self.stimTrials = [random.randint(1,3) for i in range(self.stimTrialCount)]
        print("stimTrials: ", self.stimTrials)

        self.currentStimType = 0
        self.direction = 0
        self.currentDirection = 0
        self.stimDirections = [round(random.random(),0)*2-1 for i in range(self.stimTrialCount)]
        print("stimDirections: ", self.stimDirections)
        orientation = 0

        self.currentStim= 0
        self.currentStimTrial = 0
        self.stimTrialComplete = False
        self.stimTrialInititated = False
        self.stimChangeComplete = True

        self.interStimComplete = True
        self.interStimInitiated = False
        self.interStimFRAME = 0
        self.stimTrialFRAME = 0
        rho_fish = 0

        #assert XOR of stim trial complete and inter stim complete is true
        assert self.stimTrialComplete ^ self.interStimComplete

        rho = 0.04, # radius threshold for the fish to be beyond in order to display the virtual fish

        while not rospy.is_shutdown():
            i += 1

            # get timing (in seconds)
            timing = time.time() - Time0

            # get fish position
            fishx = self.object_position.x
            fishy = self.object_position.y
            fishz = self.object_position.z

            if (i < self.noStimDurPreExp):
                # hide the wall
                #This is the code that controls the noStimDurPreExp state
                #The "self.noStimDurPreExpFRAME" is the number of frames to wait in this state
                #This code is called from the "run" function of the "ExperimentRunner" class

                self.hide_node(self._node_name1)
                self.hide_node(self._node_name2)
                self.noStimDurPreExpComplete = False
                self.noStimDurPreExpFRAME = i
                print("noStimDurPreExpFRAME: ", self.noStimDurPreExpFRAME)
            else:
                self.noStimDurPreExpComplete = True


            if self.noStimDurPreExpComplete:

                # check if stimtrial is complete, if true, initiate interstim
                if self.stimTrialComplete:
                    # If the stim trial is complete, initiate the interStim period
                    if not self.interStimInitiated:
                        print("interStimInitiated")
                        self.interStimInitiated = True
                        self.interStimComplete = False
                        self.hide_node(self._node_name1)
                        self.hide_node(self._node_name2)
                        self.interStimFRAME = 0
                    else:
                        self.interStimFRAME += 1
                        print("interStimFRAME: ", self.interStimFRAME)

                    if self.interStimFRAME > self.interStimDur:
                        self.interStimComplete = True
                        self.interStimInitiated = False
                        self.stimTrialComplete = False
                        self.stimChangeComplete = True
                        print("interStimComplete")

                        # check if interstrial is complete, if true, initiate stim

                if self.interStimComplete:

                    if not self.stimTrialInititated:
                        print("stimTrialInititated")
                        self.stimTrialInititated = True
                        self.stimTrialComplete = False
                        self.hide_node(self._node_name1)
                        self.hide_node(self._node_name2)
                        self.stimTrialFRAME = 0
                        self.currentStim = self.stimTrials[self.currentStimTrial]
                    # else:    
                        self.currentStimTrial += 1
                    #     # self.currentStim = self.stimTrials[self.currentStimTrial]

            #         # RUN stim trial by changing the position of the virtual fish
                    if self.currentStimTrial < self.stimTrialCount:
                        self.currentDirection = self.stimDirections[self.currentStimTrial]
                        print("currentDirection: ", self.currentDirection)

                        if self.currentStim == 1:
                            self.currentStim = 1
                            print("currentStim: single VF", self.currentStim)
                            self.move_in_circle(0.01,angle,rotSpeed, dt=0.01)
                            self.stimTrialFRAME += 1

                        elif self.currentStim == 2:
                            self.currentStim = 2
                            rho_fish = self.rho_fish(fishx, fishy)

                            if rho_fish > rho:
                                print("fish is beyond radius", rho_fish)
                                self.move_in_circle(0.01,angle,rotSpeed, dt=0.01)

                                self.stimTrialFRAME += 1
                            else:
                                print("fish is within radius", rho_fish)
                                print("currentStim: parallel pair VF", self.currentStim)

                                self.move_in_diverging_path(0.01, angle, rotSpeed, dt=0.01, centerX, centerY)
                                self.counter += 1
                                self.positions.append((fishx, fishy))
                                self.stimTrialFRAME += 1


                        elif self.currentStim == 3:
                            print("stim 3")      

                    if self.currentStimTrial > self.stimTrialCount:

                        if (i < self.noStimDurPostExp):
                            print("noStimDurPostExp")
                            self.hide_nodes()
                            self.noStimDurPostExpComplete = False
                        else:
                            self.noStimDurPostExpComplete = True
                        self.noStimDurPostExpFRAME += 1
                            
            
            # save all data
            self.log.realtime = timing
            self.log.osgNodeX1 = osgNodeX1
            self.log.osgNodeY1 = osgNodeY1
            self.log.osgNodeX2 = osgNodeX2
            self.log.osgNodeY2 = osgNodeY2
            self.log.orientation = orientation
            self.log.time_i = i
            self.log.fishx = fishx
            self.log.fishy = fishy
            self.log.fishz = fishz
            self.log.currentDirection = self.currentDirection
            self.log.currentStim = self.currentStim
            self.log.angle = angle
            self.log.zHeight = zHeight
            self.log.noStimDurPostExpComplete = self.noStimDurPostExpComplete
            self.log.noStimDurPostExpFRAME = self.noStimDurPostExpFRAME
            self.log.noStimDurPostExp = self.noStimDurPostExp
            self.log.noStimDurPreExpComplete = self.noStimDurPreExpComplete
            self.log.noStimDurPreExpFRAME = self.noStimDurPreExpFRAME
            self.log.stimTrialFRAME = self.stimTrialFRAME
            self.log.currentStimTrial = self.currentStimTrial
            self.log.stimTrialComplete = self.stimTrialComplete
            self.log.dt = dt
            self.log.rho_fish = rho_fish
            self.log.rotSpeed = rotSpeed
            # self.log.fishHeading = fishHeading

            self.log.update()

            r.sleep()


def main():
    rospy.init_node("experiment")
    parser, args = fishvr.experiment.get_and_parse_commandline()
    node = intrinsicBiasExperiment(args)
    return node.run()

if __name__=='__main__':
    main()

            #             elif self.currentStim == 3:
            #                 self.currentStim = 3
            #                 print("currentStim: looping pair VF-single", self.currentStim)
            #                 self.currentDirection = self.stimDirections[self.currentStimTrial]
            #                 rho_fish = np.sqrt(fishx**2+fishy**2)

            #                 if rho_fish > rho:
            #                     print("fish is beyond radius", rho_fish)
            #                     pathRadius = 0.02
            #                     zHeight = -0.03
            #                     rotSpeed = np.float64(0.025 * np.pi)
            #                     osgNodeX1 = centerX + pathRadius * np.cos(angle)
            #                     osgNodeY1 = centerY + pathRadius * np.sin(angle)
            #                     # osgNodeX1 += centerX + pathRadius * np.cos(angle)
            #                     # osgNodeY1 += centerY + pathRadius * np.sin(angle)

            #                     angle += rotSpeed * self.currentDirection * dt
            #                     orientation = angle + (np.pi/2 * self.currentDirection)
                                
            #                     print("x: ", osgNodeX1, "y: ", osgNodeY1, "z: ", zHeight, "orientation: ", angle)
            #                     self._osg_model.move_node(self._node_name1, x=osgNodeX1, y=osgNodeY1, z=zHeight, orientation_z= orientation)
            #                     self._osg_model.move_node(self._node_name2, hidden=True)

            #                     self.stimTrialFRAME += 1
            #                 else:
            #                     print("fish is within radius", rho_fish)
            #                     print("currentStim: looping pair VF", self.currentStim)
            #                     #  get heading of the fish based on previous 400 frames
            #                     zHeight = -0.03
            #                     self.currentDirection = self.stimDirections[self.currentStimTrial]
            #                     # convert fishx and fishy to numpy arrays
                                
            #                     fishHeading = - np.arctan2(fishy, fishx)

            #                     # project two virtual fish in front of the fish equidistant from the fish at a distance of 0.01 from the fish and parallel to each other and at a distance of 9cm from each other
            #                     osgNodeX1 = fishx + 0.01*np.cos(fishHeading)
            #                     osgNodeY1 = fishy + 0.01*np.sin(fishHeading)
            #                     osgNodeX2 = fishx + 0.01*np.cos(fishHeading) + 0.09*np.cos(fishHeading+90*np.pi/180)
            #                     osgNodeY2 = fishy + 0.01*np.sin(fishHeading) + 0.09*np.sin(fishHeading+90*np.pi/180)

            #                     # # make the virtual fishes move a linearly for 0.05m at a speed of 0.05m/s after their initial position
            #                     # osgNodeX1 += 0.05*np.cos(fishHeading)
            #                     # osgNodeY1 += 0.05*np.sin(fishHeading)
            #                     # osgNodeX2 += 0.05*np.cos(fishHeading) + 0.09*np.cos(fishHeading+90*np.pi/180)
            #                     # osgNodeY2 += 0.05*np.sin(fishHeading) + 0.09*np.sin(fishHeading+90*np.pi/180)

            #                     osgNodeX1 += 0.05*np.cos(fishHeading+90*np.pi/180)
            #                     osgNodeY1 += 0.05*np.sin(fishHeading+90*np.pi/180)
            #                     osgNodeX2 += 0.05*np.cos(fishHeading-90*np.pi/180)
            #                     osgNodeY2 += 0.05*np.sin(fishHeading-90*np.pi/180)

            #                     # now make one virtual fish move 90 degree in counter clockwise direction and the other 90 in clockwise direction and follow a semi-circlular path of radius 0.05m -so a distance of 0.5m pi*pathRadius

            #                     pathRadius = 0.03
            #                     rotSpeed = np.float64(0.025 * np.pi)
            #                     semiCircumference = np.pi*pathRadius
            #                     angle += rotSpeed * self.currentDirection * dt
            #                     orientation = angle + (np.pi/2 * self.currentDirection)

            #                     osgNodeX1 += centerX + pathRadius * np.cos(angle)
            #                     osgNodeY1 += centerY + pathRadius * np.sin(angle)
            #                     osgNodeX2 += centerX + pathRadius * np.cos(angle)
            #                     osgNodeY2 += centerY + pathRadius * np.sin(angle)

            #                     #  and then take a 90 turn again in the same direction
            #                     osgNodeX1 += 0.05*np.cos(fishHeading+90*np.pi/180)
            #                     osgNodeY1 += 0.05*np.sin(fishHeading+90*np.pi/180)
            #                     osgNodeX2 += 0.05*np.cos(fishHeading-90*np.pi/180)
            #                     osgNodeY2 += 0.05*np.sin(fishHeading-90*np.pi/180)

            #                     # continue virtual fish follow with a straight line of 0.05m at 0.05m/s
            #                     osgNodeX1 += 0.05*np.cos(fishHeading)
            #                     osgNodeY1 += 0.05*np.sin(fishHeading)
            #                     osgNodeX2 += 0.05*np.cos(fishHeading) + 0.09*np.cos(fishHeading+90*np.pi/180)
            #                     osgNodeY2 += 0.05*np.sin(fishHeading) + 0.09*np.sin(fishHeading+90*np.pi/180)

            #                     self._osg_model.move_node(self._node_name1, x=osgNodeX1, y=osgNodeY1, z=zHeight, orientation_z=orientation)
            #                     self._osg_model.move_node(self._node_name2, x=osgNodeX2, y=osgNodeY2, z=zHeight, orientation_z= orientation) 

            #                     self.stimTrialFRAME += 1


            #                     if self.stimTrialFRAME > self.stimTrialDur:
            #                         self.stimTrialComplete = True
            #                         self.stimTrialInititated = False
            #                         self.stimChangeComplete = False
            #                         print("stimTrialComplete")