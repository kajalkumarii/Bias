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
                                                                        'currentStimType','currentDirection', 'currentStimTrial',\
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


    def move_in_circle(self, pathRadius, angle, rotSpeed):
        zHeight = -0.03  # height of the center of the circle above the table
        # pathRadius = 0.08  # radius of the circle the arm will move in
        # rotSpeed = np.float64(0.025 * np.pi)  # speed of rotation
        dt = 0.01  # time step
        osgNodeX1 = self.centerX + pathRadius * np.cos(angle)  # x position of the node
        osgNodeY1 = self.centerY + pathRadius * np.sin(angle)  # y position of the node
        angle += rotSpeed * self.currentDirection * dt
        orientation = angle + (np.pi/2 * self.currentDirection)
        self._osg_model.move_node(self._node_name1, x=osgNodeX1, y=osgNodeY1, z=zHeight, orientation_z= orientation)
        print("x: ", osgNodeX1, "y: ", osgNodeY1, "z: ", zHeight, "orientation: ", orientation)
        self.hide_node(self._node_name2)

    def rho_fish(self, fishx, fishy):
        rho_fish = np.sqrt(fishx**2+fishy**2)
        return rho_fish
    
    def move_in_diverging_path(self, fishx, fishy):
        if not self.positions:
            self.init_x1 = fishx
            self.init_y1 = fishy
            self.init_x2 = fishx
            self.init_y2 = fishy

        dt = 0.01
        velocity = 0.05  # 5 cm/s
        distance_to_move = 0.5  # 50 cm
        max_steps = int(distance_to_move / (velocity * dt))
        zHeight = -0.03

        if self.counter > max_steps:
            self.hide_node(self._node_name1)
            self.hide_node(self._node_name2)
            return

        fishHeading = np.arctan2(fishy, fishx)

        # Calculate the angle between the real fish and each virtual fish
        angle1 = fishHeading - np.pi / 6
        angle2 = fishHeading + np.pi / 6

        # Calculate the velocity vectors for the virtual fish
        velocity_vector1 = np.array([velocity * np.cos(angle1), velocity * np.sin(angle1)])
        velocity_vector2 = np.array([velocity * np.cos(angle2), velocity * np.sin(angle2)])

        # Update the position of the virtual fish based on their velocity vectors and time step
        new_position1 = np.array([self.init_x1, self.init_y1]) + self.counter * dt * velocity_vector1
        new_position2 = np.array([self.init_x2, self.init_y2]) + self.counter * dt * velocity_vector2

        # Move the virtual fish to their new positions
        self._osg_model.move_node(self._node_name1, x=new_position1[0], y=new_position1[1], z=zHeight, orientation_z=angle1)
        self._osg_model.move_node(self._node_name2, x=new_position2[0], y=new_position2[1], z=zHeight, orientation_z=angle2)

        self.counter += 1


    def move_in_mirrored_d_paths(self, fishx, fishy, fishHeading):
        zHeight = -0.03
        dt = 0.01  # Define dt as a constant value within the function
        linear_speed = 0.05  # 0.05 m/s converted to m/frame (assuming 100 fps)
        pathRadius = 0.05

        if not self.positions:
            self.init_x1 = fishx + 0.01 * np.cos(fishHeading)
            self.init_y1 = fishy + 0.01 * np.sin(fishHeading)
            self.init_x2 = fishx + 0.01 * np.cos(fishHeading) + 0.09 * np.cos(fishHeading + 90 * np.pi / 180)
            self.init_y2 = fishy + 0.01 * np.sin(fishHeading) + 0.09 * np.sin(fishHeading + 90 * np.pi / 180)
            heading = fishHeading
        else:
            heading = fishHeading

        dist_travelled = self.counter * dt * linear_speed

        new_position1 = None
        new_position2 = None

        if dist_travelled < 0.08:
            new_position1 = np.array([self.init_x1, self.init_y1]) + dist_travelled * np.array([np.cos(heading), np.sin(heading)])
            new_position2 = np.array([self.init_x2, self.init_y2]) + dist_travelled * np.array([np.cos(heading), np.sin(heading)])

        elif dist_travelled < 0.08 + np.pi * pathRadius:
            circle_angle = (dist_travelled - 0.08) / pathRadius
            new_position1 = np.array([self.init_x1, self.init_y1]) + np.array([0.08 * np.cos(heading) + pathRadius * (np.cos(heading) - np.cos(heading + circle_angle)),
                                                                                0.08 * np.sin(heading) + pathRadius * (np.sin(heading) - np.sin(heading + circle_angle))])
            new_position2 = np.array([self.init_x2, self.init_y2]) + np.array([0.08 * np.cos(heading) + pathRadius * (np.cos(heading) - np.cos(heading - circle_angle)),
                                                                                0.08 * np.sin(heading) + pathRadius * (np.sin(heading) - np.sin(heading - circle_angle))])

        elif dist_travelled < 0.08 + np.pi * pathRadius + 0.08:
            new_position1 = np.array([self.init_x1, self.init_y1]) + (dist_travelled - np.pi * pathRadius) * np.array([np.cos(heading + np.pi), np.sin(heading + np.pi)])
            new_position2 = np.array([self.init_x2, self.init_y2]) + (dist_travelled - np.pi * pathRadius) * np.array([np.cos(heading + np.pi), np.sin(heading + np.pi)])

        if new_position1 is None or new_position2 is None:
            raise ValueError("new_position1 or new_position2 is not assigned a value")
        
        self._osg_model.move_node(self._node_name1, x=new_position1[0], y=new_position1[1], z=zHeight, orientation_z=heading)
        self._osg_model.move_node(self._node_name2, x=new_position2[0], y=new_position2[1], z=zHeight, orientation_z=heading + np.pi)

        self.positions.append((new_position1, new_position2))
        self.counter += 1

    

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

        self.stimTrialCount = 20
        self.stimTrials = [random.randint(1,3) for i in range(self.stimTrialCount)]
        print("stimTrials: ", self.stimTrials)

        self.currentStimType = 0
        self.direction = 0
        self.currentDirection = 0
        self.stimDirections = [round(random.random(),0)*2-1 for i in range(self.stimTrialCount)]
        print("stimDirections: ", self.stimDirections)
        orientation = 0

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
                        self.currentStimType = self.stimTrials[self.currentStimTrial] 
                        self.currentStimTrial += 1

            #         # RUN stim trial by changing the position of the virtual fish
                    if self.currentStimTrial < self.stimTrialCount:
                        self.currentDirection = self.stimDirections[self.currentStimTrial]
                        print("currentDirection: ", self.currentDirection)

                        if self.currentStimType == 1:
                            self.currentStimType = 1
                            print("currentStimType: single VF", self.currentStimType)
                            rotSpeed=np.float64(0.00625 * np.pi)
                            angle += rotSpeed * self.currentDirection * dt
                            self.move_in_circle(0.08,angle,rotSpeed)
                            self.stimTrialFRAME += 1

                        elif self.currentStimType == 2:
                            self.currentStimType = 2
                            rho_fish = self.rho_fish(fishx, fishy)

                            if rho_fish > rho:
                                print("fish is beyond radius", rho_fish)
                                rotSpeed = np.float64(0.025 * np.pi)
                                angle += rotSpeed * self.currentDirection * dt
                                self.move_in_circle(0.02, angle, rotSpeed)

                                self.stimTrialFRAME += 1
                            else:
                                print("fish is within radius", rho_fish)
                                print("currentStimType: parallel pair VF", self.currentStimType)
                                self.move_in_diverging_path(fishx, fishy)
                                self.positions.append((fishx, fishy))
                                self.stimTrialFRAME += 1


                        elif self.currentStimType == 3:
                            if rho_fish > rho:
                                print("fish is beyond radius", rho_fish)
                                rotSpeed = np.float64(0.025 * np.pi)
                                angle += rotSpeed * self.currentDirection * dt
                                self.move_in_circle(0.02, angle, rotSpeed)

                                self.stimTrialFRAME += 1
                            else:
                                print("fish is within radius", rho_fish)
                                print("currentStimType: MirroredD pair VF", self.currentStimType)
                                fishHeading = -np.arctan2(fishy, fishx)
                                self.move_in_mirrored_d_paths(fishx, fishy, fishHeading)
                                self.positions.append((fishx, fishy))
                                self.stimTrialFRAME += 1


                                    # Check if the current trial is finished
                    if self.stimTrialFRAME > self.stimTrialDur:
                        self.stimTrialComplete = True
                        self.stimTrialInititated = False

                        # If the current trial is complete, increment the currentStimTrial
                        if self.stimTrialComplete:
                            self.currentStimTrial += 1
                            print("currentStimTrial: ", self.currentStimTrial)   

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
            self.log.currentStimType = self.currentStimType
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
