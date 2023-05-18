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
        self.angle1 = np.pi / 2  # 90 degrees
        self.angle2 = -np.pi / 2  # -90 degrees
        self.t = 0
        self.dt = 0.01
        self.current_position = 0
        self.direction = 1  # direction of movement: 1 for forward, -1 for backward
        self.path_length = 0.10  # path length in meters
        self.distance_between_fish = 0.08  # distance between fish in meters
        self.speed = 0.04  # speed in meters per second
        self.inside_radius = False
        self.initial_position1 = np.array([0.0, 0.0])
        self.initial_position2 = np.array([0.0, 0.0])

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
    
    def get_stim_type(self):
        # Return a randomly chosen stimulus type in the range of 1 to 4
        return random.randint(1, 4)

    def run_stimuli_trial(self, i, stim_type):
        current_trial = (i - self.noStimDurPreExp) // (self.stimTrialDur + self.interStimDur)
        print("currentStimType: ", stim_type)
        print("currentStimTrial: ", current_trial + 1)

        if self.current_trial != current_trial:
            self.current_trial = current_trial
            self.direction = random.choice([-1, 1])

        if stim_type == 1:
            self.angle1 = self.move_in_constant_speed_circle(pathRadius=0.08, direction=self.direction, center=(0,0), angle=self.angle1, node_name=self._node_name1)
        elif stim_type == 2:
            self.move_back_and_forth()
        elif stim_type == 3:
            centers = [(0.08, 0), (-0.08, 0)]
            self.move_in_circling_paths(pathRadius=0.05, centers= centers, direction=self.direction)
        elif stim_type == 4:
            fishx = self.object_position.x
            fishy = self.object_position.y
            self.stimulate_fish_behavior(fishx, fishy)


    def move_in_constant_speed_circle(self, pathRadius, direction, center, angle, node_name):
        dt = 0.01  # time step
        zHeight = -0.03  # height of the center of the circle above the table
        self.speed = 0.04

        rotSpeed = self.speed / pathRadius * direction  # angular speed in rad/s
        angle += rotSpeed * dt  # Update angle based on angular speed
        osgX = center[0] + pathRadius * np.cos(angle)  # x position of the node
        osgY = center[1] + pathRadius * np.sin(angle)  # y position of the node
        orientation = angle + (np.pi / 2 * direction)  # calculate orientation
        self._osg_model.move_node(node_name, x=osgX, y=osgY, z=zHeight, orientation_z=orientation)
        print("x: ", osgX, "y: ", osgY, "z: ", zHeight, "orientation: ", orientation)
        return angle

    def move_back_and_forth(self):
        zHeight = -0.03
        offset = 0.04  # Offset of 4cm in meters

        # Calculate the x_position considering direction
        x_position = self.current_position + self.direction * self.speed * self.dt

        if x_position > self.path_length:
            # If fish has moved further than path_length, change direction
            self.direction *= -1
            x_position = self.path_length  # The fish should not move further than path_length

        elif x_position < 0:
            # If fish has reached the beginning of the path, change direction
            self.direction *= -1
            x_position = 0  # The fish should not move back beyond the start

        # Update current position for the next move
        self.current_position = x_position

        # Position of the first fish
        osgX1 = x_position
        osgY1 = -offset  # Offset applied here

        # Position of the second fish
        osgX2 = osgX1
        osgY2 = self.distance_between_fish - offset  # Offset applied here

        # Calculate the orientation
        orientation = np.pi / 2 * (1 - self.direction)

        self._osg_model.move_node(self._node_name1, x=osgX1, y=osgY1, z=zHeight, orientation_z=orientation)
        self._osg_model.move_node(self._node_name2, x=osgX2, y=osgY2, z=zHeight, orientation_z=orientation)

        # Increment time
        self.t += self.dt

   
    def move_in_circling_paths(self, pathRadius, centers, direction):
        dt = 0.01
        zHeight = -0.03
        self.speed = 0.04
        rotSpeed = self.speed / pathRadius

        for i, center in enumerate(centers):
            if i == 0:
                self.angle1 = self.move_in_constant_speed_circle(pathRadius, direction, center, self.angle1, self._node_name1)
            else:
                self.angle2 = self.move_in_constant_speed_circle(pathRadius, -direction, center, self.angle2, self._node_name2)


    def stimulate_fish_behavior(self, fishx, fishy):
        dt = 0.01  # time step
        zHeight = -0.03  # height of the center of the circle above the table
        self.speed = 0.04  # speed of virtual fish
        radiusThreshold = 0.04  # 4 cm radius threshold
        circularPathRadius = 0.02  # 2 cm radius for circular path
        straightPathLength = 0.1  # 10 cm straight path
        initialDistance = 0.01  # 1 cm distance from real fish
        angleBetweenPaths = np.radians(60)  # 60 degree angle in radians

        # Calculate distance between real fish and center
        distance = np.sqrt((fishx- 0)**2 + (fishy - 0)**2)

        if distance > radiusThreshold:  # If real fish is outside radius
            # Move virtual fish in a circular path around the center
            self.angle1 = self.move_in_constant_speed_circle(pathRadius= circularPathRadius, direction=1, center=(0,0), angle=self.angle1, node_name=self._node_name1)
            # hide node 2
            self.hide_node(self._node_name2)
            self.inside_radius = False  # set the flag to False
        else:  # If real fish is inside radius
            if not self.inside_radius:  # If it is the first entry
                self.t = 0
                self.direction = 1  # Initial direction
                self.initial_position1 = np.array([fishx+ initialDistance * np.cos(angleBetweenPaths/2), fishy + initialDistance * np.sin(angleBetweenPaths/2)])
                self.initial_position2 = np.array([fishx+ initialDistance * np.cos(-angleBetweenPaths/2), fishy + initialDistance * np.sin(-angleBetweenPaths/2)])
                self.inside_radius = True  # set the flag to True

            if self.t * self.speed > straightPathLength:  # If virtual fish has traveled 10 cm
                self.t = 0
                self.direction *= -1

            # Calculate new positions of the virtual fish
            newPosition1 = self.initial_position1 + np.array([self.t * self.speed * np.cos(angleBetweenPaths/2), self.t * self.speed * np.sin(angleBetweenPaths/2)])
            newPosition2 = self.initial_position2 + np.array([self.t * self.speed * np.cos(-angleBetweenPaths/2), self.t * self.speed * np.sin(-angleBetweenPaths/2)])

            # Move the virtual fish to the new positions
            self._osg_model.move_node(self._node_name1, x=newPosition1[0], y=newPosition1[1], z=zHeight)
            self._osg_model.move_node(self._node_name2, x=newPosition2[0], y=newPosition2[1], z=zHeight)

            # Increment time
            self.t += dt
        

    # this is the main function that is called after the node is constructed. you can do anything
    # you wish in here, but typically this is where you would dynamically change the virtual
    # environment, for example in response to the tracked object position
    def loop(self):

        # initilize iteration & timing
        i = 0
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
        pathRadius = 0 #0.08
        rotSpeed = 0 #0.625 rad/s / 100 fps
        self.angle1 = 0
        self.angle2 = 0
        
        dt = 0.01
        fps = 100
        r = rospy.Rate(fps)
        
        self.noStimDurPreExp = 3* fps
        self.noStimDurPostExp = 3* fps
        self.noStimDurPreExpFRAME = 0
        self.noStimDurPostExpFRAME = 0
        self.allStimComplete = False
       
        self.stimTrialDur = 5*fps
        self.interStimDur = 0.5*fps

        self.stimTrialCount = 20

        self.currentStimType = 0
        self.direction = None
        orientation = 0
        self.current_trial = None

        self.stimTrialComplete = False
        self.stimTrialInititated = False

        self.interStimComplete = True
        self.interStimFRAME = 0
        self.stimTrialFRAME = 0
        rho_fish = 0

        i = 0
        Time0 = time.time()
        current_trial = -1
        stim_type = None


        while not rospy.is_shutdown():
            i += 1
            timing = time.time() - Time0
            fishx = self.object_position.x
            fishy = self.object_position.y
            fishz = self.object_position.z

            if i < self.noStimDurPreExp:
                self.hide_node(self._node_name1)
                self.hide_node(self._node_name2)
                print("noStimDurPreExpFRAME: ", i)
            elif i < (self.noStimDurPreExp + (self.stimTrialCount * (self.stimTrialDur + self.interStimDur))):
                current_trial_new = (i - self.noStimDurPreExp) // (self.stimTrialDur + self.interStimDur)
                if current_trial_new != current_trial:
                    # We are in a new trial, so choose a new stimulus type
                    stim_type = self.get_stim_type()
                    current_trial = current_trial_new

                # Calculate start and end frames for stimulus trial and inter-stimulus state
                stim_start_frame = self.noStimDurPreExp + current_trial * (self.stimTrialDur + self.interStimDur)
                stim_end_frame = stim_start_frame + self.stimTrialDur
                interstim_start_frame = stim_end_frame
                interstim_end_frame = interstim_start_frame + self.interStimDur

                if stim_start_frame <= i < stim_end_frame:
                    assert not (interstim_start_frame <= i < interstim_end_frame), "Inter-stimulus state is running during stimulus trial"
                    self.run_stimuli_trial(i, stim_type)
                elif interstim_start_frame <= i < interstim_end_frame:
                    assert not (stim_start_frame <= i < stim_end_frame), "Stimulus trial is running during inter-stimulus state"
                    self.hide_node(self._node_name1)
                    self.hide_node(self._node_name2)
                    print("interStimFRAME: ", i)
            else:
                self.hide_node(self._node_name1)
                self.hide_node(self._node_name2)
                print("noStimDurPostExpFRAME: ", i)


        # while not rospy.is_shutdown():
        #     i += 1
        #     timing = time.time() - Time0
        #     fishx = self.object_position.x
        #     fishy = self.object_position.y
        #     fishz = self.object_position.z

        #     if i < self.noStimDurPreExp:
        #         self.hide_node(self._node_name1)
        #         self.hide_node(self._node_name2)
        #         print("noStimDurPreExpFRAME: ", i)
        #     elif i < (self.noStimDurPreExp + (self.stimTrialCount * (self.stimTrialDur + self.interStimDur))):
        #         current_trial_new = (i - self.noStimDurPreExp) // (self.stimTrialDur + self.interStimDur)
        #         if current_trial_new != current_trial:
        #             # We are in a new trial, so choose a new stimulus type
        #             stim_type = self.get_stim_type()
        #             current_trial = current_trial_new

        #         if i % (self.stimTrialDur + self.interStimDur) < self.stimTrialDur:
        #             self.run_stimuli_trial(i, stim_type)
        #         else:
        #             self.hide_node(self._node_name1)
        #             self.hide_node(self._node_name2)
        #             print("interStimFRAME: ", i)
        #     else:
        #         self.hide_node(self._node_name1)
        #         self.hide_node(self._node_name2)
        #         print("noStimDurPostExpFRAME: ", i)

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
            self.log.direction = self.direction
            self.log.currentStimType = self.currentStimType
            self.log.current_trial = current_trial
            self.log.angle1 = self.angle1
            self.log.angle2 = self.angle2
            self.log.pathRadius = pathRadius
            self.log.zHeight = zHeight
            self.log.noStimDurPostExpFRAME = self.noStimDurPostExpFRAME
            self.log.noStimDurPostExp = self.noStimDurPostExp
            # self.log.noStimDurPreExpComplete = self.noStimDurPreExpComplete
            self.log.noStimDurPreExpFRAME = self.noStimDurPreExpFRAME
            self.log.stimTrialFRAME = self.stimTrialFRAME
            # self.log.currentStimTrial = self.currentStimTrial
            self.log.stimTrialComplete = self.stimTrialComplete
            self.log.stimTrialInititated = self.stimTrialInititated
            # self.log.interStimInitiated = self.interStimInitiated
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
