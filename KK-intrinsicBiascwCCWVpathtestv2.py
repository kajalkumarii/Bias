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

    # def get_real_fish_position(self):
    #     #  calculate the position of the real fish
    #     #  return the position of the real fish
    #     fishx = self.object_position.x
    #     fishy = self.object_position.y
    #     fishz = self.object_position.z

    #     return (fishx, fishy, fishz)
    
    def get_stim_type(self):
        # Return a randomly chosen stimulus type in the range of 1 to 4
        return random.randint(1, 4)
    
    # def run_stimuli_trial(self, i):
    #     current_trial = (i - self.noStimDurPreExp) // (self.stimTrialDur + self.interStimDur)
    #     stim_type = self.get_stim_type()
    #     print("currentStimType: ", stim_type)
    #     print("currentStimTrial: ", current_trial + 1)

    #     if stim_type == 1:
    #         self.move_in_constant_speed_circle(pathRadius=0.08, angle=0)  # Assumes angle starts at 0
    #     elif stim_type == 2:
    #         self.move_back_and_forth(pathRadius=0.08, distance_between_fish=0.06, speed=0.05, t=0)  # Assume t starts at 0
    #     elif stim_type == 3:
    #         centers = [(-0.08, 0), (0, 0.08)]
    #         self.move_in_circling_paths(pathRadius=0.05, angle=0, centers=centers)  # Assumes angle starts at 0
    #     elif stim_type == 4:
    #         x = self.object_position.x
    #         y = self.object_position.y
    #         z = self.object_position.z
    #         real_fish_position = (x, y, z)
    #         self.move_in_radius_with_real_fish(real_fish_position)

    def run_stimuli_trial(self, i, stim_type):
        current_trial = (i - self.noStimDurPreExp) // (self.stimTrialDur + self.interStimDur)
        print("currentStimType: ", stim_type)
        print("currentStimTrial: ", current_trial + 1)

        if stim_type == 1:
            self.move_in_constant_speed_circle(pathRadius=0.08, angle=0)  # Assumes angle starts at 0
        elif stim_type == 2:
            self.move_back_and_forth(pathRadius=0.08, distance_between_fish=0.06, speed=0.05, t=0)  # Assume t starts at 0
        elif stim_type == 3:
            centers = [(-0.08, 0), (0, 0.08)]
            self.move_in_circling_paths(pathRadius=0.05, angle=[0, 0], centers=centers)  # Assumes angle starts at 0
        elif stim_type == 4:
            x = self.object_position.x
            y = self.object_position.y
            z = self.object_position.z
            real_fish_position = (x, y, z)
            self.move_in_radius_with_real_fish(real_fish_position)


    # Stimuli 1: Virtual fish moving either clockwise or counterclockwise at a distance of 8 cm radius
    def move_in_constant_speed_circle(self, pathRadius, angle, clockwise=None):
        dt = 0.01  # time step
        zHeight = -0.03  # height of the center of the circle above the table
        speed = 0.05  # 5cm/s

        if clockwise is None:
            # Choose a random direction if not provided
            clockwise = random.choice([-1, 1])

        rotSpeed = speed / pathRadius * clockwise  # angular speed in rad/s
        angle += rotSpeed * dt  # Update angle based on angular speed
        osgNodeX1 = self.centerX + pathRadius * np.cos(angle)  # x position of the node
        osgNodeY1 = self.centerY + pathRadius * np.sin(angle)  # y position of the node
        self._osg_model.move_node(self._node_name1, x=osgNodeX1, y=osgNodeY1, z=zHeight)
        print("x: ", osgNodeX1, "y: ", osgNodeY1, "z: ", zHeight)

    # Stimuli 2: Two virtual fish with a distance of 6 cm moving back and forth in a radius of 0.08 m
    def move_back_and_forth(self, pathRadius, distance_between_fish, speed, t):
        zHeight = -0.03
        speed = 0.05 # 5cm/s
        oscillation_frequency = speed / (2 * pathRadius)  # oscillation frequency in Hz
        angular_frequency = 2 * np.pi * oscillation_frequency  # angular frequency in rad/s

        x1 = pathRadius * np.sin(angular_frequency * t)
        y1 = 0
        x2 = x1 + distance_between_fish
        y2 = 0

        self._osg_model.move_node(self._node_name1, x=x1, y=y1, z=zHeight)
        self._osg_model.move_node(self._node_name2, x=x2, y=y2, z=zHeight)

    # Stimuli 3: Two virtual fish circling with a radius of 5cm and center (-0.08, 0) and (0, 0.08) respectively
    # def move_in_circling_paths(self, pathRadius, angle, centers):
    #     dt = 0.01
    #     zHeight = -0.03
    #     speed = 0.05
    #     rotSpeed = speed / pathRadius

    #     for i, center in enumerate(centers):
    #         angle[i] += rotSpeed * dt
    #         osgNodeX = center[0] + pathRadius * np.cos(angle[i])
    #         osgNodeY = center[1] + pathRadius * np.sin(angle[i])
    #         node_name = self._node_name1 if i == 0 else self._node_name2
    #         self._osg_model.move_node(node_name, x=osgNodeX, y=osgNodeY, z=zHeight)

    def move_in_circling_paths(self, pathRadius, angle, centers):
        dt = 0.01
        zHeight = -0.03
        speed = 0.05
        rotSpeed = speed / pathRadius

        for i, center in enumerate(centers):
            if i >= len(angle):  # Check if angle list needs to be expanded
                angle.append(0)  # Initialize angle for the new center
            angle[i] += rotSpeed * dt
            osgNodeX = center[0] + pathRadius * np.cos(angle[i])
            osgNodeY = center[1] + pathRadius * np.sin(angle[i])
            node_name = self._node_name1 if i == 0 else self._node_name2
            self._osg_model.move_node(node_name, x=osgNodeX, y=osgNodeY, z=zHeight)


        # Stimuli 4: Real fish inside radius of 10 cm triggers virtual fish to move in diagonal direction
    def move_in_radius_with_real_fish(self, real_fish_position, radius=0.10, virtual_fish_speed=0.05, duration=10):
        fish_direction = np.arctan2(real_fish_position[1], real_fish_position[0])  # calculate the direction of real fish
        distance_to_real_fish = np.hypot(real_fish_position[0], real_fish_position[1])  # calculate the distance to real fish

        # check if the real fish is within the defined radius
        if distance_to_real_fish <= radius:
            # calculate positions for virtual fish
            virtual_fish_positions = []
            for i in range(2):
                angle = fish_direction + (i * np.pi / 4)  # 1 cm diagonal distance in fish direction
                virtual_fish_positions.append([
                    real_fish_position[0] + np.cos(angle),
                    real_fish_position[1] + np.sin(angle),
                    -0.03
                ])

            # move virtual fish in their diagonal direction at 5cm/s for duration time
            start_time = time.time()
            while (time.time() - start_time) < duration:
                for i in range(2):
                    virtual_fish_positions[i][0] += virtual_fish_speed * np.cos(fish_direction) * (time.time() - start_time)
                    virtual_fish_positions[i][1] += virtual_fish_speed * np.sin(fish_direction) * (time.time() - start_time)
                    node_name = self._node_name1 if i == 0 else self._node_name2
                    self._osg_model.move_node(node_name, x=virtual_fish_positions[i][0], y=virtual_fish_positions[i][1], z=virtual_fish_positions[i][2])
                time.sleep(0.01)
        else:
            print("Real fish is not within the radius of interest.")
    

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
        pathRadius = 0
        rotSpeed = 0 #0.625 rad/s / 100 fps
        angle= 0
        
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
        self.direction = 0
        orientation = 0

        self.currentStimTrial = 0
        self.stimTrialComplete = False
        self.stimTrialInititated = False

        self.interStimComplete = True
        self.interStimFRAME = 0
        self.stimTrialFRAME = 0
        rho_fish = 0

        # i = 0
        # Time0 = time.time()

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
        #         if i % (self.stimTrialDur + self.interStimDur) < self.stimTrialDur:
        #             self.run_stimuli_trial(i)
        #         else:
        #             self.hide_node(self._node_name1)

        #             self.hide_node(self._node_name2)
        #             print("interStimFRAME: ", i)
        #     else:
        #         self.hide_node(self._node_name1)
        #         self.hide_node(self._node_name2)
        #         print("noStimDurPostExpFRAME: ", i)

        #         if i >= (self.noStimDurPreExp + (self.stimTrialCount * (self.stimTrialDur + self.interStimDur)) + self.noStimDurPostExp):
                    # break  # Experiment ends
      

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

                if i % (self.stimTrialDur + self.interStimDur) < self.stimTrialDur:
                    self.run_stimuli_trial(i, stim_type)
                else:
                    self.hide_node(self._node_name1)
                    self.hide_node(self._node_name2)
                    print("interStimFRAME: ", i)
            else:
                self.hide_node(self._node_name1)
                self.hide_node(self._node_name2)
                print("noStimDurPostExpFRAME: ", i)

                if i >= (self.noStimDurPreExp + (self.stimTrialCount * (self.stimTrialDur + self.interStimDur)) + self.noStimDurPostExp):
                    break  # Experiment ends

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
            # self.log.currentDirection = self.currentDirection
            self.log.currentStimType = self.currentStimType
            self.log.angle = angle
            self.log.pathRadius = pathRadius
            self.log.zHeight = zHeight
            self.log.noStimDurPostExpFRAME = self.noStimDurPostExpFRAME
            self.log.noStimDurPostExp = self.noStimDurPostExp
            # self.log.noStimDurPreExpComplete = self.noStimDurPreExpComplete
            self.log.noStimDurPreExpFRAME = self.noStimDurPreExpFRAME
            self.log.stimTrialFRAME = self.stimTrialFRAME
            self.log.currentStimTrial = self.currentStimTrial
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
