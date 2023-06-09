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
        fishvr.experiment.Experiment.__init__(self, args, state=('time_i', 'timing',\
                                                                'osg_x1', 'osg_y1', 'osg_x2','osg_y2',\
                                                                'fishx', 'fishy', 'fishz',\
                                                                'orientation','direction','angle1', 'angle2',\
                                                                'rho_fish','path_radius','z_height',\
                                                                'rot_speed','speed',\
                                                                'stim_start_frame', 'stim_end_frame','stim_trial_frame',\
                                                                'interstim_start_frame', 'interstim_end_frame',\
                                                                'stim_type','stim_flag','current_trial','current_trial_new'\
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
        self.osg_x1 = 0
        self.osg_y1 = 0
        self.osg_x2 = 0
        self.osg_y2 = 0
        self.direction = 1  # direction of movement: 1 for forward, -1 for backward
        # self.path_length = 0.16  # path length in meters
        self.distance_between_fish = 0.08  # distance between fish in meters
        self.initial_offset = -0.08  # fish starts 8 cm away from the center towards one side
        self.final_offset = 0.08  # fish ends 8 cm away from the center towards the other side
        self.current_offset = self.initial_offset  # initialize current position
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
        """
        Check if the object should be locked on based on its position.

        Args:
            obj (object): The object to be checked.

        Returns:
            bool: True if the object should be locked on, False otherwise.
        """
        r2 = obj.position.x**2 + obj.position.y**2
        return r2 < (0.15**2)

    def hide_node(self, node_name):
        """
        Hide the specified node in the OpenSceneGraph model.

        Args:
            node_name (str): Name of the node to be hidden.
        """
        self._osg_model.move_node(node_name, hidden=True)

    def show_node(self, node_name):
        """
        Show the specified node in the OpenSceneGraph model.

        Args:
            node_name (str): Name of the node to be shown.
        """
        self._osg_model.move_node(node_name, hidden=False)

    def get_stim_type(self):
        """
        Return a randomly chosen stimulus type in the range of 1 to 4.

        Returns:
            int: Randomly chosen stimulus type.
        """
        return random.randint(1, 4)

    def run_stimuli_trial(self, i, stim_type):
        """
        Run a stimulus trial.

        Args:
            i (int): Current trial number.
            stim_type (int): Stimulus type for the trial.
        """
        current_trial = (i - self.no_stim_pre_exp_dur) // (self.stim_trial_dur + self.inter_stim_durtim_dur)
        print("currentStimType: ", stim_type)
        print("currentStimTrial: ", current_trial + 1)

        if self.current_trial != current_trial:
            self.current_trial = current_trial
            self.direction = random.choice([-1, 1])

        if stim_type == 1:
            # Stimulus type 1: Move in a constant speed circle around the center of the arena
            initial_position = (0.02, 0)  # Define the initial position for the virtual fish
            self.angle1 = self.move_in_constant_speed_circle(path_radius=0.08, direction=self.direction, center=(0,0), initial_position=initial_position, angle=self.angle1, node_name=self._node_name1)
        elif stim_type == 2:
            # Stimulus type 2: Move back and forth
            self.move_back_and_forth()
        elif stim_type == 3:
            # Stimulus type 3: Move in circling paths around multiple centers
            centers = [(0.08, 0), (-0.08, 0)]
            self.move_in_circling_paths(path_radius=0.06, centers=centers, direction=self.direction)
        elif stim_type == 4:
            # Stimulus type 4: Stimulate fish behavior based on its position
            fishx = self.object_position.x
            fishy = self.object_position.y
            self.stimulate_fish_behavior(fishx, fishy)



    def generate_stimulus_order(self, num_stim_types):
        """
        Generate a random order of stimulus types.

        Args:
            num_stim_types (int): The number of stimulus types.

        Returns:
            list: A list representing the random order of stimulus types.
        """
        stimulus_order = []
        stim_counts = [0] * num_stim_types  # Counter for each stimulus type
        num_trials = self.stim_trial_count

        while len(stimulus_order) < num_trials:
            # Shuffle the stimulus types if all types have been used at least once
            if all(count == 1 for count in stim_counts):
                random.shuffle(stim_counts)

            # Choose a stimulus type randomly until it reaches the desired count
            stim_type = random.randint(1, num_stim_types)
            if stim_counts[stim_type - 1] < (num_trials // num_stim_types):  # Check if the stimulus count is less than the desired count
                stimulus_order.append(stim_type)
                stim_counts[stim_type - 1] += 1
        return stimulus_order


    def move_in_constant_speed_circle(self, path_radius, direction, center, initial_position, angle, node_name):
        """
        Move an object in a constant speed circle.

        Args:
            path_radius (float): Radius of the circular path.
            direction (int): Direction of movement (-1 for clockwise, 1 for counterclockwise).
            center (tuple): Center coordinates of the circular path.
            initial_position (tuple): Initial position of the object.
            angle (float): Current angle of the object on the circular path.
            node_name (str): Name of the node in the model.

        Returns:
            float: Updated angle of the object on the circular path.
        """
        dt = 0.01  # time step
        z_height = -0.03  # height of the center of the circle above the table
        self.speed = 0.04

        rot_speed = self.speed / path_radius * direction  # angular speed in rad/s
        angle += rot_speed * dt  # Update angle based on angular speed
        osg_x1 = center[0] + initial_position[0] + path_radius * np.cos(angle)  # x position of the node
        osg_y1 = center[1] + initial_position[1] + path_radius * np.sin(angle)  # y position of the node
        orientation = angle + (np.pi / 2 * direction)  # calculate orientation
        self._osg_model.move_node(node_name, x=osg_x1, y=osg_y1, z=z_height, orientation_z=orientation)
        print("x: ", osg_x1, "y: ", osg_y1, "z: ", z_height, "orientation: ", orientation)
        self.osg_x1 = osg_x1
        self.osg_y1 = osg_y1
        return angle

    def move_back_and_forth(self):
        """
        Move the virtual fish back and forth between two positions.

        The fish moves in a straight line between two positions with an offset.

        Note: The initial_offset and final_offset values need to be set before calling this method.

        """
        zHeight = -0.03
        offset = 0.04  # Offset of 4cm in meters

        # Calculate the new position considering direction and speed
        current_offset = self.current_offset + self.direction * self.speed * self.dt

        # Implement desired path conditions
        if self.direction < 0 and current_offset <= self.initial_offset:
            self.direction *= -1  # change direction
            current_offset = self.initial_offset  # The fish should not move back beyond the initial_offset
        elif self.direction > 0 and current_offset >= self.final_offset:
            self.direction *= -1  # change direction
            current_offset = self.final_offset  # The fish should not move further than final_offset

        # Update current position for the next move
        self.current_offset = current_offset

        # Position of the first fish
        osg_x1 = current_offset
        osg_y1 = -offset  # Offset applied here

        # Position of the second fish
        osg_x2 = osg_x1
        osg_y2 = self.distance_between_fish - offset  # Offset applied here

        # Calculate the orientation
        orientation = np.pi / 2 * (1 - self.direction)

        self._osg_model.move_node(self._node_name1, x=osg_x1, y=osg_y1, z=zHeight, orientation_z=orientation)
        self._osg_model.move_node(self._node_name2, x=osg_x2, y=osg_y2, z=zHeight, orientation_z=orientation)

        # Increment time
        self.t += self.dt


    def move_in_circling_paths(self, path_radius, centers, direction):
        """
        Move the virtual fish in circling paths around multiple centers.

        Args:
            path_radius (float): Radius of the circular paths.
            centers (list): List of center coordinates for each circular path.
            direction (int): Direction of movement (-1 for clockwise, 1 for counterclockwise).
        """
        dt = 0.01
        z_height = -0.03
        self.speed = 0.04
        rot_speed = self.speed / path_radius

        initial_positions = [(0.02, 0), (0, 0.02)]  # Define the initial positions for the virtual fish

        for i, center in enumerate(centers):
            initial_position = initial_positions[i]  # Get the initial position for the current virtual fish
            if i == 0:
                self.angle1 = self.move_in_constant_speed_circle(path_radius, direction, center, initial_position, self.angle1, self._node_name1)
            else:
                self.angle2 = self.move_in_constant_speed_circle(path_radius, -direction, center, initial_position, self.angle2, self._node_name2)
  


    def stimulate_fish_behavior(self, fishx, fishy):
        """
        Stimulate fish behavior based on the position of the real fish.

        Args:
            fishx (float): X-coordinate of the real fish position.
            fishy (float): Y-coordinate of the real fish position.
        """
        dt = 0.01  # time step
        z_height = -0.03  # height of the center of the circle above the table
        self.speed = 0.04  # speed of virtual fish
        radius_threshold = 0.04  # 4 cm radius threshold
        circularpath_radius = 0.02  # 2 cm radius for circular path
        straight_path_length = 0.1  # 10 cm straight path
        initial_distance = 0.01  # 1 cm distance from real fish
        angle_between_paths = np.radians(60)  # 60 degree angle in radians

        # Calculate distance between real fish and center
        distance = np.sqrt((fishx - 0)**2 + (fishy - 0)**2)

        if distance > radius_threshold:  # If real fish is outside radius
            initial_position = (0.01, 0)  # Define the initial position for the virtual fish
            # Move virtual fish in a circular path around the center
            self.angle1 = self.move_in_constant_speed_circle(path_radius=circularpath_radius, direction=1, center=(0, 0), initial_position=initial_position, angle=self.angle1, node_name=self._node_name1)
            # hide node 2
            self.hide_node(self._node_name2)
            self.inside_radius = False  # set the flag to False
        else:  # If real fish is inside radius
            if not self.inside_radius:  # If it is the first entry
                self.t = 0
                self.direction = 1  # Initial direction
                self.initial_position1 = np.array([fishx + initial_distance * np.cos(angle_between_paths/2), fishy + initial_distance * np.sin(angle_between_paths/2)])
                self.initial_position2 = np.array([fishx + initial_distance * np.cos(-angle_between_paths/2), fishy + initial_distance * np.sin(-angle_between_paths/2)])
                self.inside_radius = True  # set the flag to True

            if self.t * self.speed > straight_path_length:  # If virtual fish has traveled 10 cm
                self.t = 0
                self.direction *= -1

            # Calculate new positions of the virtual fish
            osg_x1 = self.initial_position1 + np.array([self.t * self.speed * np.cos(angle_between_paths/2), self.t * self.speed * np.sin(angle_between_paths/2)])
            osg_x2 = self.initial_position2 + np.array([self.t * self.speed * np.cos(-angle_between_paths/2), self.t * self.speed * np.sin(-angle_between_paths/2)])

            # Calculate the orientation angle based on the direction of motion
            angle1 = np.arctan2(self.t * self.speed * np.sin(angle_between_paths/2), self.t * self.speed * np.cos(angle_between_paths/2))
            angle2 = np.arctan2(self.t * self.speed * np.sin(-angle_between_paths/2), self.t * self.speed * np.cos(-angle_between_paths/2))

            # Move the virtual fish to the new positions with orientation
            self._osg_model.move_node(self._node_name1, x=osg_x1[0], y=osg_x1[1], z=z_height, orientation_z=angle1)
            self._osg_model.move_node(self._node_name2, x=osg_x2[0], y=osg_x2[1], z=z_height, orientation_z=angle2)

            # Increment time
            self.t += dt

            self.osg_x1 = osg_x1[0]
            self.osg_y1 = osg_x1[1]
            self.osg_x2 = osg_x2[0]
            self.osg_y2 = osg_x2[1]


    # this is the main function that is called after the node is constructed. you can do anything
    # you wish in here, but typically this is where you would dynamically change the virtual
    # environment, for example in response to the tracked object position
    def loop(self):
        """
        Main function that is called after the node is constructed.

        This function typically handles the dynamic changes in the virtual environment, such as responding to tracked object positions.
        """
        # initialize iteration & timing
        i = 0

        self.centerX = 0
        self.centerY = 0
        z_height = -0.03
        path_radius = 0  # 0.08
        rot_speed = 0  # 0.625 rad/s / 100 fps
        self.angle1 = 0
        self.angle2 = 0

        dt = 0.01
        fps = 100
        r = rospy.Rate(fps)

        self.no_stim_pre_exp_dur = 10 *60* fps
        self.no_stim_post_exp_dur = 10 *60* fps
        self.no_stim_pre_exp_durFRAME = 0
        self.no_stim_post_exp_durFRAME = 0

        self.stim_trial_dur = 4 * 60 * fps
        self.inter_stim_durtim_dur = 1*60 * fps

        self.stim_trial_count = 16

        self.direction = None
        orientation = 0
        self.current_trial = None

        interstim_end_frame = 0
        interstim_start_frame = 0
        stim_end_frame = 0
        stim_start_frame = 0
        current_trial_new = 0
        current_trial = 0
        stim_type = 0
        stim_flag = 0

        fishx = 0
        fishy = 0
        fishz = 0

        rho_fish = 0

        i = 0
        time0 = time.time()
        current_trial = -1
        stim_type = None

        num_stim_types = 4

        # Generate the random stimulus order
        stimulus_order = self.generate_stimulus_order(num_stim_types)



        while not rospy.is_shutdown():
            i += 1
            timing = time.time() - time0
            fishx = self.object_position.x
            fishy = self.object_position.y
            fishz = self.object_position.z

            if i < self.no_stim_pre_exp_dur:
                # Before the experimental duration, hide the virtual fish nodes
                self.hide_node(self._node_name1)
                self.hide_node(self._node_name2)
                stim_flag = 0  # Set stim_flag as 0
                print("no_stim_pre_exp_durFRAME: ", i)

            elif i < (self.no_stim_pre_exp_dur + (self.stim_trial_count * (self.stim_trial_dur + self.inter_stim_durtim_dur))):
                current_trial_new = int((i - self.no_stim_pre_exp_dur) // (self.stim_trial_dur + self.inter_stim_durtim_dur))
                if current_trial_new != current_trial:
                    # We are in a new trial, so get the stimulus type from the generated order
                    stim_type = stimulus_order[current_trial_new]
                    current_trial = current_trial_new

                # Calculate start and end frames for stimulus trial and inter-stimulus state
                stim_start_frame = self.no_stim_pre_exp_dur + current_trial * (self.stim_trial_dur + self.inter_stim_durtim_dur)
                stim_end_frame = stim_start_frame + self.stim_trial_dur
                interstim_start_frame = stim_end_frame
                interstim_end_frame = interstim_start_frame + self.inter_stim_durtim_dur

                if stim_start_frame <= i < stim_end_frame:
                    assert not (interstim_start_frame <= i < interstim_end_frame), "Inter-stimulus state is running during stimulus trial"
                    # During the stimulus trial, run the specific stimulus type
                    self.run_stimuli_trial(i, stim_type)
                    stim_flag = 1  # Set stim_flag as 1
                elif interstim_start_frame <= i < interstim_end_frame:
                    assert not (stim_start_frame <= i < stim_end_frame), "Stimulus trial is running during inter-stimulus state"
                    # During the inter-stimulus state, hide the virtual fish nodes
                    self.hide_node(self._node_name1)
                    self.hide_node(self._node_name2)
                    stim_flag = 0  # Set stim_flag as 0
                    print("interStimFRAME: ", i)
            else:
                # After the experimental duration, hide the virtual fish nodes
                self.hide_node(self._node_name1)
                self.hide_node(self._node_name2)
                stim_flag = 0  # Set stim_flag as 0
                print("no_stim_post_exp_durFRAME: ", i)


            # save all data
            self.log.time_i = i
            self.log.timing = timing
            self.log.osg_x1 = self.osg_x1
            self.log.osg_y1 = self.osg_y1
            self.log.osg_x2 = self.osg_x2
            self.log.osg_y2 = self.osg_y2
            self.log.orientation = orientation
            self.log.fishx = fishx
            self.log.fishy = fishy
            self.log.fishz = fishz
            self.log.direction = self.direction
            self.log.angle1 = self.angle1
            self.log.angle2 = self.angle2
            self.log.speed = self.speed
            self.log.path_radius = path_radius
            self.log.z_height = z_height
            self.log.stim_start_frame = stim_start_frame
            self.log.stim_end_frame = stim_end_frame
            self.log.stim_trial_frame = i - stim_start_frame
            self.log.interstim_start_frame = interstim_start_frame
            self.log.interstim_end_frame = interstim_end_frame
            self.log.current_trial_new = current_trial_new
            self.log.current_trial = current_trial
            self.log.stim_type = stim_type
            self.log.stim_flag = stim_flag
            self.log.dt = dt
            self.log.rho_fish = rho_fish
            self.log.rot_speed = rot_speed

            self.log.update()

            r.sleep()


def main():
    rospy.init_node("experiment")
    parser, args = fishvr.experiment.get_and_parse_commandline()
    node = intrinsicBiasExperiment(args)
    return node.run()

if __name__=='__main__':
    main()
