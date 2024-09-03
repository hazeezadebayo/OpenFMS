#!/usr/bin/env python3

import threading
import os, sys, time
import psycopg2, psycopg2.extras
from unittest import TestCase, main

# Define the path to the directory containing 'db_register.py'
agv_config_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../'))
# Add the directory to the Python path
sys.path.append(agv_config_dir)
# Import the class from the module
from viro_simple_fleet.viro_simple_fleet.db_register import DatabaseRegistration



# ----------------------------------------------------------------------- #
#
#        IF, START NAVIGATION:
#                ----L----
#               0-   x   -0
#                |       |
#                |       |
#               0-       -0
#                ---------
#
# ----------------------------------------------------------------------- #
#                    AGV CLIENT DatabaseManager                           #
# ----------------------------------------------------------------------- #


class DatabaseManager:
    """Manages database operations and fleet management tasks."""

    def __init__(self, hostname, database, username, pwd, port, robot_id, fleet_id):

        # Create a lock object
        self.lock = threading.Lock()

        # db_stuff
        self.conn = psycopg2.connect(host=hostname, dbname=database, user=username, password=pwd, port=port)
        self.cur = self.conn.cursor(cursor_factory=psycopg2.extras.DictCursor)

        # Initializing instance variables
        self.robot_id = robot_id
        self.fleet_id = fleet_id

        """Reset instance variables."""
        # helper variables
        self.wait_time_switch = False
        self.max_wait_to_cancel_sec = 100 # seconds
        self.last_state_clearance_nsec = time.time_ns()
        self.last_datetime_info = 0
        self.floor = None

        # input
        self.ros_pose_x, self.ros_pose_y, self.ros_pose_th = None, None, None
        self.ros_system_launch_process = None
        self.ros_dock_status = None
        self.ros_notifications = ['None'] * 3
        self.ros_last_checkpoint = [None]
        self.ros_dummy_checkpoints = [None]
        self.ros_elevator_map_request = None
        self.ros_state_clearance_intervel_sec = 0.5 # seconds
        self.ros_emergency_present = None
        self.ros_battery_percentage = None

        # output
        self.flag = None

        # init DB variables
        self.shutdown = None
        self.current_pose = None
        self.notifications = None
        self.last_checkpoint = None
        self.checkpoints = None
        self.job_nature = None
        self.agv_status = []
        self.agv_itinerary = []
        self.ar_tags = None
        self.datetime_info =None
        self.controls = None
        self.landmarks = None
        self.model_config = None
        self.traffic = None
        self.wait_traffic = None
        self.waitpoints = None
        self.wait_itinerary = []

        self.skip_node = None
        self.wait_start_time = None

        self.db_count = 0


    def db_list_to_pylist(self, dblist):
        """Convert database list string to Python list."""
        result = []
        for sublist in dblist:
            elements = [float(x) for x in sublist.strip("{}").split(",")]
            result.append(elements)
        return result

    # ----------------------------------------------------------------------- #
    #                                 read_db                                 #
    # ----------------------------------------------------------------------- #

    def read_db(self):
        """Read data from the database and update instance variables."""
        try:
            query = "SELECT * FROM table_robot WHERE robot_id = %s;"
            self.cur.execute(query, (self.robot_id,))
            record = self.cur.fetchone()
            # for record in self.cur.fetchall():

            if not record:
                print("No record found for robot_id:", self.robot_id)
                return

            self.shutdown = str(record['shutdown'])
            self.current_pose = str(record['current_pose']).split(',')
            self.notifications = str(record['notifications']).split(',')
            self.last_checkpoint = str(record['last_checkpoint']).split(',')
            self.checkpoints = str(record['checkpoints']).split(',')
            self.waitpoints = str(record['waitpoints']).split(',')
            self.traffic = str(record['traffic'])
            self.wait_traffic = str(record['wait_traffic'])
            self.wait_itinerary = self.db_list_to_pylist(str(record['wait_itinerary']).split("},{"))
            self.agv_status = str(record['agv_status']).split(',')
            fleet_id = str(record['fleet_id'])
            self.agv_itinerary = self.db_list_to_pylist(str(record['agv_itinerary']).split("},{"))
            self.datetime_info = str(record['created_at']).split()
            self.controls = str(record['m_controls']).split(',')
            self.landmarks = str(record['landmark']).split(',')
            self.model_config = str(record['model_config']).split(',')

            if self.fleet_id != fleet_id:
                self.shutdown = "yes"
                print("Fleet ID mismatch. Shutdown command will be sent.")

            self.process_flags_and_notifications()

        except psycopg2.Error as error:
            print("Database error:", error)
            return


    def process_flags_and_notifications(self):
        """Process flags and notifications based on the current state."""

        floor = self.agv_status[5]
        self.flag = self.agv_status[6]

        if (self.notifications[4] == "skip") and (self.skip_node is None):
            self.skip_node = self.ros_last_checkpoint[0]
            self.flag += ',skip'

        elif self.flag == 'green':
            # [next stop occuppied -robot standoff-]:
            if (str(self.notifications[5]) != 'wt') and (str(self.wait_traffic) != 'none') and (self.wait_start_time is None):
                # negotiation! we need to publish [wt,wait_traffic] routes.
                self.wait_start_time = time.time() # Record the start time
                cmd = ','.join([str(self.notifications[5]), str(self.wait_traffic)])
                self.flag += ',' + cmd
            elif (floor != self.floor) and (self.ros_elevator_map_request is not None):
                self.ros_elevator_map_request = None
                self.floor = floor
                self.flag += ',' + self.floor

    # ----------------------------------------------------------------------- #
    #                                update_db                                #
    # ----------------------------------------------------------------------- #

    def update_db(self):
        """Update the database with current state and handle fleet management."""
        with self.lock:
            # if we have a task, and a shutdown command is sent then we want to save the last state by reshuffling the checkpoints.
            # so that the place we wanna visit next starts the stack on our next turn on or active state.
            if (self.ros_dummy_checkpoints == self.checkpoints) and (self.ros_dummy_checkpoints != ['A', 'A', 'A']) and (self.shutdown == 'yes'):
                if (self.last_checkpoint[0] != 'unknown') and (self.landmarks[1] != "clean"):
                    self.reshuffle_checkpoints()

            # Again!! before i get to a specific checkpoint. That is, what happens before it is reserved for me.
            if ((self.ros_system_launch_process is not None) and (self.ros_last_checkpoint[0] != 'unknown')) or \
               ((self.ros_system_launch_process is None) and (self.ros_last_checkpoint[0] == 'manual')):
                self.update_current_pose_and_checkpoints()


    def reshuffle_checkpoints(self):
        """Reshuffle checkpoints when necessary and update the database."""
        # reshuffle checkpoints for restart
        reshuffle_index = len(self.checkpoints) - len(self.last_checkpoint)
        checkpoints_method_1 = self.checkpoints[reshuffle_index:]+self.checkpoints[:reshuffle_index]
        if len(checkpoints_method_1) != 0:
            self.checkpoints = checkpoints_method_1
            self.agv_itinerary = self.agv_itinerary[reshuffle_index:]+self.agv_itinerary[:reshuffle_index]
            # has to go to the DB so that when it doesnt enter here upon shutdown == no, it still exists in DB for continuity.
            try:
                self.cur.execute(
                    'UPDATE table_robot SET checkpoints = %s, agv_itinerary = %s, last_checkpoint = %s WHERE robot_id = %s and fleet_id = %s;',
                    (','.join(self.checkpoints), self.agv_itinerary, 'unknown', self.robot_id, self.fleet_id,))
                self.conn.commit()
            except psycopg2.Error as error: # show the exceptions basically as errors.
                print(error)
            self.ros_dummy_checkpoints = ['A','A','A'] # remove it in shutdown


    def update_current_pose_and_checkpoints(self):
        """Update current pose and checkpoints in the database."""
        try:
            self.cur.execute(
                'UPDATE table_robot SET current_pose = %s, last_checkpoint = %s WHERE robot_id = %s and fleet_id = %s;',
                (f"{self.ros_pose_x:.3f},{self.ros_pose_y:.3f},{self.ros_pose_th:.3f}", ','.join(self.ros_last_checkpoint), self.robot_id, self.fleet_id)
            )
            # print(f"[db_manager]: -ros_last_checkpoint- {self.ros_last_checkpoint}.")

            self.check_battery_and_emergency()

            if self.checkpoints != ['A', 'A', 'A'] and self.ros_last_checkpoint[0] != 'manual' and self.landmarks[1] != "clean":

                next_stop_id = self.ros_last_checkpoint[0]

                self.handle_wait_and_notifications(next_stop_id)

                self.handle_docks_or_loops(next_stop_id)

            self.conn.commit()
        except psycopg2.Error as error:
            print("Database update error:", error)


    def handle_wait_and_notifications(self, next_stop_id):
        """Handle waiting and notifications."""
        # if we skipped, we must take notification back to none,
        # and we are only gonna be sure its left for the new target if,

        if (self.notifications[4] == "skip") and (self.skip_node != next_stop_id) and (self.skip_node is not None):
            self.notifications[4] = "None"
            self.skip_node = None
        elif self.wait_start_time is not None:
            wt = 2 * float(self.notifications[5])
            # wait for some time and then release the wait zone. all things being equal, this robot must have left the wait node at this time.
            elapsed_time = time.time() - self.wait_start_time
            if elapsed_time >= wt:
                self.notifications[5] = "wt"
                self.wait_traffic = 'none'
                self.wait_start_time = None
        # Assuming self.ros_notifications and self.notifications are lists
        elif any(element != 'None' for element in self.ros_notifications):
            for index, element in enumerate(self.ros_notifications):
                if (element != 'None') and (self.notifications[index] != element):
                    # Update self.notifications[index] with the value from self.ros_notifications[index]
                    self.notifications[index] = element
            # 'notice_msg,notice_msg,node_troubleshoot,0,None,x,y',  # error_msg,error_msg,node_troubleshoot,wait_time,post/pre/none,
            self.ros_notifications = ['None'] * 3

        # update notifications!
        self.cur.execute(
            'UPDATE table_robot SET notifications = %s, wait_traffic = %s WHERE robot_id = %s and fleet_id = %s;',
            (','.join(self.notifications), self.wait_traffic, self.robot_id, self.fleet_id,)
            )


    def handle_docks_or_loops(self, next_stop_id):
        """Process and update landmarks related information."""

        # we need to check all the traffic alphabets with our own to be sure that,
        # where we wanna go has not been reserved by another robot.
        # so, obtain robots own currently occupied checkpoint. if any,
        # this robot's current reserved checkpoint is ...
        reserved_checkpoint = self.traffic

        real_landmark = [elements.split('_')[1] for elements in self.landmarks[2:]]

        # else the checkpoint the robot is going to is not being occupied. so make a reservation.
        # That is, robot is free to proceed and register itself in that checkpoint.
        if self.agv_status[0] != 'idle':
            # if a map change was requested by agv, notify the manager.
            # if he consents, green will be published
            if self.ros_elevator_map_request is not None:
                self.agv_status[5] = self.ros_elevator_map_request
            # this implies that, i have now decided to undock and must over-write the location contained in my memory about the dock station i was in.
            # or that its just a normal -not landmark- checkpoint.
            if next_stop_id not in real_landmark:
                self.handle_non_landmark_checkpoint()
            # if it is a landmark, i have already home or dock. so lock the gate.
            # if it is a landmark yeah, we want to show the previous checkpoint as the active state.
            # so that no one blocks it way when it wants to come out/reverse.
            # that is:
            # anytime we dock at a landmark point, we always put our current state as though we were at the gate.
            elif next_stop_id in real_landmark:
                self.handle_landmark_checkpoint(next_stop_id, real_landmark, reserved_checkpoint)


    def handle_non_landmark_checkpoint(self):
        """Handle the scenario where the next stop is not a landmark."""
        self.ros_dock_status = False
        if self.agv_status[1] != 'x':
            # Values to assign
            values = ['x', 'y', 'z', 'w']
            # Update multiple indices at once
            self.agv_status[1:5] = values


    def handle_landmark_checkpoint(self, next_stop_id, real_landmark, reserved_checkpoint):
        """
        When approaching a landmark, if it matches the robot’s target,
        update the docking memory to reflect this.
        Ensure that the robot handles undocking properly before proceeding,
        treating any real landmark as a potential docking station or checkpoint.
        """
        if self.ros_dock_status:
            self.ros_dock_status = False
            # Task Decision: After docking, the robot must decide its next action based on its current task.
            # The key distinction is that:
            # for moving, the robot waits if the target is occupied,
            # while for charging, it seeks a different charging spot if needed.
            if self.landmarks[1] in ["move", "charge", "transport"]:
                if reserved_checkpoint in real_landmark[2:] or self.landmarks[1] in ["move", "charge"]:
                    # if we docked successfuly, we need to make a decision on what to do next,
                    # based on what our current task is:
                    # move or charge is quite similar, its go from one point to another and stay there.
                    # only difference is, if move target is occupied it waits forever,
                    # while if charge target is occupied, it finds another location.
                    self.update_other_task_status(next_stop_id)

            elif self.landmarks[1] == "loop" and self.landmarks[0] == "high":
                # confirm that we are at the pick dock
                # all loop tasks starts with a high priority then transition after first pickup to low priority tasks
                # because they run forever yeah so we cant have every fucking robot on high priority out and about.
                # theres a chance that when the task was assigned we were not at the pickup location
                # so we first plan a path to the pickup location, if where we are now after driving is the first real landmark(pickup loc)
                # inside the real_landmark list then we can delete all the other points that come before this in our checkpoint memory
                # the reason is, if say start pos is X and we go to pick and we now move to drop, since its a cyclic task we do not want
                # to go to X and then pick and then drop and then x and then pick ... you get the gist. we want to remove the checkpoints that
                # essentially took us to our first pickup which represent path from x - pickup thus deleting x and so we are left with
                # pickup ---> drop --> pickup ---> drop ---> pickup blah blah blah.
                # you get the gist
                self.update_loop_task_checkpoints(real_landmark)


    def update_other_task_status(self, next_stop_id):
        """Handle successful docking of the AGV."""
        # use next_stop_id to return the real coordinates as next_stop_coordinate.
        next_stop_coordinate = self.agv_itinerary[self.checkpoints.index(next_stop_id)]
        # AGV Status: The agv_status array tracks the robot's state and docking coordinates.
        #    Index [0] indicates the robot's state (active, inactive, or idle).
        #    Indexes [1] to [4] store the docking coordinates (x, y, z, w).
        self.ros_last_checkpoint = ['unknown']
        self.flag = 'red'
        if self.agv_status[0] == 'active':
            self.agv_status[0] = 'idle'
            self.agv_status[1:5] = map(str, next_stop_coordinate)
            self.agv_status[6] = self.flag
            self.update_agv_status()
            print("[db_manager]: idle updated")


    def update_agv_status(self):
        """Update AGV status in the database."""
        self.cur.execute(
            'UPDATE table_robot SET agv_status = %s WHERE robot_id = %s AND fleet_id = %s;',
            (','.join(self.agv_status), self.robot_id, self.fleet_id))


    def update_loop_task_checkpoints(self, real_landmark):
        """
        Updates the checkpoints for a loop task. When a loop task starts with high priority and transitions to low priority after the first pickup,
        the function ensures that checkpoints are managed correctly. This involves:
        - Removing checkpoints that were part of the initial path leading to the first pickup location.
        - Ensuring that cyclic tasks do not include redundant checkpoints.
        """
        if self.ros_last_checkpoint[0] == real_landmark[0] and len(self.ros_last_checkpoint) > 1:
            while self.checkpoints.index(real_landmark[0]) > 0:
                self.checkpoints.pop(0)
                self.agv_itinerary.pop(0)
            self.landmarks[0] = "low"
            self.cur.execute('UPDATE table_robot SET landmark = %s, checkpoints = %s, agv_itinerary = %s WHERE robot_id = %s and fleet_id = %s;',
                            (','.join(self.landmarks), ','.join(self.checkpoints), self.agv_itinerary, self.robot_id, self.fleet_id,))
        self.ros_dummy_checkpoints = self.checkpoints


    def _update_battery_status(self):
        # Split the model_config string into a list
        # config_list = self.model_config.split(',')
        # Update the battery status
        self.model_config[4] = str(self.ros_battery_percentage)
        # Join the list back into a string
        self.model_config = ','.join(self.model_config)
        # Update the database
        self.cur.execute(
            'UPDATE table_robot SET model_config = %s WHERE robot_id = %s and fleet_id = %s;',
            (self.model_config, self.robot_id, self.fleet_id,))


    def _handle_emergency(self):
        if self.ros_emergency_present is True:
            # if it was idle, and emergency button is pushed,
            # set to inactive so robot does not get suggested for tasks.
            # if it was active or on a task currently, set to inactive too.
            if self.agv_status[0] in ['active', 'idle']:
                self.agv_status[0] = 'inactive'
                self.update_agv_status()
        # i imagine this "ros_emergency_present" is always false.
        elif self.ros_emergency_present is False:
            # if the robot was idle before, then...
            if self.agv_status[1] != 'x':
                self.agv_status[0] = 'idle'
                self.update_agv_status()


    def check_battery_and_emergency(self):
        """ # Ooops what about my battery! we dont have to check it everytime though. so there is like an update interval. """
        current_time_ns = time.time_ns()
        # print("time passed: ", (current_time_ns - self.last_state_clearance_nsec) * 1e-9)
        if (current_time_ns - self.last_state_clearance_nsec) * 1e-9 > self.ros_state_clearance_intervel_sec: # 60s - 1min so, 240s?
            try:
                self._handle_emergency()
                self._update_battery_status()
                self.last_state_clearance_nsec = current_time_ns
            except psycopg2.Error as error:
                print(f"[db_manager]: Error - {error}")

# ------------------------------------------------------------------ #
# test cases for the above functions                                 #
# ------------------------------------------------------------------ #

class TestDatabaseManager(TestCase):

    def setUp(self):
        """ test create robot """
        robot_id = 'TB3_15'
        fleet_id = 'kullar'
        current_pose = [1.0, -3.0, 0.0]
        wheel_separation = 0.1
        wheel_radius = 0.04
        lin_vel = 0.4
        ang_vel = 0.9
        bat_stat = 50.0
        hostname = "192.168.1.93"
        database = "postgres"
        username = "postgres"
        pwd = "root"
        port = 5432
        map1_path = "/home/hazeezadebayo/docker_ws/ros2_ws/viro_agv/src/viro_core/viro_core/maps/my_cartographer_map.pgm"
        map2_path = "/home/hazeezadebayo/docker_ws/ros2_ws/viro_agv/src/viro_core/viro_core/maps/devshop.pgm"

        db_registration = DatabaseRegistration(
            robot_id, fleet_id, map1_path,
            map2_path, hostname, database, username,
            pwd, port, ang_vel, lin_vel, wheel_separation,
            wheel_radius, current_pose, bat_stat)

        db_registration.shutdown = 'no'
        db_registration.traffic = 'none'
        db_registration.wait_traffic = 'none'
        db_registration.last_checkpoint = 'unknown,'
        db_registration.checkpoints = 'A11,A8,A3,A8,A11,A12,A7,A5' # 'A,A,A'
        db_registration.waitpoints = 'W12,W11,W7,W8' # 'W,W,W'
        db_registration.notifications = 'notice_msg,notice_msg,node_troubleshoot,0,None,wt'
        db_registration.agv_status = 'active,x,y,z,w,base_floor,red'
        db_registration.m_controls = 'no,False,False,False,False,False'
        db_registration.landmark = 'low,transport,A8_A3,A7_A5,A6_A10' # 'none'
        db_registration.wait_itinerary = '{{0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0}}'
        db_registration.agv_itinerary = '{{0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0}}'

        # register robot on database and check status
        if db_registration.register_robot() is False:
            print("[main]:- registration failed. status returned False. \n")
            return

        # Create instance of DatabaseManager
        self.db_manager = DatabaseManager(
            hostname=hostname, database=database,
            username=username, pwd=pwd, port=port,
            robot_id=robot_id, fleet_id=fleet_id
        )


    def test_read_db(self):
        """Test read from the database."""
        self.db_manager.read_db()
        # Check that instance variables are updated correctly
        self.assertEqual(self.db_manager.shutdown, 'no')
        self.assertEqual(self.db_manager.current_pose, ['1.0', '-3.0', '0.0'])
        self.assertEqual(self.db_manager.notifications, ['notice_msg', 'notice_msg', 'node_troubleshoot', '0', 'None', 'wt'])
        self.assertEqual(self.db_manager.last_checkpoint, ['unknown',""])
        self.assertEqual(self.db_manager.checkpoints, ['A11','A8','A3','A8','A11','A12','A7','A5'])
        self.assertEqual(self.db_manager.waitpoints, ['W12', 'W11', 'W7', 'W8'])
        self.assertEqual(self.db_manager.traffic, 'none')
        self.assertEqual(self.db_manager.wait_traffic, 'none')
        self.assertEqual(self.db_manager.wait_itinerary, [[0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]])
        self.assertEqual(self.db_manager.agv_status, ['active', 'x', 'y', 'z', 'w', 'base_floor', 'red'])
        self.assertEqual(self.db_manager.agv_itinerary, [[0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]])
        # self.assertEqual(self.db_manager.datetime_info, ['2024-08-11', '12:00:00'])
        self.assertEqual(self.db_manager.controls, ['no', 'False', 'False', 'False', 'False', 'False'])
        self.assertEqual(self.db_manager.landmarks, ['low', 'transport', 'A8_A3', 'A7_A5', 'A6_A10'])
        self.assertEqual(self.db_manager.model_config, ['0.1', '0.04', '0.4', '0.9', '50.0'])


    def test_update_db(self):
        """Test updating the database."""
        # read from db
        self.db_manager.read_db()
        # values that we wish to update
        # usually position, battery and last_checkpoints.
        self.db_manager.ros_system_launch_process = True
        self.db_manager.ros_dummy_checkpoints = self.db_manager.checkpoints
        self.db_manager.ros_last_checkpoint = ['A8','A3','A8','A11','A12','A7','A5']
        self.db_manager.ros_pose_x = float(1.1)
        self.db_manager.ros_pose_y = float(-3.0)
        self.db_manager.ros_pose_th = float(0.0)
        self.db_manager.ros_battery_percentage = 60.0
        # db_mngr.ros_notifications = []
        # db_mngr.ros_dock_status = self.dock_status
        # db_mngr.ros_emergency_present = self.emergency_present
        # Invoke the method
        self.db_manager.update_db()
        # Check that the database was updated
        self.db_manager.read_db()
        self.assertEqual(self.db_manager.current_pose, ['1.100', '-3.000', '0.000'])
        self.assertEqual(self.db_manager.last_checkpoint, ['A8','A3','A8','A11','A12','A7','A5'])
        self.assertEqual(self.db_manager.model_config, ['0.1', '0.04', '0.4', '0.9', '60.0'])


    def test_update_db_with_shutdown(self):
        """Test database update when shutdown is triggered."""
        # read from db
        self.db_manager.read_db()
        # test what happens if we shutdown mid task,
        # we surely want to continue from where we stopped yeah.
        # so we reshuffle the checkpoints so that
        # where we stopped becomes first item in checkpoints
        self.db_manager.shutdown = 'yes'
        self.db_manager.ros_dummy_checkpoints = self.db_manager.checkpoints
        self.db_manager.last_checkpoint = ['A11','A12','A7','A5']
        self.db_manager.reshuffle_checkpoints()


    def tearDown(self):
        self.db_manager.conn.close()

# ----------------------------------------------------------------------- #
#                                    main                                 #
# ----------------------------------------------------------------------- #

if __name__ == '__main__':
    main()
