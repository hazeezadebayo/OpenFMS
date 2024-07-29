#!/usr/bin/env python3

import warnings, time, math, psycopg2, psycopg2.extras, collections
import numpy as np
import threading
warnings.filterwarnings("ignore", category=np.VisibleDeprecationWarning)

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
    """ DatabaseManager """
    def __init__(self, hostname, database, username, pwd, port, robot_id, fleet_id):

        # Create a lock object
        self.lock = threading.Lock()

        # db_stuff
        self.conn = psycopg2.connect(host=hostname, dbname=database, user=username, password=pwd, port=port)
        self.cur = self.conn.cursor(cursor_factory=psycopg2.extras.DictCursor) # pass any command into action

        # helper variables
        self.previous_emergency_present = False
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
        self.ros_state_clearance_intervel_sec = 250 # seconds
        self.ros_emergency_present = None
        self.ros_battery_percentage = None

        # output
        self.flag = None

        # init DB variables
        self.robot_id = robot_id
        self.fleet_id = fleet_id
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

    # ----------------------------------------------------------------------- #
    #                             helper_func                                 #
    # ----------------------------------------------------------------------- #

    def db_list_to_pylist(self, dblist):
        """
        Helper function to convert db string in the form of dictionary to list.
        """
        # Initialize the result list
        result = []
        # Iterate over the dblist sublists
        for sublist in dblist:
            # Strip the curly braces from the sublist
            sublist = sublist.strip("{}")
            # Split the sublist on "," to get the individual elements
            elements = sublist.split(",")
            # Convert the elements to floats
            elements = [float(x) for x in elements]
            # Append the elements to a new list
            new_list = elements
            # Append the new list to the result list
            result.append(new_list)
        # Print or return the result
        return result # print(result)


    # ----------------------------------------------------------------------- #
    #                                 read_db                                 #
    # ----------------------------------------------------------------------- #

    def read_db(self):
        """
        Read from database.
        """
        try: # try to open a connection yeah. but if it closes abruptly, # connection will never be closed hence, move 'close()' to finally
            #select_script = '''SELECT * FROM table_robot''' # '''SELECT id,name FROM table_robot'''
            select_script = "SELECT * FROM table_robot WHERE robot_id = %s;"
            select_id = (self.robot_id,)
            self.cur.execute(select_script, select_id)
            # print(cur.fetchall()) # everyone gets printed on one line.
            for record in self.cur.fetchall(): #print(record) # now everyone gets its own line e.g [2, 'Robin', 22500, 'D1'] [3, 'Xavier', 19500, 'D2']
                self.shutdown = str(record['shutdown']) # print(self.shutdown)
                self.current_pose = str(record['current_pose']).split(',') # print(self.current_pose[0], self.current_pose[1], self.current_pose[2])
                self.notifications = str(record['notifications']).split(',') # print(self.notifications[0], self.notifications[1], self.notifications[2], self.notifications[3])
                self.last_checkpoint = str(record['last_checkpoint']).split(',')
                self.checkpoints = str(record['checkpoints']).split(',') # print(len(self.checkpoints),"-route: ", self.checkpoints) # [A,B,C,D]
                self.waitpoints = str(record['waitpoints']).split(',')
                self.traffic = str(record['traffic']) # print(self.traffic)
                self.wait_traffic = str(record['wait_traffic'])
                self.wait_itinerary = self.db_list_to_pylist(str(record['wait_itinerary']).split("},{"))
                self.agv_status = str(record['agv_status']).split(',') # print(self.agv_status)
                fleet_id = str(record['fleet_id']) #print(str(record['fleet_id']))
                self.agv_itinerary = self.db_list_to_pylist(str(record['agv_itinerary']).split("},{")) # Split the string on "},{" to get a list of strings
                self.datetime_info = str(record['created_at']).split() # print(datetime_info[0], datetime_info[1])  #print(record['created_at'])
                self.controls = str(record['m_controls']).split(',') # print(self.controls[0], self.controls[1], self.controls[2], self.controls[3])
                self.landmarks = str(record['landmark']).split(',')
                self.model_config = str(record['model_config']).split(',') # print(self.controls[0], self.controls[1], self.controls[2], self.controls[3])
        except psycopg2.Error as error: # show the exceptions basically as errors.
            print(error)

        # print("agv_status: ", self.agv_status)
        if  self.fleet_id != fleet_id:
            self.shutdown = "yes"
            print("Ooops!")
            print("robot fleet id provided differs from the fetched fleet id in the db.")
            print("shutdown command will be sent until rectified.")
            print(" ")

        # cmd:
        floor = self.agv_status[5]
        self.flag = self.agv_status[6]

        print("[db_manager]: FLAG ---> "+str([self.agv_status[6]])+" SKIP ---> "+str([self.notifications])+". \n")

        if (self.notifications[4] == "skip") and (self.skip_node is None):
            self.skip_node = self.ros_last_checkpoint[0]
            self.flag += ',skip'
        elif self.flag == 'green':
            # [next stop occuppied -robot standoff-]:
            if (str(self.notifications[5]) != 'wt') and (str(self.wait_traffic) != 'none') and (self.wait_start_time is None):
                # negotiation! we need to publish [wt,wait_traffic] routes.
                self.wait_start_time = time.time()  # Record the start time
                cmd = ','.join([str(self.notifications[5]),str(self.wait_traffic)])
                self.flag += ','+cmd
            elif (floor != self.floor) and (self.ros_elevator_map_request is not None):
                self.ros_elevator_map_request = None
                self.floor = floor
                self.flag += ','+self.floor


    # ----------------------------------------------------------------------- #
    #                                update_db                                #
    # ----------------------------------------------------------------------- #

    def update_db(self):
        """
        This function updates the db and performs the fleet management traffic task.

        Output:
        string: self.flag = green, red [red, x_coord,y_coord]
        """

        # Acquire the lock
        with self.lock:

        # Acquire lock before accessing shared data (db_count)
        # self.lock.acquire()

            # if we have a task, and a shutdown command is sent then we want to save the last state by reshuffling the checkpoints.
            # so that the place we wanna visit next starts the stack on our next turn on or active state.
            if (self.ros_dummy_checkpoints == self.checkpoints) and (self.ros_dummy_checkpoints != ['A','A','A']) and (self.shutdown == 'yes'):
                if (self.last_checkpoint[0] != 'unknown') and (self.landmarks[1] != "clean"):
                    # reshuffle checkpoints for restart
                    reshuffle_index = len(self.checkpoints) - len(self.last_checkpoint)
                    checkpoints_method_1 = self.checkpoints[reshuffle_index:]+self.checkpoints[:reshuffle_index]
                    if len(checkpoints_method_1) != 0:
                        self.checkpoints = checkpoints_method_1
                        self.agv_itinerary = self.agv_itinerary[reshuffle_index:]+self.agv_itinerary[:reshuffle_index]
                        # print("method_1: ", checkpoints_method_1)
                        # has to go to the DB so that when it doesnt enter here upon shutdown == no, it still exists in DB for continuity.
                        try:
                            self.cur.execute('UPDATE table_robot SET checkpoints = %s, agv_itinerary = %s, last_checkpoint = %s WHERE robot_id = %s and fleet_id = %s;',
                                            (','.join(self.checkpoints), self.agv_itinerary, 'unknown', self.robot_id, self.fleet_id,))
                            self.conn.commit()
                        except psycopg2.Error as error: # show the exceptions basically as errors.
                            print(error)
                        self.ros_dummy_checkpoints = ['A','A','A'] # remove it in shutdown

            # if self.datetime_info[1] == self.last_datetime_info:
            #     return

            # what happens before i get to a specific checkpoint.
            # Again!! before i get to a specific checkpoint.
            # That is, what happens before it is reserved for me.
            if ((self.ros_system_launch_process is not None) and (self.ros_last_checkpoint[0] != 'unknown')) or \
                ((self.ros_system_launch_process is None) and (self.ros_last_checkpoint[0] == 'manual')):

                # print("[db_manager]: -------------- ")
                # Check the transaction status (avoid writing stale data)
                # print("[db_manager]: 1. pycopg2 -> "+str([self.conn.status])+". \n")

                try:
                    self.cur.execute('UPDATE table_robot SET current_pose = %s, last_checkpoint = %s WHERE robot_id = %s and fleet_id = %s;',
                            (f"{self.ros_pose_x:.3f},{self.ros_pose_y:.3f},{self.ros_pose_th:.3f}", ','.join(self.ros_last_checkpoint), self.robot_id, self.fleet_id))


                    # Log the current checkpoint status
                    print(f"[db_manager]:-ros_last_checkpoint- {self.ros_last_checkpoint}.")

                    # Skip checkpoint and dock related checks if task is cleaning
                    if self.checkpoints != ['A', 'A', 'A'] and self.ros_last_checkpoint[0] != 'manual' and self.landmarks[1] != "clean":

                        # we need to check all the traffic alphabets with our own to be sure that,
                        # where we wanna go has not been reserved by another robot.
                        # so, obtain robots own currently occupied checkpoint. if any,
                        # this robot's current reserved checkpoint is ...
                        reserved_checkpoint = self.traffic

                        next_stop_id = self.ros_last_checkpoint[0]
                        # use next_stop_id to return the real coordinates as next_stop_coordinate.
                        next_stop_coordinate = self.agv_itinerary[self.checkpoints.index(next_stop_id)]
                        # print(f"[db_manager]:-next_stop_id-: {next_stop_id}.")

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
                        self.cur.execute('UPDATE table_robot SET notifications = %s, wait_traffic = %s WHERE robot_id = %s and fleet_id = %s;',
                                        (','.join(self.notifications), self.wait_traffic, self.robot_id, self.fleet_id,))

                        # predecessor_landmark = []
                        real_landmark = []
                        for elements in self.landmarks[2:]:
                            # predecessor_landmark.append(elements.split('_')[0])
                            real_landmark.append(elements.split('_')[1])

                        # [next stop not occuppied -docked-]:
                        # -----------------------
                        ########################################################################################################
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
                                self.ros_dock_status = False
                                if self.agv_status[1] != 'x':
                                    self.agv_status[1] = 'x'
                                    self.agv_status[2] = 'y'
                                    self.agv_status[3] = 'z'
                                    self.agv_status[4] = 'w'
                                    self.cur.execute('UPDATE table_robot SET agv_status = %s WHERE robot_id = %s and fleet_id = %s;',
                                                    (','.join(self.agv_status), self.robot_id, self.fleet_id,))
                            # if it is a landmark, i have already home or dock. so lock the gate.
                            # if it is a landmark yeah, we want to show the previous checkpoint as the active state.
                            # so that no one blocks it way when it wants to come out/reverse.
                            # that is:
                            # anytime we dock at a landmark point, we always put our current state as though we were at the gate.
                            elif next_stop_id in real_landmark:
                                # agv_status holds state:
                                #       robot_state: active/inactive/idle [0],
                                #       dock coordinate: coord_x [1], coord_y [2], coord_z [3], coord_w [4]
                                # we put real landmarks in the agv_status db, like a sock on the door.
                                # That is, if we are inside the room/dock, we need the coordinate regardless so we save it in agv_status[1:]
                                # if a value is there, then we must undock first before we navigate. its like a memory, open the door first kinda.
                                # dont just go out blindly.
                                # we know it will not be x if it is idle. i.e. available for a task, x y z w holds the dock station address where the robot is.
                                # the only logical stuff here is that if next stop id is in real landmark then i am either at the real landmark.
                                # it means that where i wanna go is an actual landmark and although i have not docked yet. i am already in the ballpark so
                                # over-write my current dock memory as if anything happens now. i need to call undock first anyway.
                                if self.ros_dock_status:
                                    self.ros_dock_status = False
                                    # if we docked successfuly, we need to make a decision on what to do next,
                                    # based on what our current task is:
                                    # move or charge is quite similar, its go from one point to another and stay there.
                                    # only difference is, if move target is occupied it waits forever,
                                    # while if charge target is occupied, it finds another location.
                                    if self.landmarks[1] in ["move", "charge", "transport"]:
                                        if reserved_checkpoint in real_landmark[2:] or self.landmarks[1] in ["move", "charge"]:
                                            self.ros_last_checkpoint = ['unknown']
                                            if self.agv_status[0] == 'active':
                                                self.flag = 'red'
                                                self.agv_status[0] = 'idle'
                                                self.agv_status[1] = str(next_stop_coordinate[0])
                                                self.agv_status[2] = str(next_stop_coordinate[1])
                                                self.agv_status[3] = str(next_stop_coordinate[2])
                                                self.agv_status[4] = str(next_stop_coordinate[3])
                                                self.agv_status[6] = self.flag
                                                self.cur.execute('UPDATE table_robot SET agv_status = %s WHERE robot_id = %s and fleet_id = %s;',
                                                                (','.join(self.agv_status), self.robot_id, self.fleet_id,))
                                                print("[db_manager]: idle updated")
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
                                    elif self.landmarks[1] == "loop" and self.landmarks[0] == "high":
                                            if self.ros_last_checkpoint[0] == real_landmark[0] and len(self.ros_last_checkpoint) > 1:
                                                while self.checkpoints.index(real_landmark[0]) > 0:
                                                    self.checkpoints.pop(0)
                                                    self.agv_itinerary.pop(0)
                                                self.landmarks[0] = "low"
                                                self.cur.execute('UPDATE table_robot SET landmark = %s, checkpoints = %s, agv_itinerary = %s WHERE robot_id = %s and fleet_id = %s;',
                                                                (','.join(self.landmarks), ','.join(self.checkpoints), self.agv_itinerary, self.robot_id, self.fleet_id,))
                                            self.ros_dummy_checkpoints = self.checkpoints


                    # all the jargons we have discussed and decided yeah,
                    # sign it and stamp it.
                    self.conn.commit()
                    # print("[db_manager]: 2. pycopg2 ---> "+str([self.conn.status])+". \n")
                # God forbid we get an error!
                # lol. we are fucked!
                # show the exceptions basically as errors.
                except psycopg2.Error as error:
                    print(f"[db_manager]:-db error- {error}.")

            # give me some breathing space bro!
            time.sleep(0.3)
            # Release lock after critical section
            # self.lock.release()

        # Ooops what about my battery! we dont have to check it everytime though. so there is like an update interval.
        # ----------------------------------------- update battery status

        # if (time.time_ns() - self.last_state_clearance_nsec)*1e-9 > self.ros_state_clearance_intervel_sec: # 60s - 1min so, 240s?
        #     try:
        #         print("[db_manager]:   1 ")
        #         # if the emergency button is pressed:
        #         if self.ros_emergency_present is not None:
        #             print("[db_manager]:   2 ")
        #             if (self.ros_emergency_present is True) and ((self.agv_status[0] == 'active') or (self.agv_status[0] == 'idle')):
        #                 print("[db_manager]:   3 ")
        #                 self.agv_status[0] = 'inactive'
        #                 self.previous_emergency_present = True
        #                 self.cur.execute('UPDATE table_robot SET agv_status = %s WHERE robot_id = %s and fleet_id = %s;',
        #                                 (','.join(self.agv_status), self.robot_id, self.fleet_id,))
        #                 print("[db_manager]:   4 ")
        #             elif (self.ros_emergency_present is False) and (self.previous_emergency_present != self.ros_emergency_present):
        #                 print("[db_manager]:   5 ")
        #                 if self.agv_status[1] != 'x': # then robot was idle before, hence...
        #                     print("[db_manager]:   6 ")
        #                     self.agv_status[0] = 'idle'
        #                 elif self.agv_status[0] != 'inactive': # it probably was active before
        #                     print("[db_manager]:   7 ")
        #                     self.agv_status[0] = 'active'
        #                 print("[db_manager]:   8 ")
        #                 self.previous_emergency_present = False
        #                 self.cur.execute('UPDATE table_robot SET agv_status = %s WHERE robot_id = %s and fleet_id = %s;',
        #                                 (','.join(self.agv_status), self.robot_id, self.fleet_id,))
        #                 print("[db_manager]:   9 ")
        #         print("[db_manager]:   10 ")
        #         # update battery regardless.
        #         self.cur.execute('UPDATE table_robot SET model_config = %s WHERE robot_id = %s and fleet_id = %s;',
        #                          (str(self.ros_battery_percentage), self.robot_id, self.fleet_id,))
        #         print("[db_manager]:   11 ")
        #         self.conn.commit()
        #         print("[db_manager]:   12 ")
        #     except psycopg2.Error as error: # show the exceptions basically as errors.
        #         print("[db_manager]: ", error)
        #     print("[db_manager]:   13 ")
        #     self.last_state_clearance_nsec = time.time_ns()

        # finally,
        # update our last db update time.
        self.last_datetime_info = self.datetime_info

# ----------------------------------------------------------------------- #
#                                    main                                 #
# ----------------------------------------------------------------------- #
if __name__ == '__main__':
    ROBOT_ID = 2
    FLEET_ID = "kullar"
    # CURRENT_POSE = [6.1, -4.6, 0.0]
    HOSTNAME = "localhost"
    DATABASE = "postgres"
    USERNAME = "postgres"
    PWD = "root"
    PORT = "5432"
    db_manager = DatabaseManager(HOSTNAME,
                                 DATABASE,
                                 USERNAME,
                                 PWD,
                                 PORT,
                                 ROBOT_ID,
                                 FLEET_ID)

# if fleet_map_path.is_file():
#     file = open(fleet_map_path,'rb')
#     filedata = file.read()
#     self.cur.execute("UPDATE table_robot SET map_data = %s WHERE robot_id = %s and fleet_id = %s;", (filedata,robot_id,fleet_id,))
#     self.conn.commit()
#     file.close()




































            #     try:
            #         self.cur.execute('UPDATE table_robot SET current_pose = %s, last_checkpoint = %s WHERE robot_id = %s and fleet_id = %s;',
            #                         (str(self.ros_pose_x)+","+str(self.ros_pose_y)+","+str(self.ros_pose_th), ','.join(self.ros_last_checkpoint), self.robot_id, self.fleet_id,))
            #         # if we have a task,
            #         # we dont wanna check checkpoints or dock related stuff or reserved traffic stuff if the task is cleaning. since the path
            #         # for cleaning is not in the graph. the fleet manager didnt create a coverage path hence routes might deviate from normal
            #         # nav cases so we dont want to treet it like being path of the fleet.
            #         print("[db_manager]:-ros_last_checkpoint-"+str([self.ros_last_checkpoint])+".")
            #         if (self.checkpoints != ['A','A','A']) and (self.ros_last_checkpoint[0] != 'manual') and (self.landmarks[1] != "clean"):
            #             # where we are currently headed is our next_stop_id and it should be the first element or alphabet in our checkpoint list.
            #             next_stop_id = self.ros_last_checkpoint[0]
            #             print("[db_manager]:-next_stop_id-: "+str([next_stop_id])+".")
            #             # predecessor_landmark is like the gate of the dock station. so like a point just in front of the landmark/dock
            #             # where we have to pass first in order to get to the dock or real_landmark itself.
            #             predecessor_landmark = []
            #             real_landmark = []
            #             for elements in self.landmarks[2:]:
            #                 predecessor_landmark.append(elements.split('_')[0])
            #                 real_landmark.append(elements.split('_')[1])
            #             # if where agv is going does infact exist, in our memory of paths or checkpoints,
            #             if next_stop_id in self.checkpoints:
            #                 # use it to return the real coordinates as next_stop_coordinate.
            #                 next_stop_coordinate = self.agv_itinerary[self.checkpoints.index(next_stop_id)]
            #                 # if where we are going is a landmark, e.g. charge station, dock station, delivery/pickup point etc
            #                 # we need to keep the location x y z w pose of the place. we already have an associated alphabet A B C etc though
            #                 # if the place we are going to is not just a checkpoint but a landmark. this is so that if we dock. and they turn us off
            #                 # and on again. we can know the dock station we currently are in.
            #                 # also we need to update elevator status no matter what!
            #                 # we,
            #                 if self.ros_elevator_map_request is not None:
            #                     self.agv_status[5] = self.ros_elevator_map_request
            #                     self.ros_elevator_map_request = None
            #                 # Then proceed as you would
            #                 # i am going to a dock or leaving a dock station?
            #                 if next_stop_id in real_landmark:
            #                     # agv_status holds state:
            #                     #       robot_state: active/inactive/idle [0],
            #                     #       dock coordinate: coord_x [1], coord_y [2], coord_z [3], coord_w [4]
            #                     # we dont put real landmarks in the db, we put the predecessor, like a sock on the door.
            #                     # but if we are inside the room/dock, we need the coordinate regardless so we save it in agv_status[1:]
            #                     # if a value is there, then we must undock first before we navigate. its like a memory, open the door first kinda.
            #                     # dont just go out blindly.
            #                     # we know it will not be x if it is idle. i.e. available for a task, x y z w holds the dock station address where the robot is.
            #                     # the only logical stuff here is that if next stop id is in real landmark then i am either at the real landmark.
            #                     # it means that where i wanna go is an actual landmark and although i have not docked yet. i am already in the ballpark so
            #                     # over-write my current dock memory as if anything happens now. i need to call undock first anyway.
            #                     if self.agv_status[1] == 'x':
            #                         self.agv_status[1] = str(next_stop_coordinate[0])
            #                         self.agv_status[2] = str(next_stop_coordinate[1])
            #                         self.agv_status[3] = str(next_stop_coordinate[2])
            #                         self.agv_status[4] = str(next_stop_coordinate[3])
            #                         self.cur.execute('UPDATE table_robot SET agv_status = %s WHERE robot_id = %s and fleet_id = %s;', (','.join(self.agv_status), self.robot_id, self.fleet_id,))
            #                 # this implies that i am leaving the gate or i am leaving a dock station.
            #                 else:
            #                     # this implies that, i have now decided to undock and must over-write the location contained in my memory about the dock station i was in.
            #                     if self.agv_status[1] != 'x':
            #                         self.agv_status[1] = 'x'
            #                         self.agv_status[2] = 'y'
            #                         self.agv_status[3] = 'z'
            #                         self.agv_status[4] = 'w'
            #                         self.cur.execute('UPDATE table_robot SET agv_status = %s WHERE robot_id = %s and fleet_id = %s;', (','.join(self.agv_status), self.robot_id, self.fleet_id,))
            #                 # Notification: error_msg_ros, error_msg_db, node_troubleshoot, wait_time(0), post/pre/None, x, y
            #                 # [0] error_msg_ros: is ros based stuffs like -->
            #                 #     dock_required | dock_completed | undock_required | undock_completed | reverse_started | reverse_completed | collision
            #                 # [1] error_msg_db: is essentially same as first but only if we wish to notify user via text or mail on robot status.
            #                 # [2] node_troubleshoot: if a main node stops publishing, i.e. no msg after certain seconds. we notify
            #                 #     joint_state_failed | scan_failed | odom_failed | map_failed | map_failed | amcl_failed | cmd_vel_failed | robot_description_failed | goal_failed | tf_failed | core_startup_success
            #                 # [3] wait_time: how long robot has been waiting to use a particular checkpoint
            #                 # [4] pre/post/None: implies what stage of a negotiation a robot is with another robot (mobile executor MEx) in order to define new paths so as not to wait forever.
            #                 # [5,6] x,y: refers to the coordinate of a new mid-point/pose for which the robot must go in order to avoid the other robot instead of a stand-off.
            #                 # recoveries_server | behavior_server: -- system error msg -- dock related/occupied msg -- reverse/recovery msg
            #                 # ---------------------------------------------------------------------------------
            #                 # ros
            #                 # If any element in ros notifications is not None

            #                 #################
            #                 if any(element != 'None' for element in self.ros_notifications):
            #                     # Update the database
            #                     self.cur.execute('UPDATE table_robot SET notifications = %s WHERE robot_id = %s and fleet_id = %s;', (','.join(self.ros_notifications), self.robot_id, self.fleet_id,))
            #                     # Re-set all elements back to 'None'
            #                     self.ros_notifications = ['None', 'None', 'None', 'None', 'None', 'None', 'None']
            #                 #################

            #                 # initialize an empty traffic list. since this is update real time. so we want fresh information every time.
            #                 traffic_control = []
            #                 # check all robot's last updated next_stop_id as to where they would be or where they are going to.
            #                 self.cur.execute("SELECT DISTINCT traffic FROM table_robot") # ORDER BY something ascending descending etc
            #                 for i in self.cur.fetchall():
            #                     traffic_control.append(i[0])

            #                 print("[db_manager]: traffic: "+str([traffic_control])+".")
            #                 # we need to check all the traffic alphabets with our own to be sure that,
            #                 # where we wanna go has not been reserved by another robot.
            #                 # so, obtain robots own currently occupied checkpoint. if any,
            #                 self.cur.execute("SELECT traffic FROM table_robot WHERE robot_id = %s and fleet_id = %s;", (self.robot_id, self.fleet_id,))
            #                 for record in self.cur.fetchall():
            #                     # this robot's current reserved checkpoint is ...
            #                     reserved_checkpoint = str(record['traffic'])
            #                     print("[db_manager]: currently reserved id : "+str([reserved_checkpoint])+". \n")
            #                     print("[db_manager]: next stop id, db_count : "+str([next_stop_id, self.db_count])+". \n")
            #                     # if this robot is not the one currently occupying where it wants to go to.
            #                     if reserved_checkpoint != next_stop_id:
            #                         # [next stop occuppied]:
            #                         # -----------------------
            #                         print("[db_manager]: reserved_checkpoint != next_stop_id.")
            #                         # it implies next_stop_id might be available so,
            #                         # we check if the headed checkpoint is being occupied by another robot.
            #                         if next_stop_id in traffic_control:
            #                             print("[db_manager]: next id occupied.")
            #                             # oh shit! it is occupied, we need to stop our robot.
            #                             # first, is our robot moving?, pause the robot.
            #                             if self.agv_status[0] == 'active':

            #                                 print("[db_manager]:-agv active-.")
            #                                 print("[db_manager]:-predecessor_landmark-"+str([predecessor_landmark])+".")

            #                                 # [ROBOT ACTIVE] CASE 1:
            #                                 #################################################################################################
            #                                 # if we are supposedly at the gate of a dock station/landmark,
            #                                 # and we have not been docked before, then...
            #                                 if next_stop_id in predecessor_landmark and self.notifications[4] == "None":
            #                                     # check what kind of landmark or dock station we are headed for precisely,
            #                                     # so we can decide what to do.
            #                                     # landmark = [task_priority 0, task_name 1, pick 2, drop 3, home_docks is 4 upwards 5 6 7 etc.]
            #                                     if self.landmarks[1] == "transport": # three landmarks in transport task - (pick, drop, home)
            #                                         # this is same as saying if we are not home; wait.
            #                                         # could be re-written as: "if next_stop_id not in self.landmarks[4:]" since 4 5 6 7 etc. represent home docks.
            #                                         # so if we are not headed for the gate that leads to (pick, drop, home),
            #                                         # it means we are just passing this place as we would pass a normal checkpoint,
            #                                         # predecessor_landmark 2 or upwards imply home gates
            #                                         if next_stop_id not in predecessor_landmark[2:]:
            #                                             # if we were not already waiting before, we need to start waiting till that checkpoint is free.
            #                                             if self.wait_time_switch is False:
            #                                                 # we begin waiting,
            #                                                 # recall that we entered this rabit hole because we wanted to stop/pause our robot since
            #                                                 # where we wanted to go "next_stop_id" was already occupied by another robot.
            #                                                 self.wait_time_switch = True
            #                                                 # please let the world know what time we made the decision to wait yeah.
            #                                                 # so we can know how long we would be here for
            #                                                 self.notifications[3] = time.time_ns()
            #                                             # we might even raise alarm if we had one
            #                                             # stop the robot
            #                                             # ----------------bookmark----------------------------- [output]
            #                                             print("[db_manager]:-decided to wait- dbcount:",self.db_count)
            #                                             self.flag = 'yellow' # 'inactive'
            #                                             # -----------------------------------------------------
            #                                             # so long as 'next_stop_id' is occupied, this loop will keep entering this condition,
            #                                             # therefore, we keep updating how long we have waited for by calculating time
            #                                             self.notifications[3] = str(time.time_ns() - int(self.notifications[3]))
            #                                             # shout it to the world mahn!
            #                                             self.cur.execute('UPDATE table_robot SET notifications = %s WHERE robot_id = %s and fleet_id = %s;',
            #                                                             (','.join(self.notifications), self.robot_id, self.fleet_id,))
            #                                         # if it is home from (pick, drop, home) in Transport task, and it is not the last on the list,
            #                                         # meaning it appears someone or some robot as docked in the home location we wanna go, but we realize that
            #                                         # there are more homes to go to, i.e. home is not just one id 4 5 6 7 ect so we try all of them in that order,
            #                                         # till we find an un-isrealed one.
            #                                         # we may even re-write this statement as:
            #                                         # " elif next_stop_id in self.landmarks[4:] and next_stop_id != self.landmarks[-1] "
            #                                         # predecessor_landmark 2 or upwards imply home gates
            #                                         elif next_stop_id in predecessor_landmark[2:] and next_stop_id != predecessor_landmark[-1]:
            #                                             # proceed to the next checkpoint. that is, +1 the check point. i.e. give the robot a new target say 4 --> 5.
            #                                             # ----------------bookmark----------------------------- [output]
            #                                             self.flag = 'red' # self.pub_route_stat(next_stop_id, 'goto_next')
            #                                             # -----------------------------------------------------
            #                                         # but if we are at the last home available or perhaps the only home available, then we need to seek help.
            #                                         # there is really no where else to go. # it is home and it is last; announce or notify
            #                                         # we can re-write the below code as: "elif next_stop_id == self.landmarks[-1]"
            #                                         elif next_stop_id == predecessor_landmark[-1]:
            #                                             # we need to start waiting here for help as we have no where to go anymore.
            #                                             if self.wait_time_switch is False:
            #                                                 # we need to take note of the time we made the decision to wait.
            #                                                 self.wait_time_switch = True
            #                                                 self.notifications[3] = time.time_ns()
            #                                             # we would need to stop the robot still. before we raise alarm so,
            #                                             # ----------------bookmark----------------------------- [output]
            #                                             print("[db_manager]:-a2-.")
            #                                             self.flag = 'yellow' # 'inactive'
            #                                             # -----------------------------------------------------
            #                                             self.notifications[3] = str(time.time_ns() - int(self.notifications[3]))
            #                                             # you might want to notify the user or admin
            #                                             self.notifications[0] = "Inactive: all home_dock occupied."
            #                                             self.cur.execute('UPDATE table_robot SET notifications = %s WHERE robot_id = %s and fleet_id = %s;',
            #                                                             (','.join(self.notifications), self.robot_id, self.fleet_id,))
            #                                     # we have looked at transport based task,
            #                                     # what about if it was a loop task
            #                                     # loop task has only two landmarks (pick, drop) - loop.
            #                                     # therefore,
            #                                     elif self.landmarks[1] == "loop":
            #                                         # we have no choice but to wait since this task is a forever task yeah.
            #                                         if self.wait_time_switch is False:
            #                                             self.wait_time_switch = True
            #                                             # what time did we reach the decision to wait.
            #                                             self.notifications[3] = time.time_ns()
            #                                         # again, we need to stop the robot first.
            #                                         # ----------------bookmark----------------------------- [output]
            #                                         print("[db_manager]:-a3-.")
            #                                         self.flag = 'yellow' # 'inactive'
            #                                         # -----------------------------------------------------
            #                                         # then we raise alarm to notify user while we keep track of how long we have been waiting for
            #                                         self.notifications[3] = str(time.time_ns() - int(self.notifications[3]))
            #                                         self.cur.execute('UPDATE table_robot SET notifications = %s WHERE robot_id = %s and fleet_id = %s;',
            #                                                         (','.join(self.notifications), self.robot_id, self.fleet_id,))
            #                                     # we have looked at transport and loop based task,
            #                                     # what about if it was a move task
            #                                     # move task has only one landmark (target) - move.
            #                                     # therefore, we will wait forever till that spot is un-Isrealed so that we can continue,
            #                                     # again recall that we entered this rabit hole because our next target is the gate of where we wanna dock.
            #                                     elif self.landmarks[1] == "move": #  2 landmarks (target) - move
            #                                         if self.wait_time_switch is False:
            #                                             self.wait_time_switch = True
            #                                             self.notifications[3] = time.time_ns()
            #                                         # again, we need to stop the robot first.
            #                                         # ----------------bookmark----------------------------- [output]
            #                                         print("[db_manager]:-a4-.")
            #                                         self.flag = 'yellow' # 'inactive'
            #                                         # -----------------------------------------------------
            #                                         # then we raise alarm to notify user while we keep track of how long we have been waiting for
            #                                         self.notifications[3] = str(time.time_ns() - int(self.notifications[3]))
            #                                         self.cur.execute('UPDATE table_robot SET notifications = %s WHERE robot_id = %s and fleet_id = %s;',
            #                                                         (','.join(self.notifications), self.robot_id, self.fleet_id,))
            #                                     # similarly,
            #                                     # if the task is to go charge the robot
            #                                     # just multiple landmarks but all (charge docks) - charge
            #                                     elif self.landmarks[1] == "charge":
            #                                         # for every charge dock, from robot's current position to the location of the charge dock/station,
            #                                         # there are checkpoints, if we have reached the last point on the checkpoint, and it is Isrealed.
            #                                         # we can skip to a new charge station, if it exists...
            #                                         # confirm that there is more location/checkpoints after this checkpoint.
            #                                         if len(self.ros_last_checkpoint) > 1:
            #                                             # proceed to the next checkpoint. That is +1 the check point
            #                                             # ----------------bookmark----------------------------- [output]
            #                                             self.flag = 'red' # self.pub_route_stat(next_stop_id, 'goto_next')
            #                                             # -----------------------------------------------------
            #                                         # but if we are at the very last, lol we have no choice but to stop and start waiting.
            #                                         elif len(self.ros_last_checkpoint) == 1:
            #                                             # print("No element exists after checkpoint in the list")
            #                                             if self.wait_time_switch is False:
            #                                                 self.wait_time_switch = True
            #                                                 self.notifications[3] = time.time_ns()
            #                                             # raise alarm
            #                                             # ----------------bookmark----------------------------- [output]
            #                                             print("[db_manager]:   a5 ")
            #                                             self.flag = 'yellow' # 'inactive'
            #                                             # -----------------------------------------------------
            #                                             self.notifications[0] = "Inactive: all charge_dock occupied."
            #                                             # then we raise alarm to notify user while we keep track of how long we have been waiting for
            #                                             self.notifications[3] = str(time.time_ns() - int(self.notifications[3]))
            #                                             self.cur.execute('UPDATE table_robot SET notifications = %s WHERE robot_id = %s and fleet_id = %s;',
            #                                                             (','.join(self.notifications), self.robot_id, self.fleet_id,))

            #                                 # [ROBOT ACTIVE] CASE 2:
            #                                 #################################################################################################
            #                                 # what if where we were headed which is reserved by another robot is not a predecessor landmark or gate.
            #                                 # and we have not docked before. then, we wait bro! we wait!!
            #                                 elif next_stop_id not in predecessor_landmark and self.notifications[4] == "None":
            #                                     # it is just a checkpoint. you have to wait till its available
            #                                     if self.wait_time_switch is False:
            #                                         self.wait_time_switch = True
            #                                         self.notifications[3] = time.time_ns()
            #                                     # raise alarm
            #                                     # ----------------bookmark----------------------------- [output]
            #                                     print("[db_manager]:   a6 ")
            #                                     self.flag = 'yellow' # 'inactive'
            #                                     # -----------------------------------------------------
            #                                     # then we raise alarm to notify user while we keep track of how long we have been waiting for
            #                                     self.notifications[3] = str(time.time_ns() - int(self.notifications[3]))
            #                                     self.cur.execute('UPDATE table_robot SET notifications = %s WHERE robot_id = %s and fleet_id = %s;',
            #                                                     (','.join(self.notifications), self.robot_id, self.fleet_id,))

            #                                 # [ROBOT INACTIVE] CASE 1:
            #                                 #################################################################################################
            #                                 # we talked about notifications[4] earlier where i said "pre/post/one: implies what stage of a
            #                                 # negotiation a robot is with another robot (mobile executor MEx) in order to define new
            #                                 # paths so as not to wait forever. "
            #                                 # we are now waiting, but what if the person we are waiting for is also waiting for us.
            #                                 # if we have not started negotiation,
            #                                 if self.notifications[4] == "None":
            #                                     # i know we are waiting yeah. but we could be waiting forever.
            #                                     # we might have to negotiate with the other robot 'MEx', get his last_checkpoint and negotiation profile.
            #                                     self.cur.execute("SELECT last_checkpoint, notifications FROM table_robot WHERE traffic = %s and fleet_id = %s;",
            #                                                     (next_stop_id, self.fleet_id,) )
            #                                     for record in self.cur.fetchall():
            #                                         mex_last_checkpoint = str(record['last_checkpoint']).split(',')
            #                                         mex_notifications = str(record['notifications']).split(',')
            #                                     # at this point, reserved_checkpoint is still the place i am but not where i want to go (next_stop_id).
            #                                     # if at some point in time, the place i am is where the other guy wants to come.
            #                                     if mex_last_checkpoint[0] == reserved_checkpoint:
            #                                         # we should start negotiation, put my 'how long ive been waiting' and my 'task priority' into consideration.
            #                                         if int(mex_notifications[3]) != 0:
            #                                             # To avoid each other, the strategy is simple,
            #                                             # one person goes or drives right and the other goes or drives left,
            #                                             print("---------------------------------, ", int(mex_notifications[3]) )
            #                                             left, right = self.shift_coordinates([float(self.ros_pose_x),float(self.ros_pose_y)],
            #                                                                                 [float(next_stop_coordinate[0]),float(next_stop_coordinate[1])],
            #                                                                                 float(self.model_config[1]),
            #                                                                                 0.5)
            #                                             # set our respective negotiation stage to pre-negotiation. it has not been agreed upon.
            #                                             self.notifications[4] = "pre"
            #                                             # if my 'how long ive been waiting' is greater, then im the one to move to the intended position while he waits at shifted midpoint
            #                                             if int(mex_notifications[3]) < int(self.notifications[3]):
            #                                                 self.notifications[5] = str(round(float(left[0]), 2))
            #                                                 self.notifications[6] = str(round(float(left[1]), 2))
            #                                             else:
            #                                                 self.notifications[5] = str(round(float(right[0]), 2))
            #                                                 self.notifications[6] = str(round(float(right[1]), 2))
            #                                             # we now have an agreement as while this code block is individual,
            #                                             # the math will agree at both robot's run because its done relative to wait times.
            #                                             # update our db:
            #                                             self.cur.execute('UPDATE table_robot SET notifications = %s WHERE robot_id = %s and fleet_id = %s;',
            #                                                             (','.join(self.notifications), self.robot_id, self.fleet_id,))
            #                                 # we always start at None. its the default state at birth of the robot.
            #                                 # now we move to pre-negotiation,
            #                                 elif self.notifications[4] == "pre":
            #                                     # stop the robot,
            #                                     # tell it where to go next as an emergency goal
            #                                     # ----------------bookmark----------------------------- [output]
            #                                     flag = 'red' # 'inactive'
            #                                     cmd = ','.join([str(self.notifications[5]),str(self.notifications[6])])
            #                                     # negotiation! we need to publish [x,y] midpoint shifted routes.
            #                                     self.flag = flag+','+cmd
            #                                     # -----------------------------------------------------
            #                                     # then, calculate distance of robot to goal.
            #                                     d = math.sqrt((float(self.notifications[5]) - float(self.ros_pose_x))**2 + (float(self.notifications[6]) - float(self.ros_pose_y))**2)
            #                                     # wait for it to get to some threshold of the place wherein considered success.
            #                                     if d < 0.55:
            #                                         # negotiation is a success. we can now goto the post negotiation phase.
            #                                         self.notifications[4] = "post"
            #                                         # update our db.
            #                                         self.cur.execute('UPDATE table_robot SET notifications = %s WHERE robot_id = %s and fleet_id = %s;',
            #                                                         (','.join(self.notifications), self.robot_id, self.fleet_id,))
            #                                 # our next phase just after pre-negotiation and negotiation success.
            #                                 # comes post-negotiation, we re-set the data in our db that concerns negotiation.
            #                                 # we are no longer waiting, we are no longer goint to an intermediate point, we can now officially
            #                                 # swap our reserved pose for our respective next_stop_id.
            #                                 elif self.notifications[4] == "post":
            #                                     self.notifications[4] = "None"
            #                                     self.notifications[5] = "x"
            #                                     self.notifications[6] = "y"
            #                                     # update my reserved position and next_stop_id in traffic
            #                                     if next_stop_id in real_landmark:
            #                                         idx = real_landmark.index(next_stop_id)
            #                                         self.cur.execute('UPDATE table_robot SET traffic = %s, notifications = %s WHERE robot_id = %s and fleet_id = %s;',
            #                                                         (predecessor_landmark[idx],','.join(self.notifications), self.robot_id, self.fleet_id,))
            #                                     else:
            #                                         self.cur.execute('UPDATE table_robot SET traffic = %s, notifications = %s WHERE robot_id = %s and fleet_id = %s;',
            #                                                         (next_stop_id,','.join(self.notifications), self.robot_id, self.fleet_id,))
            #                                     # ----------------bookmark----------------------------- [output]
            #                                     self.flag = 'green' # 'active'
            #                                     # -----------------------------------------------------

            #                         # [next stop not occuppied]:
            #                         # -----------------------
            #                         ########################################################################################################
            #                         # else the checkpoint the robot is going to is not being occupied. so make a reservation.
            #                         else: # That is, robot is free to proceed and register itself in that checkpoint.
            #                             if self.agv_status[0] != 'reset':
            #                                 # if it is a landmark, i have already home or dock. so lock the gate.
            #                                 # if it is a landmark yeah, we want to show the previous checkpoint as the active state.
            #                                 # so that no one blocks it way when it wants to come out/reverse.
            #                                 # that is:
            #                                 # anytime we dock at a landmark point, we always put our current state as though we were at the gate.
            #                                 if next_stop_id in real_landmark:
            #                                     idx = real_landmark.index(next_stop_id)
            #                                     # ----------------bookmark----------------------------- [input]
            #                                     if self.ros_dock_status is False: # == 'undocked':
            #                                     # -----------------------------------------------------
            #                                         # ----------------bookmark----------------------------- [output]
            #                                         self.flag = 'green' # 'active'
            #                                         # -----------------------------------------------------
            #                                         self.ros_dock_status = None
            #                                         self.wait_time_switch = False
            #                                         self.notifications[3] = '0'
            #                                         self.cur.execute('UPDATE table_robot SET traffic = %s, notifications = %s WHERE robot_id = %s and fleet_id = %s;',
            #                                                         (predecessor_landmark[idx],','.join(self.notifications), self.robot_id, self.fleet_id,))
            #                                     # ----------------bookmark----------------------------- [input]
            #                                     elif self.ros_dock_status is True: # == 'docked':
            #                                         self.ros_dock_status = False # = 'undocked'
            #                                         # if we are at a dock station,
            #                                         # we no longer desire to enter this loop or rabit hole again
            #                                         # so we set this to 'anything' but alphabets basically.
            #                                         # because this is the first condition checked. pay attention!!
            #                                         if self.landmarks[1] != "loop":
            #                                             # because if the task is loop we continue navigating yeah
            #                                             # you know, it actually doesnt matter for other guys as last_checkpoint will
            #                                             # be published on ros again hence over-writing this.
            #                                             # this just ensures robot cools off for a bit before moving again.
            #                                             self.ros_last_checkpoint = ['unknown']
            #                                         # -----------------------------------------------------
            #                                         # if we docked successfuly, we need to make a decision on what to do next,
            #                                         # based on what our current task is:
            #                                         if self.landmarks[1] == "transport":
            #                                             # if reserved_checkpoint in self.landmarks[4:]:
            #                                             # print("[db_manager]: ")
            #                                             # print("[db_manager]: a7 reserved: "+str([reserved_checkpoint])+". \n")
            #                                             # print("[db_manager]: a7 pred_land: "+str([predecessor_landmark])+". \n")
            #                                             # print("[db_manager]: a7 pred_land[2:]: "+str([predecessor_landmark[2:]])+". \n")
            #                                             # same as if reserved_checkpoint is home, that is, if we are home:
            #                                             if reserved_checkpoint in predecessor_landmark[2:]:
            #                                                 # re-set agv status, set our stat to idle.
            #                                                 # recall that agv_status holds state, settled dock location
            #                                                 if self.agv_status[0] == 'active':
            #                                                     self.agv_status[0] = 'idle'
            #                                                     self.agv_status[1] = str(next_stop_coordinate[0])
            #                                                     self.agv_status[2] = str(next_stop_coordinate[1])
            #                                                     self.agv_status[3] = str(next_stop_coordinate[2])
            #                                                     self.agv_status[4] = str(next_stop_coordinate[3])
            #                                                     self.cur.execute('UPDATE table_robot SET agv_status = %s WHERE robot_id = %s and fleet_id = %s;',
            #                                                                     (','.join(self.agv_status), self.robot_id, self.fleet_id,))
            #                                                     # we also wanna canncel all illusion of motion.
            #                                                     # ----------------bookmark----------------------------- [output]
            #                                                     print("[db_manager]:   a7 ")
            #                                                     self.flag = 'yellow' # 'inactive'
            #                                                     # -----------------------------------------------------
            #                                         # if the task was loop
            #                                         elif self.landmarks[1] == "loop" and self.landmarks[0] == "high":
            #                                             # confirm that we are at the pick dock
            #                                             # all loop tasks starts with a high priority then transition after first pickup to low priority tasks
            #                                             # because they run forever yeah so we cant have every fucking robot on high priority out and about.
            #                                             # theres a chance that when the task was assigned we were not at the pickup location
            #                                             # so we first plan a path to the pickup location, if where we are now after driving is the first real landmark(pickup loc)
            #                                             # inside the real_landmark list then we can delete all the other points that come before this in our checkpoint memory
            #                                             # the reason is, if say start pos is X and we go to pick and we now move to drop, since its a cyclic task we do not want
            #                                             # to go to X and then pick and then drop and then x and then pick ... you get the gist. we want to remove the checkpoints that
            #                                             # essentially took us to our first pickup which represent path from x - pickup thus deleting x and so we are left with
            #                                             # pickup ---> drop --> pickup ---> drop ---> pickup blah blah blah.
            #                                             # you get the gist
            #                                             if self.ros_last_checkpoint[0] == real_landmark[0] and len(self.ros_last_checkpoint) > 1:
            #                                                 # if self.checkpoints.index(self.landmarks[2]) > 0:
            #                                                 if self.checkpoints.index(real_landmark[0]) > 0:
            #                                                     # print("There are elements before the first occurrence of 'self.landmarks[2]'")
            #                                                     # while self.checkpoints.index(self.landmarks[2]) > 0:
            #                                                     # delete the path from X --> pickup then,
            #                                                     while self.checkpoints.index(real_landmark[0]) > 0:
            #                                                         self.checkpoints.pop(0)
            #                                                         self.agv_itinerary.pop(0)
            #                                                     self.landmarks[0] = "low"
            #                                                     # we re-write our history!
            #                                                     self.cur.execute('UPDATE table_robot SET landmark = %s, checkpoints = %s, agv_itinerary = %s  WHERE robot_id = %s and fleet_id = %s;',
            #                                                                     (','.join(self.landmarks), ','.join(self.checkpoints), self.agv_itinerary, self.robot_id, self.fleet_id,))
            #                                             self.ros_dummy_checkpoints = self.checkpoints
            #                                         # move or charge is quite similar, its go from one point to another and stay there.
            #                                         # only difference is, if move target is occupied it waits forever,
            #                                         # while if charge target is occupied, it finds another location.
            #                                         elif self.landmarks[1] == "move" or self.landmarks[1] == "charge":
            #                                             # we live here now we used to be active but we can now rest in peace.
            #                                             if self.agv_status[0] == 'active':
            #                                                 # this is the robot chilling
            #                                                 self.agv_status[0] = 'idle'
            #                                                 # this is the robot's chill spot
            #                                                 self.agv_status[1] = str(next_stop_coordinate[0])
            #                                                 self.agv_status[2] = str(next_stop_coordinate[1])
            #                                                 self.agv_status[3] = str(next_stop_coordinate[2])
            #                                                 self.agv_status[4] = str(next_stop_coordinate[3])
            #                                                 # update our db:
            #                                                 self.cur.execute('UPDATE table_robot SET agv_status = %s WHERE robot_id = %s and fleet_id = %s;',
            #                                                                 (','.join(self.agv_status), self.robot_id, self.fleet_id,))
            #                                                 # ----------------bookmark----------------------------- [output]
            #                                                 print("[db_manager]:   a8 ")
            #                                                 self.flag = 'yellow' # 'inactive'
            #                                                 # -----------------------------------------------------
            #                                 # if next_stop_id is no longer occupied or was never occupied to begin with,
            #                                 # lets register the checkpoint as ours and reserve it in traffic.
            #                                 else:
            #                                     # yup! its a normal checkpoint/waypoint
            #                                     # ----------------bookmark----------------------------- [output]
            #                                     self.flag = 'green' # 'active'
            #                                     # -----------------------------------------------------
            #                                     self.wait_time_switch = False
            #                                     # like we never waited.
            #                                     self.notifications[3] = '0'
            #                                     # so, we keep it moving!
            #                                     self.cur.execute('UPDATE table_robot SET traffic = %s, notifications = %s WHERE robot_id = %s and fleet_id = %s;',
            #                                                     (next_stop_id, ','.join(self.notifications), self.robot_id, self.fleet_id,))
            #                     # will reach here if this robot is the one currently who has reserved the next_stop_id.
            #                     # i am where i wanna be!!
            #                     # i know its not a dock station because if it were, when i wanted to reserve the position,
            #                     # i would have moved to idle state and would have already been docked.
            #                     # and my dock_stat would have been NONE.
            #                     # just normal waypoints/checkpoints not docks.
            #                     else: # if the robot is the one currently occupying the radius for the checkpoint where its headed,
            #                         # then it should not have to report itself anymore.
            #                         # before it became mine, it was not mine and when it was not mine, i reserved it for myself.
            #                         # now i have reserved it for myself, and i know its not a dock station,
            #                         # i keep it moving like nothing happened.
            #                         self.ros_dock_status = False # = 'undocked'
            #                         # ----------------bookmark----------------------------- [output]
            #                         self.flag = 'green' # 'active'
            #                         # -----------------------------------------------------
            #         # all the jargons we have discussed and decided yeah,
            #         # sign it and stamp it.
            #         self.conn.commit()
            #         # print("[db_manager]: 2. pycopg2 ---> "+str([self.conn.status])+". \n")
            #     # God forbid we get an error!
            #     # lol. we are fucked!
            #     # show the exceptions basically as errors.
            #     except psycopg2.Error as error:
            #         print(f"Database error: {error}")
            #     except ValueError as error:
            #         print(f"Value error: {error}")
            #     except TypeError as error:
            #         print(f"Type error: {error}")
            # # give me some breathing space bro!
            # time.sleep(0.3)
            # # Release lock after critical section
            # # self.lock.release()


                                # self.cur.execute('UPDATE table_robot SET wait_traffic = %s WHERE robot_id = %s and fleet_id = %s;',
                                #     (self.wait_traffic, self.robot_id, self.fleet_id,))