#!/usr/bin/env python3

import os, warnings, yaml, time, ast, psycopg2, psycopg2.extras, collections
import numpy as np
from pathlib import Path
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
#               AGV CLIENT DatabaseRegistration                           #
# ----------------------------------------------------------------------- #

class DatabaseRegistration:
    """
        create table if table does not already exist
        and register agv if its id does not already exist
        if it does, update the table with some robot params
        all robots in a particular fleet should have same base_map
        where their respective elevators lead may differ.
        so long as you properly graphed it.
        thank God for graph theory. PROF. BURAK INER.
    """
    def __init__(self, robot_id, fleet_id, fleet_base_map_path, fleet_floor1_map_path,
                 hostname, database, username, pwd, port, ang_vel, lin_vel,
                 wheel_seperation, wheel_radius, curr_pose, bat_stat):
        self.robot_id = robot_id
        self.fleet_id = fleet_id
        self.pose_x, self.pose_y, self.pose_th = float(curr_pose[0]), float(curr_pose[1]), float(curr_pose[2])
        self.hostname = hostname  #'localhost' # home computer -->  '192.168.1.115'
        self.database = database  #'postgres'
        self.username = username  #'postgres'
        self.pwd = pwd            #'root' - ubuntu # 03210 - windows
        self.port_id = port       #5432
        self.wheel_seperation = wheel_seperation
        self.wheel_radius = wheel_radius
        self.lin_vel = lin_vel
        self.ang_vel = ang_vel
        self.battery_status = bat_stat
        self.fleet_base_map_data = None
        self.fleet_floor1_map_data = None

        fleet_base_map_file = Path(fleet_base_map_path)
        if fleet_base_map_file.is_file():
            file = open(fleet_base_map_file,'rb')
            self.fleet_base_map_data = file.read()
            file.close()
        else:
            print("robot not registered. no valid fleet map file path found.")
            return
        fleet_floor1_map_file = Path(fleet_floor1_map_path)
        if fleet_floor1_map_file.is_file():
            file = open(fleet_floor1_map_file,'rb')
            self.fleet_floor1_map_data = file.read()
            file.close()
        else:
            print("no first floor map provided. elevator can therefore not be used.")
        self.register_robot()


    ############ REGISTER AGV
    def register_robot(self):
        """
            register robot anew or update robot params
        """
        conn = None # this was done because if the connection was never opened then,
        # cur = None # finally will throw its own error. hence, to avoid that.
        # if you use the with instead of directly defining conn and cur then cur.close() is not necesary
        try: # try to open a connection yeah. but if it closes abruptly,
            # connection will never be closed hence, move 'close()' to finally
            with psycopg2.connect(host=self.hostname, dbname=self.database, user=self.username, password=self.pwd, port=self.port_id) as conn:
                with conn.cursor(cursor_factory=psycopg2.extras.DictCursor) as cur: # pass any command into action

                    # dont forget to uncomment this, because it means im deleting the table everytime i run this code.
                    # cur.execute('DROP TABLE IF EXISTS table_robot')

                    create_script = '''CREATE TABLE IF NOT EXISTS table_robot (
                                            robot_id varchar PRIMARY KEY,
                                            shutdown varchar(20) NOT NULL,
                                            current_pose varchar NOT NULL,
                                            traffic varchar,
                                            wait_traffic varchar,
                                            last_checkpoint varchar NOT NULL,
                                            checkpoints varchar,
                                            waitpoints varchar,
                                            notifications varchar NOT NULL,
                                            agv_status varchar,
                                            m_controls varchar(60) NOT NULL,
                                            landmark varchar,
                                            fleet_id varchar(50) NOT NULL,
                                            wait_itinerary varchar,
                                            agv_itinerary varchar,
                                            model_config varchar,
                                            base_map_data bytea,
                                            floor1_map_data bytea,
                                            created_at TIMESTAMP WITHOUT TIME ZONE DEFAULT CURRENT_TIMESTAMP  ) ''' # checkpoints varchar(40) NOT NULL,

                    cur.execute(create_script) # without commit, this will not be recognized.

                    insert_script = '''INSERT INTO table_robot (robot_id, shutdown, current_pose, notifications, last_checkpoint, checkpoints, waitpoints, agv_itinerary, \
                        agv_status, fleet_id, m_controls, traffic, landmark, model_config, base_map_data, floor1_map_data, wait_traffic, wait_itinerary)
                            VALUES (%(robot_id)s, %(shutdown)s, %(current_pose)s, %(notifications)s, %(last_checkpoint)s, %(checkpoints)s, %(waitpoints)s, \
                                %(agv_itinerary)s, %(agv_status)s, %(fleet_id)s, %(m_controls)s, %(traffic)s, %(landmark)s, %(model_config)s, \
                                    %(base_map_data)s, %(floor1_map_data)s, %(wait_traffic)s, %(wait_itinerary)s) \
                            ON CONFLICT (robot_id) DO UPDATE SET shutdown=%(shutdown)s, m_controls=%(m_controls)s, model_config=%(model_config)s, \
                                base_map_data=%(base_map_data)s, floor1_map_data=%(floor1_map_data)s;'''

                    insert_value = collections.OrderedDict(
                                    {   'robot_id':self.robot_id,          #'1',
                                        'shutdown':'no',
                                        'current_pose':str(self.pose_x)+","+str(self.pose_y)+","+str(self.pose_th), # '0.0,0.0,0.0',   0.0, -0.5, 0.0
                                        'traffic':'none',
                                        'wait_traffic':'none',
                                        'last_checkpoint':'unknown,',
                                        'checkpoints':'A,A,A',         # 'A,B,C,D,E,F,G,H', #  dont forget to make this same as dummy_checkpoints default
                                        'waitpoints':'W,W,W',
                                        'notifications':'notice_msg,notice_msg,node_troubleshoot,0,None,wt',  # error_msg,error_msg,node_troubleshoot,wait_time,post/pre/none,
                                        'agv_status':'active,x,y,z,w,base_floor,red', # status,x,y,z,w,floor,flag'
                                        'm_controls':'no,False,False,False,False,False',
                                        'landmark':'none',
                                        'fleet_id':self.fleet_id,
                                        'wait_itinerary': '{{0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0}}',
                                        'agv_itinerary': '{{0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0}}', # '[x,y,z,w],[x,y,z,w],[x,y,z,w]',
                                        'model_config':str(self.wheel_seperation)+","+str(self.wheel_radius)+","+str(self.lin_vel)+","+str(self.ang_vel)+","+str(self.battery_status),
                                        'base_map_data':self.fleet_base_map_data,
                                        'floor1_map_data':self.fleet_floor1_map_data,
                                    })

                    cur.execute(insert_script, insert_value)

                    # for now there are no job queues and that would mean that you can not have timed jobs,
                    # or monitor specific jobs by names. not types. etc.
                    # this features could be added later but for now. i dont care much for it.
                    # # does table_tasks exist? create it if does not.
                    # create_script = '''CREATE TABLE IF NOT EXISTS table_tasks (
                    #                         job_id int PRIMARY KEY,
                    #                         priority varchar(20) NOT NULL,
                    #                         fleet_id varchar(50) NOT NULL,
                    #                         landmark varchar, # more like its address
                    #                         description varchar,
                    #                         job_status varchar, # active, unclaimed, waiting etc...
                    #                         robot_id varchar(20) NOT NULL, # who is executing this job? who can?
                    #                         created_at TIMESTAMP WITHOUT TIME ZONE DEFAULT CURRENT_TIMESTAMP  ) '''
                    # cur.execute(create_script) # without commit, this will not be recognized.

        except psycopg2.Error as error: # show the exceptions basically as errors.
            print(error)

        finally: # whether code throws an exception while trying or not.
            # the code here will always run.
            if conn is not None:
                conn.close()


if __name__ == '__main__':

    # directory = os.path.abspath(os.path.join(os.path.dirname(__file__),'../../../..'))
    # CONFIG_PATH = Path(directory+'/fleet_mngr.yaml')

    ROBOT_ID = 1
    FLEET_ID = "kullar"
    CURRENT_POSE = [1.0, -3.0, 0.0]
    WHEEL_SEPERATION = 0.1
    WHEEL_RADIUS = 0.04
    LIN_VEL = 0.4
    ANG_VEL = 0.9
    BAT_STAT = 50.0
    HOSTNAME = "192.168.1.93"
    DATABASE = "postgres"
    USERNAME = "postgres"
    PWD = "root"
    PORT = 5432
    MAP1_PATH = "/home/hazeezadebayo/docker_ws/ros2_ws/viro_agv/src/viro_core/viro_core/maps/my_cartographer_map.pgm"
    MAP2_PATH = "/home/hazeezadebayo/docker_ws/ros2_ws/viro_agv/src/viro_core/viro_core/maps/devshop.pgm"
    database_registration = DatabaseRegistration(
        ROBOT_ID, FLEET_ID, MAP1_PATH, MAP2_PATH,
        HOSTNAME,DATABASE,USERNAME,PWD,PORT,ANG_VEL,
        LIN_VEL,WHEEL_SEPERATION,WHEEL_RADIUS,CURRENT_POSE,BAT_STAT)
