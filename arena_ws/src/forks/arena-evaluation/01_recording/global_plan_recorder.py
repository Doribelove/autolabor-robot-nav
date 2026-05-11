#!/usr/bin/env python3

# general packages
import numpy as np
import os
import sys
import subprocess
import json

# ros packages
import rospy
from nav_msgs.msg import Path

# for transformations
from tf.transformations import euler_from_quaternion

class recorder():
    def __init__(self) -> None:
        self.dir_path = os.path.dirname(os.path.abspath(__file__)) # get path for current file, does not work if os.chdir() was used
        self.target_file_path = os.path.dirname(self.dir_path) +"/01_recording/"#+ "/02_evaluation/"
        self.target_file = self.target_file_path + "/global_path_database.json"

        # check whether global path database file exists already, if not initialize
        if not os.path.exists(self.target_file):
            with open(self.target_file, "w+") as file:
                json.dump({}, file)
                file.close()

        # initialize variables to be recorded with default values
        self.scenario = rospy.get_param("scenario_file").replace(".json","").replace("eval/","")
        self.global_path_poses = []
        self.global_path_length = 0

        # subscribe to topics
        rospy.Subscriber("/globalPlan", Path, self.global_plan_callback)

    # define callback function for all variables and their respective topics
    def global_plan_callback(self, msg: Path):
        self.global_path_poses = [[p.pose.position.x,p.pose.position.y] for p in msg.poses]
        self.global_path_length = sum([np.linalg.norm(np.array(self.global_path_poses[i])-np.array(self.global_path_poses[i-1])) for i in range(1,len(self.global_path_poses))])

        with open(self.target_file,"r") as file: # read current json database
            database = json.loads(file.read())
            database[self.scenario] = {
                "globalPlan": self.global_path_poses,
                "globalPlan_length": self.global_path_length,
            }
            file.close()
        with open(self.target_file, "w+") as file: # write new global plan to database
            json.dump(database, file)
            file.close()

        subprocess.call(["killall","-9","rosmaster"]) # apt-get install psmisc necessary
        sys.exit() # kill ROS



if __name__=="__main__":
    rospy.init_node("global_plan_recorder")    
    global_plan_recorder = recorder()
    rospy.spin()