import pandas as pd
from dataclasses import dataclass
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.srv import GetPlan
import tf
import time

@dataclass
class Pose:
    x: float
    y: float
    z: float

class PathPlanner:
    """calulate best paths before starting the robot to save time on running"""
    def __init__(self):
        rospy.wait_for_service('/move_base/make_plan')
        self.planner: rospy.ServiceProxy = rospy.ServiceProxy('/move_base/make_plan', GetPlan)
        
    def planner_test(self, start, goal, tolerance):
        """calculate path from start to goal"""
        # plan = self.planner(start, goal)
        # return plan
        return [Pose(0, 0, 0), Pose(1, 1, 1), Pose(2, 2, 2)]

    def get_abs_difference(self, current_pos: PoseStamped, coords: PoseStamped, tolerance: int = 0) -> float:
        try:
            plan = self.planner(current_pos, coords, 0)
            poses = ((plan.plan.poses))
            # sum difference is our weight in the graph for a path
            sum_diff = 0
            for i, j in zip(range(0, len(poses)), range(1, len(poses))):
                #rospy.loginfo(poses[i].pose.position.x - poses[j].pose.position.x)
                # get sum of the absolute differences
                partial_difference = abs(poses[i].pose.position.x - poses[j].pose.position.x) + abs(poses[i].pose.position.y - poses[j].pose.position.y)
                sum_diff += partial_difference

            return sum_diff

        except Exception as e:
            rospy.loginfo(e)
            return None


    def plan(self):  
        # get current pos from rosbulla
        try:
            (trans,rot) = tf_listener.lookupTransform('map', 'base_link', rospy.Time(0))
            #rospy.loginfo(trans)
           
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.loginfo('Failed %s', e)
            return

        curr_pos = [round(trans[0],2), round(trans[1],2), round(trans[2],2)] # last one is height
        rospy.logerr(f'got position of rosbulla {curr_pos}')
        current_pos = PoseStamped()
        current_pos.header.frame_id = 'map'
        current_pos.pose.position.x = curr_pos[0]
        current_pos.pose.position.y = curr_pos[1]
        current_pos.pose.orientation.w = curr_pos[2]
        


        coords = pd.read_csv('~/Documents/position_data.csv', delimiter=',', header=None)

        rospy.loginfo(coords.head())



        # get second column of coords
        #rospy.loginfo(coords.iloc[:, 1])
        weight_map = {}
        for index, row in coords.iterrows():
            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.pose.position.x = row[1]
            goal.pose.position.y = row[2]
            goal.pose.orientation.w = row[3]
            weight = self.get_abs_difference(current_pos, goal)
            weight_map[int(row[0])] = weight

        # sort by weight
        weight_map = {k: v for k, v in sorted(weight_map.items(), key=lambda item: item[1])}

        #new_coords = coords.copy(deep=True)
        new_coords = pd.DataFrame()

        # iterate keys in weight map and change csv order
        for i, key in enumerate(weight_map.keys()):
            rospy.loginfo(f'setting postit nr {key} to priority nr {i+1} because of weight_map difference:{weight_map[key]} ')
            # get row from key
            row = coords.loc[coords[0] == key]
            new_coords = pd.concat([new_coords, row], axis=0)

        rospy.loginfo(new_coords.head())
        rospy.loginfo("saved nsv!")
        new_coords.to_csv("~/Documents/position_data.csv", index=False, header=False)

        self.planner.close()


if __name__ == '__main__':
    try:
        rospy.init_node('path_planner_node')
        # WTF is this shit and why is it working
        global tf_listener
        tf_listener = tf.TransformListener()
        rospy.loginfo("HALLO NINO")
        time.sleep(2)
        PathPlanner().plan()
    except rospy.ROSInterruptException:
        pass