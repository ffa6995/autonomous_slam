import rospy
from sensor_msgs.msg import PointCloud
from nav_msgs.msg import OccupancyGrid

class Explorer:
    def __init__(self):
        self.mapSub = rospy.Subscribe("/map", 1, self.mapCallback)
        self.frontier_cloud = PointCloud()
        self.frontier_cloud.header.frame_id = "map"

    def mapCallback(self, map):
        resolution = map.info.resolution
        map_x = map.info.origin.position.x / resolution
        map_y = map.info.origin.position.y / resolution
        x = 0. - map_x
        y = 0. - map_y
        frontiers = wtf.wfd(map, map.info.height, map.info.width, x + (y * map.info.width))
        num_points = 0
        for i in range(frontiers.size()):
            for j in range(frontiers[i].size()):
                num_points += 1

        self.frontier_cloud.points = [0]*num_points
        pointI = 0
        for i in range(frontiers.size()):
            for j in range(frontiers[i].size()):
                self.frontier_cloud.points[pointI].x = ((frontiers[i][j] % map.info.width) + map_x) * resolution
                self.frontier_cloud.points[pointI].y = ((frontiers[i][j] / map.info.width) + map_y) * resolution
                self.frontier_cloud.points[pointI].z = 0
                pointI += 1

        self.frontier_pub.publish(self.frontier_cloud)
        
    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()
            
if __name__ == '__main__':
    rospy.init_node('explorer', anonymous=True)
    explorer = Explorer()
    explorer.spin()
    rospy.spin()
