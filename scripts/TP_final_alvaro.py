#!/usr/bin/env python

from time import sleep
import rospy

import numpy as np
import matplotlib.image as img
import matplotlib.pyplot as plt

# ros-msgs
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Twist, PointStamped

try:
    from pmr_20212.rrt_star import RRTStar
except ImportError:
    raise

####################################################################################################################################

def main(goal_config):

    rospy.init_node("rrt_planner", anonymous=True)

    pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    rate = rospy.Rate(1)

    M, obstacle_list = map_reader('map_rrt.bmp', 50, 1)
    #print(obstacle_list)

    max_iterations = 20

    starting_position = (0.0, 0.0, 0.0)
    rrt_planner = RRTStar(starting_position, goal_config, obstacle_list, [-50, 50])
    #rrt_planner.obstacle_list = obstacle_list
    vel_msg = Twist()

    rospy.Subscriber('/base_pose_ground_truth', Odometry, rrt_planner.callback_pose)
    rospy.Subscriber('/clicked_point', PointStamped, rrt_planner.callback_goalPoint)
    rospy.Subscriber('/map', OccupancyGrid, rrt_planner.callback_map)
    
    while not rospy.is_shutdown():
        if goal_config and obstacle_list:

            print("Trying to find a path...")
            path = rrt_planner.planning()

            if path is None:
                print("Cannot find path")
            else:
                print("Path found")

                new_traj = np.zeros((len(path),2))
                j = 0
                for i in range(len(path)-1,-1,-1):
                    new_traj[j,0] = path[i][0]
                    new_traj[j,1] = path[i][1]
                    j+=1
                print(new_traj)

                rrt_planner.draw_graph()
                plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
                plt.grid(True)
                plt.xlabel('X (m)')
                plt.ylabel('Y (m)')
                #print(len(new_traj))
                plt.axis([-65, 65, -65, 65])
                plt.savefig('/workspaces/arthur_ws/src/pmr_20212/rrt_star.png')

                for i in range(len(new_traj)):
                    D = 1000
                    while(D > 0.1 and not rospy.is_shutdown()):
                        D = rrt_planner.dist([new_traj[i,0],new_traj[i,1]],[rrt_planner.q[0],rrt_planner.q[1]])

                        vel_msg.linear.x, vel_msg.angular.z = rrt_planner.control([new_traj[i,0],new_traj[i,1]],rrt_planner.q)
                        pub_cmd_vel.publish(vel_msg)

            sleep(30)

    	rate.sleep()


def map_reader(map_name, scale, resolution):
    image_path = '/workspaces/arthur_ws/src/pmr_20212/worlds/' + map_name
    image = img.imread(image_path)
    image.setflags(write=1)

    obs = []

    M = np.zeros((len(image),len(image)))
    for i in range(len(image)):
        for j in range(len(image)):
            if(image[i,j,0] == 255 and image[i,j,1] == 255 and image[i,j,2] == 255):
                M[i,j] = 0
            else:
                M[i,j] = 1
                obs.append([float(j*resolution)-scale, -float(i*resolution)+scale,  0.3])
                obs.append([float(j*resolution)-scale+0.15, -float(i*resolution)+scale+0.15,  0.3])                    
                obs.append([float(j*resolution)-scale-0.15, -float(i*resolution)+scale-0.15,  0.3])

    return M, obs



#########################################################################################################################################

if __name__ == '__main__':
    try:
        x_goal = input('x_goal = '); y_goal = input('y_goal = ')
        main((x_goal, y_goal))
    except rospy.ROSInterruptException:
        pass
