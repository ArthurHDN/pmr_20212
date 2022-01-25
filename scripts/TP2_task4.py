#!/usr/bin/env python
import rospy
from tf.transformations import euler_from_quaternion

# ros-msgs
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Twist, PointStamped

# python
import matplotlib.image as img
import matplotlib.pyplot as plt
import numpy as np
import random
from math import pi, tan, cos, sin



class RRT():

    class Node():
        def __init__(self, x, y, theta):
            self.x = x
            self.y = y
            self.theta = theta
            self.path_x = []
            self.path_y = []
            self.parent = None

    def __init__(self, start, goal, obstacle_list, step_size=2.0, dt=0.1, max_iterations=500):
        self.start = self.Node(start[0], start[1], start[2])
        self.end_point = self.Node(goal[0], goal[1], 0.0)
        self.min_rand = -50
        self.max_rand = 50
        self.step_size = 2.0
        self.dt = dt
        self.max_iterations = max_iterations
        self.obstacle_list = obstacle_list
        self.node_list = []
        self.q = [0.0, 0.0, 0.0]
        self.goal = ()
        self.map = []

        self.d = 0.2
        self.k = 5

        # robot velocity for nonholonomic paths
        self.uV = 1.0
        self.uW = [pi/-6.0, pi/-12.0, 0.0, pi/6.0, pi/12.0]

    def dist(self, p1, p2): 
        return ((p1[0]-p2[0])**2 +(p1[1]-p2[1])**2)**(0.5)

    def callback_pose(self, data):
        self.q[0] = data.pose.pose.position.x  # robot pos x
        self.q[1] = data.pose.pose.position.y  # robot pos y

        x_q = data.pose.pose.orientation.x
        y_q = data.pose.pose.orientation.y
        z_q = data.pose.pose.orientation.z
        w_q = data.pose.pose.orientation.w
        euler = euler_from_quaternion([x_q, y_q, z_q, w_q])

        self.q[2] = euler[2]  # robot orientation

    def callback_goalPoint(self, data):
        self.goal = ((data.point.x),(data.point.y))



    def callback_map(self, msg):
        self.map = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)

    def expand_randomly(self):
        rnd = self.Node(random.uniform(self.min_rand, self.max_rand), random.uniform(self.min_rand, self.max_rand), 0.0)
        return rnd

    def find_qnear(self,q_rnd):
        distances_list = [(q.x - q_rnd.x)**2 + (q.y - q_rnd.y)**2 for q in self.node_list]
        min_index = distances_list.index(min(distances_list))

        return self.node_list[min_index]

    def add_to_rrt(self,q_rand):
        #print('addin to rrt')
        q_near = self.find_qnear(q_rand)
        #print('q_near found')
        q_new = self.step(q_near, q_rand)

        #print(self.check_collision(q_new))

        if self.check_collision(q_new):
            self.node_list.append(q_new)
            return q_new
        else:
            return None

    def trajectory(self,xi,yi,thetai,ori_vec):
        (x,y,theta)=([],[],[])
        x.append(xi)
        y.append(yi)
        theta.append(thetai)
        p = self.step_size/self.dt
        for i in range(1,int(p)):
            theta.append(theta[i-1]+(self.uV*tan(ori_vec))*self.dt)
            x.append(x[i-1]+self.uV*cos(theta[i-1])*self.dt)
            y.append(y[i-1]+self.uV*sin(theta[i-1])*self.dt)    

        return (x,y,theta)

    def step(self,q1,q2):
        #print('in step function')

        xr=[]
        yr=[]
        thetar=[]
        # 
        for j in self.uW:
            #print('step 1:' + str(j))
            (x,y,theta)=self.trajectory(q1.x,q1.y,q1.theta,j)
            xr.append(x)
            yr.append(y)
            thetar.append(theta)
                
        dmin = self.dist([q2.x,q2.y],[xr[0][-1],yr[0][-1]])
        near = 0
        for i in range(1,len(xr)):
            #print('step 2:' + str(i))
            d = self.dist([q2.x,q2.y],[xr[i][-1],yr[i][-1]])
            if d < dmin:
                dmin= d
                near = i

        q_new = self.Node(xr[near][-1],yr[near][-1],thetar[near][-1])
        q_new.parent = q1
        q_new.path_x = xr[near]
        q_new.path_y = yr[near]
        #print(q_new)

        return q_new

    def check_collision(self, q):
        #print('checking for collision')
        #obstacleList = self.obstacle_list

        if q is None:
            #print('q is none')
            return False
        
        #i = 0
        for (cx, cy, size) in self.obstacle_list:
            #print(i)
            #i += 1
            dx_list = [cx - x for x in q.path_x]
            dy_list = [cy - y for y in q.path_y]
            d_list = [dx * dx + dy * dy for (dx, dy) in zip(dx_list, dy_list)]

            if min(d_list) <= 3*(size**2):
                #print('collision found')
                return False  # collision

        return True  #not_collision

    def find_path(self, goal_ind):
        path = [[self.end_point.x, self.end_point.y]]
        node = self.node_list[goal_ind]
        while node.parent is not None:
            n = len(node.path_x) - 1
            while n >=0:
                path.append([node.path_x[n],node.path_y[n]])
                n -= 1
            node = node.parent
        path.append([node.x, node.y])

        return path

    def planner(self):
        self.node_list = [self.start]
        max_iterations = 10000
        for i in range(max_iterations):
            print(i)
            q_rnd = self.expand_randomly()
            #print(q_rnd.x)
            #print(q_rnd.y)
            q_new = self.add_to_rrt(q_rnd)
            #print(q_new)

            if (self.dist([self.node_list[-1].x, self.node_list[-1].y],[self.end_point.x,self.end_point.y]) <= self.step_size):
                print(self.step_size)
                print(self.dist([self.node_list[-1].x, self.node_list[-1].y],[self.end_point.x,self.end_point.y]))
                q_final = self.step(self.node_list[-1], self.end_point)
                if self.check_collision(q_final):
                    print(self.check_collision(q_final))
                    path = self.find_path(len(self.node_list) - 1)
                    return path

        #path = self.find_path(len(self.node_list) - 1)
        return None

    def control(self,q_next, q_now):

        Ux = self.k * (q_next[0] - q_now[0])
        Uy = self.k * (q_next[1] - q_now[1])

        return self.feedback_linearization(Ux,Uy,q_now[2])

    def feedback_linearization(self,Ux, Uy, theta_n):

        vx = cos(theta_n) * Ux + sin(theta_n) * Uy
        w = -(sin(theta_n) * Ux)/ self.d  + (cos(theta_n) * Uy) / self.d 

        return vx, w

    def draw_graph(self):
        print('drawing')
        plt.clf()
        traj = []
        for node in self.node_list:
            if node.parent:
                plt.plot(node.path_x, node.path_y, "-g")
                traj.append([node.path_x, node.path_y])

        # HIGH LOADING - can be removed if needed 
        for (ox, oy, size) in self.obstacle_list:
            self.plot_obstacles(ox, oy, size)

        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.end_point .x, self.end_point .y, "xr")
        plt.axis("equal")
        plt.axis([-50, 50, -50, 50])
        plt.grid(True)

    def plot_obstacles(self, x, y, size, color="-k"):
        deg = list(range(0, 360, 5))
        deg.append(0)
        xl = [x + size * cos(np.deg2rad(d)) for d in deg]
        yl = [y + size * sin(np.deg2rad(d)) for d in deg]
        plt.plot(xl, yl, color)

    

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


def main(goal_config):

    rospy.init_node("rrt_planner", anonymous=True)

    pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    rate = rospy.Rate(2)

    M, obstacle_list = map_reader('map_rrt.bmp', 50, 1)
    #print(obstacle_list)

    max_iterations = 1000

    starting_position = (0.0, 0.0, 0.0)
    rrt_planner = RRT(starting_position, goal_config, obstacle_list, max_iterations)
    #rrt_planner.obstacle_list = obstacle_list
    vel_msg = Twist()

    rospy.Subscriber('/base_pose_ground_truth', Odometry, rrt_planner.callback_pose)
    rospy.Subscriber('/clicked_point', PointStamped, rrt_planner.callback_goalPoint)
    rospy.Subscriber('/map', OccupancyGrid, rrt_planner.callback_map)
    
    while not rospy.is_shutdown():
        if goal_config and obstacle_list:

            print("Trying to find a path...")
            path = rrt_planner.planner()

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
                print(len(new_traj))
                plt.axis([-65, 65, -65, 65])
                plt.savefig('/workspaces/arthur_ws/src/pmr_20212/rrt_plot_2.png')

                for i in range(len(new_traj)):
                    D = 1000
                    while(D > 0.1 and not rospy.is_shutdown()):
                        D = rrt_planner.dist([new_traj[i,0],new_traj[i,1]],[rrt_planner.q[0],rrt_planner.q[1]])

                        vel_msg.linear.x, vel_msg.angular.z = rrt_planner.control([new_traj[i,0],new_traj[i,1]],rrt_planner.q)
                        pub_cmd_vel.publish(vel_msg)

    	rate.sleep()


if __name__ == '__main__':
    try:
        x_goal = input('x_goal = '); y_goal = input('y_goal = ')
        main((x_goal, y_goal))
    except rospy.ROSInterruptException:
        pass
