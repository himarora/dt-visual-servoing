import numpy as np
import dubins
import scipy.optimize as sciopt
from math import pi, isnan

class LanePlanner:
    """
    

    Args:
        

    """
    def __init__(self, parameters):
        self.params = parameters

    def update_parameters(self, parameters):
        """Updates parameters of LaneController object.

            Args:
                parameters (:obj:`dict`): dictionary containing the new parameters for LaneController object.
        """
        self.params = parameters

    def get_new_path(self, homog_mat):
        """

        Args:
            
        Returns:
            
        """
        # Get maximum radius of curvature
        dt = self.params['~dt_path']
        min_dist = self.params['~min_dist']
        min_angle = self.params['~min_angle']
        num_dt = self.params['~num_dt']
        max_dist_fact = self.params['~max_dist_fact']
        min_r = self.params['~min_r']
        max_r = self.params['~max_r']
        bnds = ((min_r, max_r),)    # Bounds on curvature

        # Get pose in [y, x, theta] form
        x_curr = [0.0, 0.0, 0.0]           # Default position of the bot in the projected frame
        x_targ = self.homog_decompose(homog_mat)
        x_targ[2] = -x_targ[2]              # Flip theta since its measured curr - targ

        dist_min = np.sqrt((x_curr[0]-x_targ[0])**2 + (x_curr[1]-x_targ[1])**2)
        dist_max = max_dist_fact*dist_min

        # If travel distance decent, use dubins path
        if dist_min > min_dist:
            print("getting path")
            con = {'type': 'ineq', 'fun': self.cons_fun, 'args': (x_curr, x_targ, dt, dist_max)}
            res = sciopt.minimize(self.opt_rad, min_r, args=(x_curr, x_targ, dt), bounds=bnds, constraints=con)
            max_r_opt = res.x

            path, u, dist = self.gen_path(x_curr, x_targ, max_r_opt, dt)
            u[1,:] = -u[1,:]
            print("")
            print(max_r_opt)
            print("planned path")
            print(path)
            print("planned input")
            print(u)
            print(x_targ)
            print("")
        elif abs(x_targ[2] - x_curr[2]) > min_angle:
            # If we only want to turn
            # Accomplish turn in 10 time steps
            print("turn only")
            u = np.zeros([2, num_dt])
            u[1,:] = (x_curr[2] + x_targ[2])/(num_dt*dt)        # Plus here since we already flipped -targ above
            dist = dist_min

            # Now add constant forward velocity if no dist info available, meaning we cant see red line
            if isnan(dist_min):
                u[0,:] = dt
            path = np.array([None, None])
        elif isnan(isnan(x_curr[0]) and isnan(x_curr[1])) and not isnan(isnan(x_targ[0]) and isnan(x_targ[1])):
            # This means our target is a line and point but we only see lines right now
            # We should just drive straight
            print("just drive straight")
            u = np.zeros([2, num_dt])
            u[0,:] = dt
            path = np.array([None, None])
            dist = dist_min
        else:
            # If we get here, we are confused and should just continue the previous plan
            print("continuing previous path")
            path = np.array([None, None])
            u = np.array([None, None])
            dist = None

        return path, u, dist

    def gen_path(self, q0, q1, r, dt):
        path = dubins.shortest_path(q0, q1, r)
        qs, _ = path.sample_many(dt)
        qs = np.array(qs)
        qs = np.vstack((qs, q1))

        # Figure out velocity stuff
        omega = np.zeros(qs.shape[0]-1)
        dist = 0.0
        for ii in range(qs.shape[0]-1):
            x_k = qs[ii,0]
            x_k1 = qs[ii+1,0]
            y_k = qs[ii,1]
            y_k1 = qs[ii+1,1]
            t_k = qs[ii,2]
            t_k1 = qs[ii+1,2]

            del_x = x_k1 - x_k
            del_y = y_k1 - y_k
            del_t = t_k1 - t_k

            dist = dist + np.sqrt(del_x**2 + del_y**2)

            omega[ii] = del_t/dt
        v = np.ones(omega.shape)
        u = dt*np.vstack((v, omega))

        return np.transpose(qs), u, dist

    def cons_fun(self, r, q0, q1, dt, dist_max):
        neg_dist = self.opt_rad(r, q0, q1, dt)
        return neg_dist + dist_max

    def opt_rad(self, r, q0, q1, dt):
        path = dubins.shortest_path(q0, q1, r)
        qs, _ = path.sample_many(dt)
        qs = np.array(qs)
        qs = np.vstack((qs, q1))

        # Figure out velocity stuff
        dist = 0.0
        for ii in range(qs.shape[0]-1):
            x_k = qs[ii,0]
            x_k1 = qs[ii+1,0]
            y_k = qs[ii,1]
            y_k1 = qs[ii+1,1]

            del_x = x_k1 - x_k
            del_y = y_k1 - y_k

            dist = dist + np.sqrt(del_x**2 + del_y**2)
        return -dist

    def homog_decompose(self, SE2_el):
        C = SE2_el[0:2,0:2]
        theta = self.SO2_decompose(C)
        r = SE2_el[0:2,2]

        return[r[1], -r[0], theta]

    def SO2_decompose(self, SO2_el):
        theta = np.arccos(SO2_el[0, 0])
        if abs(theta) < 0.001:
            theta = 0.0
        elif abs(pi - theta) < 0.001:
            theta = pi
        elif np.arcsin(SO2_el[1, 0]) < 0:
            theta = -theta
        return theta