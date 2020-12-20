from collections import OrderedDict
from scipy.stats import multivariate_normal, norm
import numpy as np
from scipy.ndimage.filters import gaussian_filter
from math import floor, sqrt, pi
from scipy.linalg import expm


class LaneFilterHistogramKF():
    """ Generates an estimate of the lane pose.

    TODO: Fill in the details

    Args:
        configuration (:obj:`List`): A list of the parameters for the filter

    """

    def __init__(self, **kwargs):
        param_names = [
            # TODO all the parameters in the default.yaml should be listed here.
            'mean_d_0',
            'mean_phi_0',
            'sigma_d_0',
            'sigma_phi_0',
            'delta_d',
            'delta_phi',
            'd_max',
            'd_min',
            'phi_max',
            'phi_min',
            'cov_v',
            'linewidth_white',
            'linewidth_yellow',
            'lanewidth',
            'min_max',
            'sigma_d_mask',
            'sigma_phi_mask',
            'range_min',
            'range_est',
            'range_max',
        ]

        for p_name in param_names:
            assert p_name in kwargs
            setattr(self, p_name, kwargs[p_name])

        self.encoder_resolution = 0
        self.wheel_radius = 0.0
        self.baseline = 0.0
        self.initialized = False
        self.reset()

    def reset(self):
        self.mean_0 = [self.mean_d_0, self.mean_phi_0]
        self.cov_0 = [[self.sigma_d_0, 0], [0, self.sigma_phi_0]]

        # Initialize SE2 element
        SO2_0 = self.SO2_synthesise(self.mean_phi_0)
        self.T = np.zeros((3, 3))
        self.T[0:2, 0:2] = SO2_0
        self.T[0, 2] = self.mean_d_0
        self.T[2, 2] = 1
        self.P_hat = np.array([[self.sigma_phi_0, 0, 0], [0, self.sigma_d_0, 0], [0, 0, self.sigma_d_0]])

        self.belief = {'mean': self.mean_0, 'covariance': self.cov_0}
        self.pose = {'pose mean': [0, 0, 0]}
        print(self.pose)
        np.set_printoptions(suppress=True)

    def predict(self, dt, left_encoder_delta, right_encoder_delta):
        if not self.initialized:
            return
        # Load in current state
        T_hat = self.T
        P_hat = self.P_hat

        # Get wheel velocities and angular velocity from encoders through Euler discretization
        # Update frequency is 30 Hz so we should be more than fine not doing anything fancier
        d_left = left_encoder_delta * 2 * pi * self.wheel_radius / self.encoder_resolution
        d_right = right_encoder_delta * 2 * pi * self.wheel_radius / self.encoder_resolution
        v_left = d_left / dt
        v_right = d_right / dt
        v_forward = (v_left + v_right) / 2
        omega = (v_right - v_left) / self.baseline

        # Get noise for velocities
        # Cap encoder delta to be 1 for noise calculations
        if left_encoder_delta < 1.0: left_encoder_delta = 1.0
        if right_encoder_delta < 1.0: right_encoder_delta = 1.0
        w_enc = 1 / self.encoder_resolution
        w_v_l = v_left * sqrt((w_enc / left_encoder_delta) ** 2)
        w_v_r = v_right * sqrt((w_enc / right_encoder_delta) ** 2)
        w_v_f = 10 * sqrt(w_v_l ** 2 + w_v_r ** 2)  # Factor these up to allow for slip + uncertainty during turns
        w_ome = 12 * sqrt(w_v_l ** 2 + w_v_r ** 2)

        # Define velocity vector, note we cannot move sideways in body frame so we set that as 0
        u_b = np.expand_dims(np.array([omega, v_forward, 0.0]), axis=1)

        # Predict next T
        Xi = np.zeros((3, 3))
        Xi[0:2, 0:2] = self.so2_exp(self.so2_wedge(dt * u_b[0, 0]))
        Xi[0:2, 2] = dt * u_b[1:3, 0].T
        Xi[2, 2] = 1
        T_check = T_hat @ Xi

        # Get linearized continuous time matrices
        A = -self.SE2_ad(u_b[:, 0])
        L = -np.eye(3)
        Q = np.array([[w_ome, 0, 0], [0, w_v_f, 0], [0, 0, 0]])

        # Use Van Loan's method to get discrete time A, Q
        Xi_VL = np.zeros((10, 10))
        Xi_VL[0:3, 0:3] = A
        Xi_VL[0:3, 3:6] = L @ Q @ L.T
        Xi_VL[3:6, 3:6] = -1 * A.T
        Xi_VL[6:9, 6:9] = A

        Y = expm(dt * Xi_VL)
        A_d = Y[0:3, 0:3]
        Q_d = Y[0:3, 3:6] @ Y[0:3, 0:3].T

        # Update P_hat (even though this is P_check technically)
        P_check = A_d @ P_hat @ A_d.T + L @ Q_d @ L.T
        P_check = 0.5 * (P_check + P_check.T)  # Enforce symmetry

        # Make updates
        self.T = T_check
        self.P_hat = P_check
        self.belief = self.SE2_to_belief(T_check)
        self.pose = self.SE2_to_pose(T_check)

    def update(self, segments):
        # Load in current state
        T_check = self.T
        P_check = self.P_hat

        # prepare the segments for each belief array
        segmentsArray = self.prepareSegments(segments)
        # generate all belief arrays

        measurement_likelihood = self.generate_measurement_likelihood(
            segmentsArray)
        ml_edit = np.copy(measurement_likelihood)

        # Get the number of unique likelihoods
        (unique, counts) = np.unique(ml_edit, return_counts=True)

        # Make sure that we actually have data
        if unique[0] != None:
            # Get rid of zero probability entries
            if np.min(unique) == 0.0:
                unique = unique[1:]
                counts = counts[1:]

            # Check and get rid of baseline noise
            sorted_counts = np.sort(counts)
            if len(counts) > 1:
                # This checks if the mode likelihood is widespread compared to other likelihoods
                if np.sort(counts)[-1] > np.sort(counts)[-2] * 4 and np.sort(counts)[-1] > 5:
                    mode_idx = np.argmax(counts)  # Get index of mode likelihood
                    ml_edit = ml_edit - unique[mode_idx]  # Subtract mode likelihood
                    ml_edit[ml_edit < 0.0] = 0.0  # Clamp likelihood to be >= 0.0
                    if np.sum(ml_edit) != 0.0:
                        ml_edit = ml_edit / np.sum(ml_edit)  # Renormalize likelihoods

            # Now generate discrete d and phi probabilities
            d_vals = np.mgrid[self.d_min:self.d_max:self.delta_d]
            d_probs = ml_edit.sum(axis=1)
            phi_vals = np.mgrid[self.phi_min:self.phi_max:self.delta_phi]
            phi_prob = ml_edit.sum(axis=0)

            # Finally, calculate mean and variance from the useful buckets
            d_mean = np.sum(np.multiply(d_vals, d_probs))
            d_var = np.sum(np.multiply(np.multiply(d_vals, d_vals), d_probs)) - d_mean ** 2
            phi_mean = np.sum(np.multiply(phi_vals, phi_prob))
            phi_var = np.sum(np.multiply(np.multiply(phi_vals, phi_vals), phi_prob)) - phi_mean ** 2

            # Clamp variance to be at least a little noisy
            d_var = max(d_var, self.delta_d / 6.0)
            phi_var = max(phi_var, self.delta_phi / 6.0)

            # First process phi measurements
            Y_k = self.SO2_synthesise(phi_mean)  # Get measured prediction
            Y_k_inv = np.linalg.inv(Y_k)
            Y_check = self.T[0:2, 0:2]  # Get predicted measurement

            # Define innovation for phi measurements
            z_phi = self.so2_decompose(self.SO2_log(Y_k_inv @ Y_check))
            H_1 = np.array([1, 0, 0])
            M_1 = np.array([-1, 0, 0])

            # Now process position measurements
            # Measured position is a bit weird since we're only measuring the lateral deviation..
            # Additionally, the "xy" plane moves with the lane so r1 has no actual meaning...
            # Lets try just snapping it back to 0...
            # r1_meas = 0.0

            # Alternatively we can keep it as is from wheel encoder
            # r1_meas = T_check[0,2]

            # Alternatively maybe we can try to keep it as our "amount travelled forward" through some math
            # This is also super weird since we are now providing r1 in the body frame instead of the world frame
            # But this might be a more "natural" way to transition the position through lane turns
            r1_meas = (Y_k @ T_check[0:2, 2])[0]

            y_k = np.expand_dims(np.array([r1_meas, d_mean, 1.0]), axis=1)  # Get measured prediction
            y_check = np.expand_dims(T_check[0:3, 2], axis=1)  # Get predicted measurement

            # Define innovation for d
            z_r = np.linalg.inv(T_check) @ (y_k - y_check)
            H_2 = np.array([[0, -1, 0], [0, 0, -1]])
            M_2 = np.zeros((2, 3))
            M_2[0:2, 1:3] = T_check[0:2, 0:2].T

            # Get full innovation
            z = np.zeros((3, 1))
            z[0, 0] = z_phi
            z[1:3, 0] = z_r[0:2, 0]

            # Calculate IEKF matrices
            H = np.vstack((H_1, H_2))
            M = np.vstack((M_1, M_2))
            R = np.diag([phi_var, d_var, d_var])

            K = P_check @ H.T @ np.linalg.inv(H @ P_check @ H.T + M @ R @ M.T)
            P_hat = (np.eye(3) - K @ H) @ P_check @ (np.eye(3) - K @ H).T + K @ M @ R @ M.T @ K.T
            P_hat = 0.5 * (P_hat + P_hat.T)  # Enforce symmetry

            T_hat = T_check @ self.SE2_exp(-self.se2_wedge(K @ z))

            # Make updates
            self.P_hat = P_hat
            self.T = T_hat
            self.belief = self.SE2_to_belief(T_hat)

    def getEstimate(self):
        return self.belief

    def getPoseEst(self):
        return self.pose

    def SE2_to_belief(self, SE2_el):
        # Convert SE2 to d and phi estimate
        # d should just be our r_a^2 value
        new_phi = self.SO2_decompose(self.T[0:2, 0:2])
        new_d = self.T[1, 2]

        new_mean = [new_d, new_phi]
        new_cov = [[self.P_hat[2, 2], self.P_hat[2, 0]], [self.P_hat[0, 2], self.P_hat[0, 0]]]

        return {'mean': new_mean, 'covariance': new_cov}

    def SE2_to_pose(self, SE2_el):
        # Convert SE2 to vector of [theta, x, y]
        new_x = self.T[0, 2]
        new_y = self.T[1, 2]
        new_phi = self.SO2_decompose(self.T[0:2, 0:2])

        new_mean = [new_phi, new_x, new_y]
        new_cov = self.P_hat

        return {'pose mean': new_mean}

    def SE2_exp(self, se2_el):
        xi_theta, xi_r = self.se2_decompose(se2_el)
        C = self.SO2_synthesise(xi_theta)
        J = self.se2_J(xi_theta)
        r = J @ xi_r

        SE2_el = np.zeros((3, 3))
        SE2_el[0:2, 0:2] = C
        SE2_el[0:2, 2] = r
        SE2_el[2, 2] = 1
        return SE2_el

    def se2_decompose(self, se2_el):
        xi_theta = se2_el[1, 0]
        xi_r = se2_el[0:2, 2]
        return xi_theta, xi_r

    def se2_wedge(self, se2_col):
        xi_theta = se2_col[0, 0]
        xi_r = se2_col[1:3, 0]
        se2_el = np.zeros((3, 3))
        se2_el[0:2, 0:2] = self.so2_wedge(xi_theta)
        se2_el[0:2, 2] = xi_r
        return se2_el

    def se2_J(self, xi_theta):
        # If xi_theta small use TSE
        if abs(xi_theta) < 0.001:
            t2 = xi_theta ** 2
            J11 = 1 - t2 / 6 * (1 - t2 / 20 * (1 - t2 / 42))
            J12 = -xi_theta * 0.5 * (1 - t2 / 12 * (1 - t2 / 30 * (1 - t2 / 56)))
        else:
            J11 = np.sin(xi_theta) / xi_theta
            J12 = -(1 - np.cos(xi_theta)) / xi_theta

        J21 = -J12
        J22 = J11
        J = np.array([[J11, J12], [J21, J22]])
        return J

    def SO2_log(self, SO2_el):
        return self.so2_wedge(self.SO2_decompose(SO2_el))

    def SE2_ad(self, u):
        return np.array([[0, 0, 0], [u[2], 0, -u[0]], [-u[1], u[0], 0]])

    def SO2_synthesise(self, theta):
        return np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])

    def SO2_decompose(self, SO2_el):
        theta = np.arccos(SO2_el[0, 0])
        if abs(theta) < 0.001:
            theta = 0
        elif abs(pi - theta) < 0.001:
            theta = pi
        elif np.arcsin(SO2_el[1, 0]) < 0:
            theta = -theta
        return theta

    def so2_wedge(self, theta):
        return np.array([[0, -theta], [theta, 0]])

    def so2_decompose(self, so2_el):
        return so2_el[1, 0]

    def so2_exp(self, so2_el):
        theta = self.so2_decompose(so2_el)
        return self.SO2_synthesise(theta)

    def generate_measurement_likelihood(self, segments):

        if len(segments) == 0:
            return None

        grid = np.mgrid[self.d_min:self.d_max:self.delta_d,
               self.phi_min:self.phi_max:self.delta_phi]

        # initialize measurement likelihood to all zeros
        measurement_likelihood = np.zeros(grid[0].shape)

        for segment in segments:
            d_i, phi_i, l_i, weight = self.generateVote(segment)

            # if the vote lands outside of the histogram discard it
            if d_i > self.d_max or d_i < self.d_min or phi_i < self.phi_min or phi_i > self.phi_max:
                continue

            i = int(floor((d_i - self.d_min) / self.delta_d))
            j = int(floor((phi_i - self.phi_min) / self.delta_phi))
            measurement_likelihood[i, j] = measurement_likelihood[i, j] + 1

        if np.linalg.norm(measurement_likelihood) == 0:
            return None

        # lastly normalize so that we have a valid probability density function

        measurement_likelihood = measurement_likelihood / \
                                 np.sum(measurement_likelihood)
        return measurement_likelihood

    # generate a vote for one segment
    def generateVote(self, segment):
        p1 = np.array([segment.points[0].x, segment.points[0].y])
        p2 = np.array([segment.points[1].x, segment.points[1].y])
        t_hat = (p2 - p1) / np.linalg.norm(p2 - p1)

        n_hat = np.array([-t_hat[1], t_hat[0]])
        d1 = np.inner(n_hat, p1)
        d2 = np.inner(n_hat, p2)
        l1 = np.inner(t_hat, p1)
        l2 = np.inner(t_hat, p2)
        if (l1 < 0):
            l1 = -l1
        if (l2 < 0):
            l2 = -l2

        l_i = (l1 + l2) / 2
        d_i = (d1 + d2) / 2
        phi_i = np.arcsin(t_hat[1])
        if segment.color == segment.WHITE:  # right lane is white
            if (p1[0] > p2[0]):  # right edge of white lane
                d_i = d_i - self.linewidth_white
            else:  # left edge of white lane

                d_i = - d_i

                phi_i = -phi_i
            d_i = d_i - self.lanewidth / 2

        elif segment.color == segment.YELLOW:  # left lane is yellow
            if (p2[0] > p1[0]):  # left edge of yellow lane
                d_i = d_i - self.linewidth_yellow
                phi_i = -phi_i
            else:  # right edge of white lane
                d_i = -d_i
            d_i = self.lanewidth / 2 - d_i

        # weight = distance
        weight = 1
        return d_i, phi_i, l_i, weight

    def get_inlier_segments(self, segments, d_max, phi_max):
        inlier_segments = []
        for segment in segments:
            d_s, phi_s, l, w = self.generateVote(segment)
            if abs(d_s - d_max) < 3 * self.delta_d and abs(phi_s - phi_max) < 3 * self.delta_phi:
                inlier_segments.append(segment)
        return inlier_segments

    # get the distance from the center of the Duckiebot to the center point of a segment
    def getSegmentDistance(self, segment):
        x_c = (segment.points[0].x + segment.points[1].x) / 2
        y_c = (segment.points[0].y + segment.points[1].y) / 2
        return sqrt(x_c ** 2 + y_c ** 2)

    # prepare the segments for the creation of the belief arrays
    def prepareSegments(self, segments):
        segmentsArray = []
        self.filtered_segments = []
        for segment in segments:

            # we don't care about RED ones for now
            if segment.color != segment.WHITE and segment.color != segment.YELLOW:
                continue
            # filter out any segments that are behind us
            if segment.points[0].x < 0 or segment.points[1].x < 0:
                continue

            self.filtered_segments.append(segment)
            # only consider points in a certain range from the Duckiebot for the position estimation
            point_range = self.getSegmentDistance(segment)
            if point_range < self.range_est:
                segmentsArray.append(segment)

        return segmentsArray
