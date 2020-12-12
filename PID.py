from gym_duckietown.tasks.task_solution import TaskSolution
import numpy as np


class PIDController:
    def __init__(self, K_p, K_i, K_d):
        self.K_p = K_p
        self.K_i = K_i
        self.K_d = K_d
        self.p = 0
        self.i = 0
        self.d = 0
        self.cte_t_1 = 0

    def __call__(self, cte_t):
        self.p = cte_t
        self.d = cte_t - self.cte_t_1
        self.i += cte_t
        self.cte_t_1 = cte_t

        return -(self.K_p * self.p + self.K_i * self.i + self.K_d * self.d)


class LfChallengeNoCvTaskSolution(TaskSolution):
    def __init__(self, generated_task):
        super().__init__(generated_task)
        self.road_center = 0
        self.straight = 0
        self.dist_pid = PIDController(25, 1e-7, 0.1)
        self.angle_pid = PIDController(25, 1e-7, 0.1) 

    def solve(self):
        env = self.generated_task['env']
        while True:
            lane_pose = env.get_lane_pos2(env.cur_pos, env.cur_angle)
            distance_to_road_center = lane_pose.dist
            angle_from_straight_in_rads = lane_pose.angle_rad

            # Требуется по положению робота в полосе определить линейную и угловые скорости
            dist_cte = self.road_center - distance_to_road_center
            dist_error = self.dist_pid(dist_cte)

            angle_cte = self.straight - angle_from_straight_in_rads
            angle_error = self.angle_pid(angle_cte)

            error = angle_error + dist_error
            steering = np.clip(error, -1, 1)

            speed = 1 / (5 * np.abs(steering))
            speed = np.clip(speed, 0, 1)
            print(f'angle error {angle_error}')
            print(f'dist error {dist_error}')
            print(f'speed {speed}')
            print(f'steering {steering}')
            print('----------')

            env.step([speed, steering])
            env.render()
