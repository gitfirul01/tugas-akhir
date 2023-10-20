from time import time
from math import cos, sin, atan2, radians, degrees

class EMA:
    def __init__(self, alpha, x0 = 0):
        self.prev_time = 0
        self.prev_speed = 0
        self.speed = 0
        self.speed_thresh = 100
        self.first = True

        self.prev_val = x0
        self.prev_ema = x0
        self.curr_ema = x0
        self.alpha = alpha

    def step(self, val=0, ang_avg=False):
        if ang_avg:
            # angle averaging
            cosa = self.alpha*cos(radians(val)) + (1-self.alpha)*cos(radians(self.prev_ema))
            sina = self.alpha*sin(radians(val)) + (1-self.alpha)*sin(radians(self.prev_ema))
            self.curr_ema = degrees(atan2(sina, cosa))
        else:
            self.curr_ema = (self.alpha*val + (1-self.alpha)*self.prev_ema)

        # calculate movement speed
        now = time()
        dt = now - self.prev_time
        self.speed = (self.curr_ema - self.prev_ema)/dt
        # constraint the speed
        if not self.first:
            if self.speed > self.speed_thresh:
                # self.speed = self.speed_thresh
                return self.prev_ema
            if self.speed < -self.speed_thresh:
                # self.speed = -self.speed_thresh
                return self.prev_ema
        else: self.first = False
        # update parameter
        self.curr_ema = self.prev_ema + self.speed*dt
        self.prev_time = now
        self.prev_speed = self.speed

        self.prev_ema = self.curr_ema
        return self.curr_ema
        
        # if ang_avg:
        #     # Angle averaging with wrapping
        #     delta = (val - self.prev_ema + 180) % 360 - 180
        #     self.curr_ema = (self.prev_ema + self.alpha * delta)
        # else:
        #     self.curr_ema = (self.alpha * val + (1 - self.alpha) * self.prev_ema)
        # if not self.first:
        #     # Calculate movement speed
        #     self.speed = (self.curr_ema - self.prev_ema)
        #     # Constraint the EMA value based on self.speed
        #     if self.speed > self.speed_thresh:
        #         self.curr_ema = self.prev_ema + self.speed_thresh
        #     # if self.speed < self.speed_thresh:
        #     #     self.curr_ema = -(self.prev_ema + self.speed_thresh
        # else:
        #     self.first = False
        # self.prev_ema = self.curr_ema
        # return self.curr_ema


# class SMA:
#     def __init__(self, data_len):
#         self.data_len = data_len
#         self.data = []

#     def step(self, val):
#         self.data.append(val)
#         if len(self.data) > self.data_len:
#             self.data.pop(0)
#         return sum(self.data) / len(self.data)


