import sys
from multiprocessing import Queue
import time
import Adafruit_PCA9685

def x_angle2duty(angle):
    plus = True if angle > 0 else False
    duty = int( float(angle) * 2.17 + 102 )
    # reverse
    # duty = 594 - duty
    print('x reset angle is', f' {angle}' 'duty is' f' {duty}')
    return duty if plus==True else -duty

def y_angle2duty(angle):
    plus = True if angle > 0 else False
    duty = int(float(angle) * 2.17 + 102 )
    print('y reset angle is', f' {angle}' 'duty is' f' {duty}')
    return duty if plus==True else -duty


def hw_thread(q:Queue, opt):
    xaxis_pin = opt.xaxis_pin
    yaxis_pin = opt.yaxis_pin
    pwm = Adafruit_PCA9685.PCA9685()
    # Set frequency to 60hz, good for servos.
    pwm.set_pwm_freq(opt.pwm_freq)
    x_defalut = x_angle2duty(angle=opt.x_defalut)
    pwm.set_pwm(xaxis_pin, 0, x_defalut)
    time.sleep(0.5)
    y_defalut = y_angle2duty(angle=opt.y_defalut)
    pwm.set_pwm(yaxis_pin, 0, y_defalut)
    time.sleep(0.5)
    try:
        while True:
            duty = q.get()
            axis, angle = duty
            if axis=='x':
                duty = x_angle2duty(angle)
                pwm.set_pwm(xaxis_pin, 0, duty)
                #time.sleep(0.1)
            elif axis=='y':
                duty = y_angle2duty(angle)
                pwm.set_pwm(yaxis_pin, 0, duty)
                #time.sleep(0.1)
    finally:
        pwm.set_pwm(xaxis_pin, 0, x_defalut)
        time.sleep(1)
        pwm.set_pwm(yaxis_pin, 0, y_defalut)
        time.sleep(1)
        sys.exit()

#if __name__ == "__main__":
  #  q = Queue()
  #  hw_thread(q, opt=None)
    
