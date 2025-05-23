import time
import Adafruit_PCA9685
from tkinter import Frame, Scale, HORIZONTAL, VERTICAL

class App:
    def __init__(self, master, opt):
        self.yaxis_pin = opt.yaxis_pin
        self.xaxis_pin = opt.xaxis_pin
        self.pwm = Adafruit_PCA9685.PCA9685()
        self.pwm.set_pwm_freq(opt.pwm_freq)
        frame = Frame(master)
        frame.pack()
        scale_pan = Scale(frame, label="yangle", from_=0, to=180, tickinterval=90, orient=HORIZONTAL, command=self.update_yaxis_pan)
        scale_pan.set(90)
        scale_pan.grid(row=0, column=0)
        scale_tilt = Scale(frame, label="xangle", from_=0, to=180, tickinterval=90, orient=VERTICAL, command=self.update_xais_tilt)
        scale_tilt.set(90)
        scale_tilt.grid(row=0, column=1)

    def update_yaxis_pan(self, angle):
        duty = int( float(angle) * 2.17 + 102 )
        print('y-axis, angle is', f' {angle}' 'duty is' f' {duty}')
        self.pwm.set_pwm(self.yaxis_pin, 0, duty)
        time.sleep(0.1)

    def update_xais_tilt(self, angle):
        duty = int( float(angle) * 2.17 + 102)
        # reverse
        # duty = 594 - duty
        print('xaxis angle is', f' {angle}' 'duty is' f' {duty}')
        self.pwm.set_pwm(self.xaxis_pin, 0, duty)
        time.sleep(0.1)

# root = Tk()
# root.wm_title('Servo Control')
# app = App(root)
# root.geometry("220x120+0+0")
# root.mainloop()
