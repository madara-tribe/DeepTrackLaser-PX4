import cv2
import numpy as np
import time
import onnxruntime
from multiprocessing import Queue
from yolov7s.common import letterbox, preprocess, onnx_inference, post_process
from yolov7s.dist_calcurator import prams_calcurator, angle_convert
from .csicam_pipeline import CSI_Camera, gstreamer_pipeline
cuda = False

def create_csicams(hyp, sensor_id):
    csi_camera = CSI_Camera()
    csi_camera.open(
        gstreamer_pipeline(
            sensor_id=sensor_id,
            capture_width=hyp['capture_width'],
            capture_height=hyp['capture_height'],
            flip_method=0,
            display_width=hyp['display_width'],
            display_height=hyp['display_height'],
        )
    )
    return csi_camera


class DualCamera(object):
    def __init__(self, q:Queue, opt, hyp):
        self.q = q
        self.TIMEOUT = opt.frame_interval
        self.count = 0
        self.def_x = opt.x_coord
        self.def_y = opt.y_coord
        self.conf_thres = opt.conf_thres
        self.Rstack = self.Lstack = []
        self.distance = self.disparity = 0
        self.realX = self.realY = 0
        self.real_x_angle = self.real_y_angle = 0
        self.max_disparity = opt.max_disparity
        self.min_disparity = opt.min_disparity
        self.left_camera = create_csicams(hyp, sensor_id=0)
        self.right_camera = create_csicams(hyp, sensor_id=1)
        self.init_onnx_model(opt)
        self.run_dual_cam(opt, hyp)

    def init_onnx_model(self, opt):
        providers = ['CUDAExecutionProvider', 'CPUExecutionProvider'] if cuda else ['CPUExecutionProvider']
        self.session = onnxruntime.InferenceSession(opt.onnx_path, providers=providers)
        IN_IMAGE_H = self.session.get_inputs()[0].shape[2]
        IN_IMAGE_W = self.session.get_inputs()[0].shape[3]
        self.new_shape = (IN_IMAGE_W, IN_IMAGE_H)
    
    def qt_onnx_inference(self, frame):
        ori_images = [frame.copy()]
        resized_image, ratio, dwdh = letterbox(frame, new_shape=self.new_shape, auto=False)
        input_tensor = preprocess(resized_image)
        outputs = onnx_inference(self.session, input_tensor)
        pred_output, coordinate_x, coordinate_y = post_process(outputs, ori_images, ratio, dwdh, self.conf_thres)
        return pred_output[0], coordinate_x, coordinate_y

    def frame_reset(self):
        self.Rstack = []
        self.Lstack = []
        self.count = 0
    
    
    def run_dual_cam(self, opt, hyp):
        window_title = "Dual CSI Cameras"
        # left csi camera
        self.left_camera.start()
        # right camera
        self.right_camera.start()
        if self.left_camera.video_capture.isOpened() and self.right_camera.video_capture.isOpened():
            cv2.namedWindow(window_title, cv2.WINDOW_AUTOSIZE)
            try:
                while True:
                    _, frameR = self.right_camera.read()
                    _, frameL = self.left_camera.read()
                    if frameL is None or frameR is None:
                        continue
                    #self.count += 0.05
                    #if (time.time() - self.count) > self.TIMEOUT:
                    start = time.time()
                        # print(frameR.shape)
                    frameR_, Rx, Ry = self.qt_onnx_inference(frameR)
                    frameL_, Lx, Ly = self.qt_onnx_inference(frameL)
                    # print(frameR_.shape)
                    if Rx >0 and Lx > 0:
                        hlen, wlen = frameR.shape[:2]
                        disparity = abs(Rx-Lx)
                        print('disparity', disparity, wlen, hlen)
                        if disparity <= self.max_disparity and disparity > self.min_disparity:
                            self.disparity, self.distance, self.realX, self.realY = prams_calcurator(hyp, disparity,
                            wlen, cx=self.def_y, cy=self.def_y, x=int((Rx+Lx)/2), y=int((Ry+Ly)/2))
                    self.real_x_angle, self.real_y_angle = angle_convert(self.realX, self.realY, self.distance)
                    print("x angle, yangle, disparity, distance", int((Rx+Lx)/2), int((Ry+Ly)/2), self.real_x_angle, self.real_y_angle, self.disparity, self.distance)
                    xang, yang = self.real_x_angle, self.real_y_angle
                    if opt.software_test:
                        camera_images = np.hstack((cv2.resize(frameR_, (960, 540)), cv2.resize(frameL_, (960, 540))))
                        cv2.imshow(window_title, camera_images)
                    elif opt.pwm:
                        if isinstance(xang, float):
                            self.q.put(['x', xang])
                        if isinstance(xang, float):
                            self.q.put(['y', yang])
                    # This also acts as
                    pred_time = np.round((time.time() - start), decimals=5)
                    print('pred_time is ', pred_time)
                    # self.frame_reset()
                    keyCode = cv2.waitKey(30) & 0xFF
                    # Stop the program on the ESC key
                    if keyCode == 27:
                        break
            finally:        
                self.left_camera.stop()
                self.left_camera.release()
                self.right_camera.stop()
                self.right_camera.release()
                cv2.destroyAllWindows()
        else:
            print('unable to open cameras')
            self.left_camera.stop()
            self.left_camera.release()
            self.right_camera.stop()
            self.right_camera.release()
            cv2.destroyAllWindows()

