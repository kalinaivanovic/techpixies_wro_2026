import threading
import cv2
import numpy as np

CAMERA_INDEX = 0
CAMERA_WIDTH = 640
CAMERA_HEIGHT= 480

class Camera:
    """
    Captures frames from camera in background thread

    Usage: 
        camera = Camera()
        camera.start()

        jpeg = camera.get_jpeg_frame()

        camera.stop()
    """
    def __init__(self):
        self._running = False
        self._cap: cv2.VideoCapture | None = None
        self._thread: threading.Thread | None = None
        self._frame: np.ndarray | None = None
        self._lock = threading.Lock()
        

    @property
    def is_running(self):
        return self._running

    def start(self) -> bool:
        """Open camera and start capturing in background"""
        if self._running:
            return True
        
        #Openning camera
        self._cap = cv2.VideoCapture(CAMERA_INDEX)
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)

        if not self._cap.isOpened():
            print("ERROR: Failed to open camera")
            return False   

        #Start background thread
        self._running = True
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()
    
    def stop(self):
        pass

    def get_frame(self):
        pass

    def get_jpeg_frame(self, quality=80):
        pass

    def _capture_loop(self):
        while self._running:
            ret, frame = self._cap.read()
            if not ret:
                continue
            #Store frame safely
            with self._lock:
                self._frame = frame

    
    