from evdev import InputDevice, categorize, ecodes
import io
import picamera
import logging
import socketserver
from threading import Condition, Thread
from http import server
from urllib.parse import urlparse

from motorControl import *
motor = motorControl(disable_motors=False)

gamepad = InputDevice('/dev/input/event0')

class StreamingOutput(object):
    def __init__(self):
        self.frame = None
        self.buffer = io.BytesIO()
        self.condition = Condition()

    def write(self, buf):
        if buf.startswith(b'\xff\xd8'):
            # New frame, copy the existing buffer's content and notify all
            # clients it's available
            self.buffer.truncate()
            with self.condition:
                self.frame = self.buffer.getvalue()
                self.condition.notify_all()
            self.buffer.seek(0)
        return self.buffer.write(buf)

class StreamingHandler(server.SimpleHTTPRequestHandler):

    def __init__(self, *args, **kwargs):
        kwargs['directory'] = '/home/pi/piwars2022/server/pages'
        super().__init__(*args, **kwargs)

    def do_GET(self):

        print("do_GET {}".format(self.path))

        if not self.path.startswith('/api/'):
            print("calling super do_GET {}".format(self.path))
            # fall back to super class GET handler if we are not calling an api method.
            super(StreamingHandler, self).do_GET()
            return

        print("calling api method {}".format(self.path))

        parsed = urlparse(self.path)
        if "=" in parsed.query:
            query_components = dict(qc.split("=") for qc in parsed.query.split("&"))
        else:
            query_components = None

        speed = 80
        
        if parsed.path == '/api/stop':
            content = "Stop".encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.send_header('Content-Length', len(content))
            self.end_headers()
            self.wfile.write(content)
            motor.throttle(0, 0)
            motor.release()
        elif parsed.path == '/api/forward':
            content = "Fwd".encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.send_header('Content-Length', len(content))
            self.end_headers()
            self.wfile.write(content)
            if query_components is not None:
                speed = query_components["speed"]
            motor.throttle(float(speed)/255.0, float(speed)/255.0)
        elif parsed.path == '/api/backward':
            content = "Bwd".encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.send_header('Content-Length', len(content))
            self.end_headers()
            self.wfile.write(content)
            if query_components is not None:
                speed = query_components["speed"]
            motor.throttle(-float(speed)/255.0, -float(speed)/255.0)
        elif parsed.path == '/api/left':
            content = "Left".encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.send_header('Content-Length', len(content))
            self.end_headers()
            self.wfile.write(content)
            if query_components is not None:
                speed = query_components["speed"]
            motor.throttle(-float(speed)/256.0, float(speed)/256.0)
        elif parsed.path == '/api/right':
            content = "Right".encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.send_header('Content-Length', len(content))
            self.end_headers()
            self.wfile.write(content)
            if query_components is not None:
                speed = query_components["speed"]
            motor.throttle(float(speed)/256.0, -float(speed)/256.0)
        elif parsed.path == '/api/servo':
            content = "Servo".encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.send_header('Content-Length', len(content))
            self.end_headers()
            self.wfile.write(content)
            id = query_components["id"]
            value = query_components["value"]
            motor.set_servo(id, value)
        elif parsed.path == '/api/servos':
            content = "Servo".encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.send_header('Content-Length', len(content))
            self.end_headers()
            self.wfile.write(content)
            query_components = dict(qc.split("=") for qc in parsed.query.split("&"))
            value1 = query_components["servo1"]
            value2 = query_components["servo2"]
            motor.set_servos(value1, value2)
        elif parsed.path == '/api/stream.mjpg':
            self.send_response(200)
            self.send_header('Age', 0)
            self.send_header('Cache-Control', 'no-cache, private')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
            self.end_headers()
            try:
                while True:
                    with output.condition:
                        output.condition.wait()
                        frame = output.frame
                    self.wfile.write(b'--FRAME\r\n')
                    self.send_header('Content-Type', 'image/jpeg')
                    self.send_header('Content-Length', len(frame))
                    self.end_headers()
                    self.wfile.write(frame)
                    self.wfile.write(b'\r\n')
            except Exception as e:
                logging.warning(
                    'Removed streaming client %s: %s',
                    self.client_address, str(e))
        else:
            self.send_error(404)
            self.end_headers()

class StreamingServer(socketserver.ThreadingMixIn, server.HTTPServer):
    allow_reuse_address = True
    daemon_threads = True

if False:
    with picamera.PiCamera(resolution='640x480', framerate=24) as camera:
        output = StreamingOutput()
        # Uncomment the next line to change your Pi's Camera rotation (in degrees)
        #camera.rotation = 90
        camera.start_recording(output, format='mjpeg')
        try:
            address = ('', 80)
            server = StreamingServer(address, StreamingHandler)
            server.serve_forever()
        finally:
            camera.stop_recording()
else:
    address = ('', 80)
    my_server = StreamingServer(address, StreamingHandler)
    Thread(target=my_server.serve_forever, daemon=True).start()

    for event in gamepad.read_loop():
        print(categorize(event))
        
