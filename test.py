import cv2
import os
import threading
from datetime import datetime
from flask import Flask, Response, jsonify, request
from ultralytics import YOLO

app = Flask(__name__)

model = YOLO("yolov8n_ncnn_model/")

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 30)

if not cap.isOpened():
    print("Error: Could not open camera")
    exit(1)

recording = False
video_writer = None
record_thread = None

def get_save_path():
    downloads = os.path.expanduser("~/Downloads")
    os.makedirs(downloads, exist_ok=True)
    filename = datetime.now().strftime("recording_%Y%m%d_%H%M%S.avi")
    return os.path.join(downloads, filename)

def record_for(duration: int):
    global recording, video_writer
    path = get_save_path()
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    video_writer = cv2.VideoWriter(path, fourcc, 30.0, (640, 480))
    recording = True
    print(f"Recording started -> {path}")
    frames_to_capture = 30 * duration
    count = 0
    while recording and count < frames_to_capture:
        ret, frame = cap.read()
        if not ret:
            break
        video_writer.write(frame)
        count += 1
    recording = False
    video_writer.release()
    video_writer = None
    print(f"Recording saved -> {path}")

def generate():
    frame_count = 0
    annotated = None
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        if frame_count % 5 == 0:
            results = model(frame, imgsz=320, conf=0.4, verbose=False)
            annotated = results[0].plot()
        display = annotated if annotated is not None else frame
        _, buffer = cv2.imencode('.jpg', display)
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
        frame_count += 1

@app.route('/')
def index():
    return '''
        <html>
        <head><title>TwinRover Camera</title></head>
        <body style="background:black; display:flex; flex-direction:column; justify-content:center; align-items:center; height:100vh; margin:0; gap:16px;">
            <img src="/video" style="max-width:100%;">
            <div style="display:flex; gap:10px; align-items:center;">
                <input id="duration" type="number" value="10" min="1"
                    style="padding:8px; border-radius:6px; border:none; width:80px; text-align:center;">
                <span style="color:white;">seconds</span>
                <button onclick="startRecording()"
                    style="padding:8px 20px; background:#e53e3e; color:white; border:none; border-radius:6px; cursor:pointer; font-size:14px;">
                    ⏺ Record
                </button>
                <span id="status" style="color:#68d391; font-family:monospace; font-size:13px;"></span>
            </div>
            <script>
                function startRecording() {
                    const duration = document.getElementById('duration').value;
                    const status = document.getElementById('status');
                    status.textContent = `Recording ${duration}s...`;
                    fetch(`/record?duration=${duration}`)
                        .then(r => r.json())
                        .then(d => { status.textContent = d.message; })
                        .catch(() => { status.textContent = 'Error starting recording'; });
                }
            </script>
        </body>
        </html>
    '''

@app.route('/video')
def video():
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/record')
def record():
    global record_thread, recording
    if recording:
        return jsonify({"message": "Already recording"}), 400
    duration = request.args.get('duration', 10, type=int)
    record_thread = threading.Thread(target=record_for, args=(duration,), daemon=True)
    record_thread.start()
    return jsonify({"message": f"Recording {duration}s -> ~/Downloads"})

if __name__ == '__main__':
    print("Stream running at http://192.168.137.180:5000")
    app.run(host='0.0.0.0', port=5000)
