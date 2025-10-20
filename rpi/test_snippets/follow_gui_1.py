import cv2
import numpy as np
import time
import base64
import threading
from flask import Flask, render_template_string, jsonify, request
from picamera2 import Picamera2

# ----------------------------
# CONFIG
# ----------------------------
ONNX_MODEL = "yolo11n.onnx" 
INPUT_WIDTH = 320
INPUT_HEIGHT = 320
TARGET_CLASSES = [0, 2]  # person, car
CONFIDENCE_THRESHOLD = 0.5
NMS_THRESHOLD = 0.4
CLASS_NAMES = {0: "person", 2: "car"}
GUI_UPDATE_INTERVAL = 3.0  # seconds

# ----------------------------
# GLOBAL STATE
# ----------------------------
selected_target_id = None
tracked_targets = {}  # {id: {"thumbnail_b64": str, "class": str, "bbox": [...], "last_seen": float}}
next_target_id = 1
lock = threading.Lock()
last_gui_update = 0

# ----------------------------
# YOLO11 Post-processing (your code)
# ----------------------------
def decode_yolov8(outputs, img_shape, input_shape):
    outputs = outputs[0]
    num_candidates = outputs.shape[1]
    boxes = []
    scores = []
    class_ids = []

    x_factor = img_shape[1] / input_shape[0]
    y_factor = img_shape[0] / input_shape[1]

    for i in range(num_candidates):
        class_scores = outputs[4:, i]
        max_score = np.max(class_scores)
        if max_score >= CONFIDENCE_THRESHOLD:
            class_id = np.argmax(class_scores)
            if class_id not in TARGET_CLASSES:
                continue

            cx, cy, w, h = outputs[0:4, i]
            left = int((cx - w / 2) * x_factor)
            top = int((cy - h / 2) * y_factor)
            width = int(w * x_factor)
            height = int(h * y_factor)

            boxes.append([left, top, width, height])
            scores.append(float(max_score))
            class_ids.append(class_id)

    indices = cv2.dnn.NMSBoxes(boxes, scores, CONFIDENCE_THRESHOLD, NMS_THRESHOLD)
    if len(indices) == 0:
        return [], [], []

    if isinstance(indices, list):
        indices = np.array(indices)

    final_boxes = [boxes[i] for i in indices.flatten()]
    final_scores = [scores[i] for i in indices.flatten()]
    final_classes = [class_ids[i] for i in indices.flatten()]

    return final_boxes, final_scores, final_classes

# ----------------------------
# DETECTION + TRACKING LOOP
# ----------------------------
def detection_loop():
    global tracked_targets, next_target_id, last_gui_update

    # Load model
    net = cv2.dnn.readNetFromONNX(ONNX_MODEL)
    net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
    net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

    # Camera
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"size": (640, 480), "format": "RGB888"},
        controls={"FrameRate": 30}
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(2)

    print("Detection loop started. GUI: http://<RPI_IP>:5000")

    while True:
        frame = picam2.capture_array()
        h, w = frame.shape[:2]

        # Preprocess & infer
        blob = cv2.dnn.blobFromImage(
            frame, 1/255.0, (INPUT_WIDTH, INPUT_HEIGHT), swapRB=True, crop=False
        )
        net.setInput(blob)
        outputs = net.forward()

        boxes, scores, class_ids = decode_yolov8(outputs, (h, w), (INPUT_HEIGHT, INPUT_WIDTH))

        # Simple tracking: assign new IDs to boxes (no re-ID yet)
        current_targets = {}
        now = time.time()

        for i, (box, cls_id) in enumerate(zip(boxes, class_ids)):
            x, y, bw, bh = box
            # Crop thumbnail
            thumb = frame[y:y+bh, x:x+bw]
            if thumb.size == 0:
                continue
            thumb = cv2.resize(thumb, (96, 96))
            _, buffer = cv2.imencode('.jpg', thumb, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            b64 = base64.b64encode(buffer).decode()

            # Assign ID (in real app, use SORT/ByteTrack for persistent IDs)
            target_id = next_target_id
            next_target_id += 1

            current_targets[target_id] = {
                "thumbnail_b64": b64,
                "class": CLASS_NAMES[cls_id],
                "bbox": box,
                "last_seen": now
            }

        # Update global state (thread-safe)
        with lock:
            tracked_targets = current_targets
            last_gui_update = now

        time.sleep(0.05)  # ~20 Hz detection
        # ----------------------------
# FLASK APP
# ----------------------------
app = Flask(__name__)

HTML_TEMPLATE = '''
<!DOCTYPE html>
<html>
<head>
    <title>Drone Target Selector</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: Arial, sans-serif; padding: 10px; background: #f0f0f0; }
        .target { display: inline-block; margin: 10px; background: white; padding: 8px; border-radius: 8px; text-align: center; }
        img { width: 100px; height: 100px; object-fit: cover; border: 2px solid #ddd; }
        button { margin-top: 5px; padding: 6px 12px; background: #4CAF50; color: white; border: none; border-radius: 4px; cursor: pointer; }
        button:hover { background: #45a049; }
        #locked { color: green; font-weight: bold; margin-top: 15px; }
    </style>
</head>
<body>
    <h2>?? Select Target to Follow</h2>
    <div id="targets"></div>
    <div id="locked">Locked Target: <span id="locked_id">None</span></div>

    <script>
        function loadTargets() {
            fetch('/targets')
                .then(res => res.json())
                .then(targets => {
                    let html = '';
                    targets.forEach(t => {
                        html += `
                            <div class="target">
                                <img src="data:image/jpeg;base64,${t.thumbnail}" />
                                <p>ID: ${t.id} (${t.class})</p>
                                <button onclick="lock(${t.id})">Lock</button>
                            </div>`;
                    });
                    document.getElementById('targets').innerHTML = html;
                })
                .catch(err => console.log("Load error:", err));
        }

        function lock(id) {
            fetch('/lock', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({target_id: id})
            }).then(() => {
                document.getElementById('locked_id').textContent = id;
            });
        }

        setInterval(loadTargets, 3000);
        loadTargets();
    </script>
</body>
</html>
'''

''

@app.route('/')
def index():
    return render_template_string(HTML_TEMPLATE)

@app.route('/targets')
def get_targets():
    with lock:
        targets = [
            {"id": tid, "class": data["class"], "thumbnail": data["thumbnail_b64"]}
            for tid, data in tracked_targets.items()
        ]
    return jsonify(targets)

@app.route('/lock', methods=['POST'])
def lock_target():
    global selected_target_id
    data = request.get_json()
    tid = data.get("target_id")
    if tid is not None:
        with lock:
            selected_target_id = int(tid)
        print(f"TARGET LOCKED: ID = {selected_target_id}")
    return jsonify({"status": "locked", "id": tid})

# ----------------------------
# MAIN
# ----------------------------
if __name__ == "__main__":
    # Start detection in background thread
    detector_thread = threading.Thread(target=detection_loop, daemon=True)
    detector_thread.start()

    # Start Flask server
    print("Starting web server on http://0.0.0.0:5000")
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
