from ultralytics import YOLO
model = YOLO("yolo11n.pt")
model.export(
    format="onnx",
    imgsz=320,             
    opset=12,             
    simplify=True,        
    dynamic=False,       
    batch=1 
)
