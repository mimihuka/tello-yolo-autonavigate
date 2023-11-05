from ultralytics import YOLO
import cv2, math, time
from djitellopy import Tello

fov = 82.6 / 180 * math.pi
halftanfov = math.tan(fov / 2)
x = 0.0
y = 0.0
midx = 0.0
midy = 0.0
isDetectPerson = False
delay = 150

tello= Tello()
tello.connect()
tello.set_speed(100)

tello.streamon()
frame_read = tello.get_frame_read()

model = YOLO('yolov8n.pt')
exit = False
count = 0
tello.takeoff()

while not exit:
    img = frame_read.frame
    cvtd = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    results = model(cvtd)
    annotated_frame = results[0].plot()
    cv2.imshow("test_drone_yolo_smh", annotated_frame)
    h = tello.get_height()

    #get the position in YOLO
    for r in results:
        count = 0
        for j in r.boxes.cls.tolist():
            if j == 0.0:
                isDetectPerson = True
                x1,y1,x2,y2 = r.boxes.xyxyn[0].tolist()
                midx = (x2 - x1)/2
                midy = (y2 - y1)/2
                break
            count += 1

    #calc the center point of the person

    #control the drone to move
    if isDetectPerson:
        x = (midx - 0.5) * 2
        y = abs(midy - 0.5) * 2
        if y==0:
            y += 0.05
        disy = h / (halftanfov * y)
        angx = math.atan(x * halftanfov) / math.pi * 180
        if int(angx) == 0:
            angx = 1
        #delay for 150 frames
        delay -= 1
        if not delay:
            delay = 150
            if x > 0:
                tello.rotate_clockwise(int(abs(angx)))
            else:
                tello.rotate_counter_clockwise(int(abs(angx)))
            print("landinglandinglanding",int(disy))
            tello.go_xyz_speed(int(disy), 0, 0, 100)

            #falls down
            tello.emergency()
    
    #press q to quit
    if cv2.waitKey(1) & 0xFF == ord("q"):
        exit = True
    
    #count the loop
    count += 1

cv2.destroyAllWindows()