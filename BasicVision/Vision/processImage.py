import cv2
import numpy as np

def start_process(cv_sink, nt_instance, output, imageWidth, imageHeight):
    data_entry = nt_instance.getTable("targetData").getEntry("data")
    targetData = nt_instance.getTable("targetData")

    center_x = imageWidth / 2
    center_y = imageHeight / 2
    data_entry.setDefaultString("-1 -1")

    while True:
        time, frame = cv_sink.grabFrame(np.zeros((imageHeight, imageWidth, 3), dtype=np.uint8))

        targetData.putNumber("centerX", center_x)
        targetData.putNumber("centerY", center_y)
        targetData.putNumber("rectWidth", width)
        targetData.putNumber("rectHeight", height)

        # Draw a line on the frame
        cv2.line(frame, (center_x,0), (center_x, imageHeight), (0,0,255), 2)
        cv2.line(frame, (center_y,0), (center_y, imageWidth), (0,0,255), 2)    

        # Send processed images to Shuffleboard
        if time == 0: # There is an error
            output.notifyError(cv_sink.getError()) 
            continue
  
        output.putFrame(frame)
