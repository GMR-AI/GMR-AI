import numpy as np
import cv2

def segmentation(model, image):
    r = model.predict(image)[0]
    
    robot_mask = np.zeros_like(image, dtype=np.uint8)
    obstacle_mask = np.zeros_like(image, dtype=np.uint8)
    cls_list = r.boxes.cls.tolist()
    while cls_list:
        label = r.names[cls_list.pop()]

        contour = r.masks.xy.pop()
        contour = contour.astype(np.int32)
        contour = contour.reshape(-1, 1, 2)

        if label == 'robot':
            robot_mask = cv2.drawContours(robot_mask, [contour], -1, (255, 255, 255), cv2.FILLED)
        else:
            obstacle_mask = cv2.drawContours(obstacle_mask, [contour], -1, (255, 255, 255), cv2.FILLED)

    if not np.all(robot_mask == 0):
        robot_mask = cv2.threshold(cv2.cvtColor(robot_mask, cv2.COLOR_RGB2GRAY), 100, 255, cv2.THRESH_BINARY)[1]
    
    if not np.all(obstacle_mask == 0):
        obstacle_mask = cv2.threshold(cv2.cvtColor(obstacle_mask, cv2.COLOR_RGB2GRAY), 100, 255, cv2.THRESH_BINARY)[1]
    
    return robot_mask, obstacle_mask