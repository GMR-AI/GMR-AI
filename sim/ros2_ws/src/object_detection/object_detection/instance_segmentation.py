import numpy as np
import cv2

def segmentation(model, image):
    r = model.predict(image)
    
    robot_mask = np.zeros_like(image, dtype=np.uint8)
    obstacle_mask = np.zeros_like(image, dtype=np.uint8)
    
    for ci, c in enumerate(r):
        label = c.names[c.boxes.cls.tolist().pop()]

        contour = c.masks.xy.pop()
        contour = contour.astype(np.int32)
        contour = contour.reshape(-1, 1, 2)

        if label == 'robot':
            robot_mask = cv2.drawContours(robot_mask, [contour], -1, (255, 255, 255), cv2.FILLED)
        else:
            obstacle_mask = cv2.drawContours(obstacle_mask, [contour], -1, (255, 255, 255), cv2.FILLED)
    
    robot_mask = cv2.threshold(cv2.cvtColor(robot_mask, cv2.COLOR_RGB2GRAY), 100, 255, cv2.THRESH_BINARY)[1]
    obstacle_mask = cv2.threshold(cv2.cvtColor(obstacle_mask, cv2.COLOR_RGB2GRAY), 100, 255, cv2.THRESH_BINARY)[1]
    return robot_mask, obstacle_mask