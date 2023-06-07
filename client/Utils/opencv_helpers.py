import cv2


def draw_object(img, object_type, path):
    color = (0, 0, 0)
    if object_type == "obstacle":
        color = (0, 0, 255)
    elif object_type == "small_goal":
        color = (255, 0, 0)
    elif object_type == "big_goal":
        color = (0, 255, 0)

    for i in range(len(path)):
        point = path[i]

        # Add a dot at the point
        cv2.circle(img, point, 3, color)

        # Add a line
        if i >= 1:
            cv2.line(img, path[i - 1], point, color)
