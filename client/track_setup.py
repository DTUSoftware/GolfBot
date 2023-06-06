import os
import cv2
import json
import datetime
import drive_algorithm as drivealg

VIDEO_INPUT = int(os.environ.get('VIDEO_INPUT', 1))
TRACK_PRESET = os.environ.get('TRACK_PRESET', "track.json")

track_setup_mode = None
draw_path = []
objects = []


def save_preset(filename):
    if not os.path.exists("track_presets"):
        os.mkdir("track_presets")
    with open(f"track_presets/{filename}.json", "w+") as preset_file:
        json.dump(objects, preset_file)


def list_presets():
    if not os.path.exists("track_presets"):
        os.mkdir("track_presets")
    return os.listdir("track_presets")


def load_preset(preset):
    if not os.path.exists(f"track_presets/{preset}"):
        return
    with open(f"track_presets/{preset}", "r") as preset_file:
        data = json.load(preset_file)
        for obj in data:
            path = obj["path"]
            new_path = []
            for point in path:
                new_path.append((point[0], point[1]))
            obj["path"] = new_path
        global objects
        objects = data


def setup_track_mouse_input(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDBLCLK and track_setup_mode:
        global draw_path
        draw_path.append((x, y))


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


def setup_track() -> drivealg.Track:
    # Setup OpenCV window and mouse callback
    cv2.namedWindow("Track Setup")
    cv2.setMouseCallback("Track Setup", setup_track_mouse_input)

    # Open the video stream from camera
    cap = cv2.VideoCapture(VIDEO_INPUT)

    # Default width and height
    width = 1920.0
    height = 1080.0

    # Get variables from video stream
    global track_setup_mode
    global draw_path
    global objects

    # Try and load environment preset
    load_preset(TRACK_PRESET)
    # Whether loaded as a preset or not
    if objects:
        is_preset = True
    else:
        is_preset = False

    track_setup_mode = None
    draw_path = []
    text = None
    while cap.isOpened():
        # Get frame size of video
        width = cap.get(3)
        height = cap.get(4)

        # Capture a frame from the webcam
        success, frame = cap.read()

        # Break if fail to read
        if not success:
            break

        # Add text
        if text:
            put_text = text
        else:
            put_text = "q = quit, o = obstacle, s = small goal, b = big goal, u = undo, w = write/save, a = abort/abandon"
        cv2.putText(frame, put_text, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255))

        # Draw past objects
        if objects:
            for obj in objects:
                draw_object(frame, obj["object_type"], obj["path"])

        # Draw on track if adding objects
        if track_setup_mode and draw_path:
            draw_object(frame, track_setup_mode, draw_path)

        cv2.imshow('Track Setup', frame)

        # Key presses
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):  # Quit
            break
        elif key == ord("o"):  # Obstacle
            track_setup_mode = "obstacle"
            text = "Inserting obstacle... u = undo, w = write/save, a = abort - Double-click to add a point"
        elif key == ord("s"):  # Small goal
            track_setup_mode = "small_goal"
            text = "Inserting small goal... u = undo, w = write/save, a = abort - Double-click to add a point"
        elif key == ord("b"):  # Big goal
            track_setup_mode = "big_goal"
            text = "Inserting big goal... u = undo, w = write/save, a = abort - Double-click to add a point"
        elif key == ord("u"):  # Undo
            if track_setup_mode and draw_path:
                # Remove last added item
                draw_path.pop(-1)
        elif key == ord("l"):  # Load preset
            while True:
                presets = list_presets()
                preset = input(f"Choose a preset:\n{presets}\n\n")
                if preset == "quit":
                    break
                elif preset in presets:
                    load_preset(preset)
                    is_preset = True
                    break
        elif key == ord("w"):  # Save
            objects.append({"object_type": track_setup_mode, "path": draw_path})
            track_setup_mode = None
            draw_path = []
            text = None
            is_preset = False
        elif key == ord("a"):  # Abort
            track_setup_mode = None
            draw_path = []
            text = None

    # Release the webcam and close window
    cap.release()
    cv2.destroyAllWindows()

    # If not preset, save objects as new preset
    if not is_preset:
        save_preset(datetime.datetime.now().strftime("%Y%m%d%H%M%S"))

    # Setup the track / driving algorithm with given parameters
    track = drivealg.setup({"x": width, "y": height})

    # Add objects
    for obj in objects:
        object_type = obj["object_type"]
        if object_type == "obstacle":
            # print(f"Adding obstacle path {obj['path']} to track")
            nodes_in_path = track.graph.get_nodes_in_path(obj["path"])
            # print(f"Nodes in obstacle path: {[(node.x, node.y) for node in nodes_in_path]}")
            obstacle = drivealg.Obstacle(nodes_in_path)
            track.add_obstacle(obstacle)
        elif object_type == "small_goal":
            goal = drivealg.Goal(track.graph.get_nodes_in_path(obj["path"]), small=True)
            track.add_goal(goal)
        elif object_type == "big_goal":
            goal = drivealg.Goal(track.graph.get_nodes_in_path(obj["path"]), small=False)
            track.add_goal(goal)

    return track


if __name__ == "__main__":
    setup_track()
