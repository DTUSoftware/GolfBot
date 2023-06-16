import datetime
import json
import os
import cv2
from Utils import path_algorithm as pathalg
from Utils import opencv_helpers

# The webcam to use
VIDEO_INPUT = int(os.environ.get('VIDEO_INPUT', 0))
# The track preset to use
TRACK_PRESET = os.environ.get('TRACK_PRESET', "track.json")

# Global variables
track_setup_mode = None  # The current track setup mode
draw_path = []  # The path to draw
objects = []  # The objects to draw


def save_preset(filename):
    """
    Saves the current track presets to a JSON file.

    Args:
        filename (str): The name of the preset file (without the extension).

    Returns:
        None
    """
    if not os.path.exists("track_presets"):
        os.mkdir("track_presets")
    with open(f"track_presets/{filename}.json", "w+") as preset_file:
        json.dump(objects, preset_file)


def list_presets():
    """
    Lists all available track presets.

    Returns:
        list: A list of preset filenames.
    """
    if not os.path.exists("track_presets"):
        os.mkdir("track_presets")
    return os.listdir("track_presets")


def load_preset(preset):
    """
    Loads a track preset from a JSON file.

    Args:
        preset (str): The name of the preset file (including the extension).

    Returns:
        None
    """
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
    """
    Mouse callback function for track setup.

    Args:
        event (int): The mouse event (e.g., cv2.EVENT_LBUTTONDBLCLK).
        x (int): The x-coordinate of the mouse position.
        y (int): The y-coordinate of the mouse position.
        flags (int): Additional flags.
        param (Any): Optional parameters.

    Returns:
        None
    """
    if event == cv2.EVENT_LBUTTONDBLCLK and track_setup_mode:
        global draw_path
        draw_path.append((x, y))


def setup_track() -> pathalg.Track:
    """
    Sets up the track for capturing mouse input.

    Returns:
        pathalg.Track: The track object.
    """
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
                opencv_helpers.draw_object(frame, obj["object_type"], obj["path"])

        # Draw on track if adding objects
        if track_setup_mode and draw_path:
            opencv_helpers.draw_object(frame, track_setup_mode, draw_path)

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
    track = pathalg.setup({"x": width, "y": height})
    # opencv_helpers.FRAME_SIZE = (width, height)

    # Add objects
    for obj in objects:
        object_type = obj["object_type"]
        path = [opencv_helpers.opencv_position_to_graph_position(point) for point in obj["path"]]
        if object_type == "obstacle":
            # print(f"Adding obstacle path {obj['path']} to track")
            # print(f"Adding obstacle path {path} to track")
            nodes_in_path = track.graph.get_nodes_in_path(path)
            # print(f"Nodes in obstacle path: {[(node.x, node.y) for node in nodes_in_path]}")
            obstacle = pathalg.Obstacle(nodes_in_path, path)
            track.add_obstacle(obstacle)
        elif object_type == "small_goal":
            goal = pathalg.Goal(track.graph.get_nodes_in_path(path), path, small=True)
            track.add_goal(goal)
        elif object_type == "big_goal":
            goal = pathalg.Goal(track.graph.get_nodes_in_path(path), path, small=False)
            track.add_goal(goal)

    return track


if __name__ == "__main__":
    setup_track()
