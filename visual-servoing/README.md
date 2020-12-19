Run `docker exec -it $(docker ps -aqf "name=^agent$") /code/exercise_ws/src/checkpoint.sh` to save the current image as a checkpoint.

Custom Topics:
* line_detector_node/homography. FloatList of length 9.
* line_detector_node/debug/checkpoint/compressed. Checkpoint image.
* line_detector_node/debug/vs_lines/compressed. Ground projected color coordinates and lines for the current image.
* line_detector_node/debug/vs_lines_checkpoint/compressed. Ground projected color coordinates and lines for the current checkpoint.
* line_detector_node/debug/color_coordinates/compressed. Color coordinates and lines for the current image in image space.
* line_detector_node/debug/color_coordinates_checkpoint/compressed. Color coordinates and lines for the current checkpoint in image space.