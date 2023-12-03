export const CANVAS_WIDTH = 500;
export const CANVAS_HEIGHT = 500;
/* HTML RELATED */
export const MAP_CANVAS_ID = "mapCanvas";
export const BUTTON_ID = "formSubmit";
export const XINPUT_ID = "xinput";
export const YINPUT_ID = "yinput";
export const ZINPUT_ID = "zinput";
export const POSITON_INDICATOR_ID = "positionIndicator";
export const TIME_INDICATOR_ID = "timeIndicator";
export const OUTPUT_MAP_CANVAS_ID = "outputMapCanvas";
export const SHOW_RAYS_RADIO_ID = "showRaysRadioInput";
export const SHOW_BASE_COORD_RADIO_ID = "showBaseCoordInput";
export const SHOW_ROBOT_COORD_RADIO_ID = "showRobotCoordInput";
export const SHOW_ROBOT_POSITION_RADIO_ID = "showRobotPositionInput";
export const SHOW_ROBOT_DIRECTION_RADIO_ID = "showRobotDirectionInput";
/* DRAWING RELATED */
export const ROBOT_BOX_SIZE = 20;
export const MAIN_SYSTEM_ARROW_LENGTH = 100;
export const ROBOT_SYSTEM_ARROW_LENGTH = 50;
/* SIMULATION RELATED */
export const FPS = 20;
export const SECOND_MS = 1000;
export const dt = 0.001;
export const EPOCH = 20;
/* WITH THESE CONFIGS, WE HAVE 20 * 0.001 * 20 = 0.4 SEC PER 1 SEC IN REAL LIFE */
/* MAP RELATED */
export const DEFAULT_OBSTACLE_COUNT = 4;
export const CANVAS_TO_MAP_FACTOR = 20;
export const MAP_DOMAIN = [CANVAS_WIDTH / (2 * CANVAS_TO_MAP_FACTOR), CANVAS_WIDTH / (2 * CANVAS_TO_MAP_FACTOR)];
export const MAP_IMAGE = [CANVAS_HEIGHT / (2 * CANVAS_TO_MAP_FACTOR), CANVAS_HEIGHT / (2 * CANVAS_TO_MAP_FACTOR)];
/* MATH RELATED */
export const EPSILON = 0.0001;
/* LIDAR RELATED */
export const ANGLE_VIEW = Math.PI;
export const NUM_RAYS = 10;
export const ERROR_DISTANCE = -1;
/* COMMUNICATION WITH ROS NODE RELATED */
export const SERVER_ADDRESS = "127.0.0.1";
export const SERVER_PORT = "50601";
