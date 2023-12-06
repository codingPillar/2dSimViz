var __awaiter = (this && this.__awaiter) || function (thisArg, _arguments, P, generator) {
    function adopt(value) { return value instanceof P ? value : new P(function (resolve) { resolve(value); }); }
    return new (P || (P = Promise))(function (resolve, reject) {
        function fulfilled(value) { try { step(generator.next(value)); } catch (e) { reject(e); } }
        function rejected(value) { try { step(generator["throw"](value)); } catch (e) { reject(e); } }
        function step(result) { result.done ? resolve(result.value) : adopt(result.value).then(fulfilled, rejected); }
        step((generator = generator.apply(thisArg, _arguments || [])).next());
    });
};
import { Communication } from "./communication.js";
import { BUTTON_ID, CANVAS_HEIGHT, CANVAS_TO_MAP_FACTOR, CANVAS_WIDTH, EPOCH, EPSILON, FPS, MAIN_SYSTEM_ARROW_LENGTH, MAP_CANVAS_ID, OUTPUT_MAP_CANVAS_ID, POSITON_INDICATOR_ID, ROBOT_BOX_SIZE, ROBOT_SYSTEM_ARROW_LENGTH, SECOND_MS, SERVER_ADDRESS, SERVER_PORT, SHOW_BASE_COORD_RADIO_ID, SHOW_RAYS_RADIO_ID, SHOW_ROBOT_COORD_RADIO_ID, SHOW_ROBOT_DIRECTION_RADIO_ID, SHOW_ROBOT_POSITION_RADIO_ID, SYNCHRONIZE_ROUTE, TIME_INDICATOR_ID, XINPUT_ID, YINPUT_ID, ZINPUT_ID } from "./constants.js";
import { DrawManager } from "./drawManager.js";
import { Model } from "./model.js";
import { Vec2 } from "./vec2.js";
class InputCheckBox {
    constructor(target, callback = (target) => { }, initial = false) {
        this.checked = false;
        this.target = target;
        this.callback = callback;
        this.checked = initial;
        this.target.checked = this.checked;
        target.addEventListener('click', (event) => {
            this.checked = !this.checked;
            this.target.checked = this.checked;
            callback(this);
        });
    }
    ischecked() { return this.checked; }
}
function waitForServer(communicationService) {
    return __awaiter(this, void 0, void 0, function* () {
        let connected = false;
        while (!connected) {
            try {
                yield communicationService.get(SYNCHRONIZE_ROUTE);
                connected = true;
            }
            catch (_a) {
                yield new Promise(r => setTimeout(r, 2000));
            }
        }
    });
}
function drawRobotPositionInFrame(canvasManager, position) {
    /* RENDER POSITION IN ROBOTS OWN CORRDINATE SYSTEM */
    let k1 = 0;
    let k2 = 0;
    const orientation = position.angular;
    if (Math.abs(Math.cos(orientation + Math.PI / 2)) < EPSILON) {
        k1 = position.linear.x / Math.cos(orientation);
        k2 = (position.linear.y - (position.linear.x * Math.tan(orientation))) / Math.sin(orientation + Math.PI / 2);
    }
    else {
        k1 = (position.linear.y - position.linear.x * Math.tan(orientation + Math.PI / 2)) / (Math.sin(orientation)
            - Math.cos(orientation) * Math.tan(orientation + Math.PI / 2));
        k2 = (position.linear.x - k1 * Math.cos(orientation)) / Math.cos(orientation + Math.PI / 2);
    }
    /* k1 IS THE MULTIPLE OF THE X AXIS OF THE ROBOT AND k2 IS THE MULTIPLE OF THE Y AXIS OF THE ROBOT AGAIN */
    const robotOrthogonalBaseX = Vec2.fromAngle(orientation).mult(k1);
    canvasManager.drawArrow(new Vec2(), orientation, k1, '#00ff00');
    canvasManager.drawArrow(robotOrthogonalBaseX, orientation + Math.PI / 2, k2, '#00ff00');
    return robotOrthogonalBaseX.add(Vec2.fromAngle(orientation + Math.PI / 2).mult(k2));
}
function drawObstacles(model, canvasManager) {
    const obstacles = model.getObstacles();
    const context = canvasManager.getContext();
    context.strokeStyle = '#800080';
    for (const obstacle of obstacles) {
        const segments = obstacle.getSegments();
        for (let i = 0; i < segments.length; i++) {
            const first = canvasManager.transformToCanvasCoord(segments[i]);
            const second = canvasManager.transformToCanvasCoord(segments[(i + 1) % segments.length]);
            context.beginPath();
            context.moveTo(first.x, first.y);
            context.lineTo(second.x, second.y);
            context.stroke();
        }
    }
}
function drawRays(model, canvasManager) {
    const data = model.getLidarData();
    const position = model.getWorldPosition();
    for (let i = 0; i < data.distances.length; i++) {
        if (data.distances[i] < 0)
            continue;
        const deltaAngle = data.minAngle + i * data.angleStep;
        canvasManager.drawArrow(position.linear, position.angular + deltaAngle, data.distances[i], '#0000ff');
    }
}
function updateMapOutput(model, canvasManager) {
    const data = model.getLidarData();
    const position = model.getOdomPosition();
    const context = canvasManager.getContext();
    for (let i = 0; i < data.distances.length; i++) {
        if (data.distances[i] < 0)
            continue;
        const deltaAngle = data.minAngle + i * data.angleStep;
        const endPoint = Vec2.fromAngle(position.angular + deltaAngle).mult(data.distances[i]).add(position.linear);
        const canvasEndPoint = canvasManager.transformToCanvasCoord(endPoint);
        context.fillRect(canvasEndPoint.x, canvasEndPoint.y, 10, 10);
    }
}
function main() {
    return __awaiter(this, void 0, void 0, function* () {
        console.log("HOLLA");
        const formButton = document.getElementById(BUTTON_ID);
        const mapCanvas = document.getElementById(MAP_CANVAS_ID);
        const xinput = document.getElementById(XINPUT_ID);
        const yinput = document.getElementById(YINPUT_ID);
        const zinput = document.getElementById(ZINPUT_ID);
        const positionIdicator = document.getElementById(POSITON_INDICATOR_ID);
        const timeIdicator = document.getElementById(TIME_INDICATOR_ID);
        const outputMapCanvas = document.getElementById(OUTPUT_MAP_CANVAS_ID);
        mapCanvas.setAttribute("width", CANVAS_WIDTH.toString());
        mapCanvas.setAttribute("height", CANVAS_HEIGHT.toString());
        outputMapCanvas.setAttribute("height", CANVAS_HEIGHT.toString());
        outputMapCanvas.setAttribute("width", CANVAS_WIDTH.toString());
        const model = new Model();
        const canvasManager = new DrawManager(mapCanvas);
        const mapCanvasManager = new DrawManager(outputMapCanvas);
        mapCanvas.addEventListener('click', (event) => {
            model.setInitialPosition(canvasManager.transformToDomainCoord(new Vec2(event.offsetX, event.offsetY)), Number.parseFloat(zinput.value));
        });
        formButton.addEventListener('click', (event) => {
            event.preventDefault();
            model.setInitialPosition(new Vec2(Number.parseFloat(xinput.value), Number.parseFloat(yinput.value)), Number.parseFloat(zinput.value));
        });
        let showRays = false;
        let showBaseCoord = true;
        let showRobotCoordSystem = false;
        let showRobotPosition = false;
        let showRobotDirection = true;
        const inputs = [
            new InputCheckBox(document.getElementById(SHOW_RAYS_RADIO_ID), (target) => {
                showRays = target.ischecked();
            }, showRays),
            new InputCheckBox(document.getElementById(SHOW_BASE_COORD_RADIO_ID), (target) => {
                showBaseCoord = target.ischecked();
            }, showBaseCoord),
            new InputCheckBox(document.getElementById(SHOW_ROBOT_COORD_RADIO_ID), (target) => {
                showRobotCoordSystem = target.ischecked();
            }, showRobotCoordSystem),
            new InputCheckBox(document.getElementById(SHOW_ROBOT_POSITION_RADIO_ID), (target) => {
                showRobotPosition = target.ischecked();
            }, showRobotPosition),
            new InputCheckBox(document.getElementById(SHOW_ROBOT_DIRECTION_RADIO_ID), (target) => {
                showRobotDirection = target.ischecked();
            }, showRobotDirection)
        ];
        const communitionService = new Communication(SERVER_ADDRESS, SERVER_PORT);
        /* WAIT FOR SERVER CONNEXION */
        yield waitForServer(communitionService);
        setInterval(() => __awaiter(this, void 0, void 0, function* () {
            canvasManager.clearDisplay();
            /* DRAW BASIC COORD SYSTEM ARROWS */
            if (showBaseCoord) {
                canvasManager.drawArrow(new Vec2(), 0, MAIN_SYSTEM_ARROW_LENGTH / CANVAS_TO_MAP_FACTOR);
                canvasManager.drawArrow(new Vec2(), Math.PI / 2, MAIN_SYSTEM_ARROW_LENGTH / CANVAS_TO_MAP_FACTOR);
            }
            /* SET UP ROBOT IN MAP COORDINATE SYSTEM */
            const robotOdomPosition = model.getOdomPosition(); /* IN ROBOT SPACE */
            const robotInitialPosition = model.getInitialPosition(); /* IN MAP SPACE */
            const robotPosition = model.getWorldPosition();
            /* DRAW ROBOT */
            canvasManager.getContext().fillStyle = '#000000';
            const canvasPosition = canvasManager.transformToCanvasCoord(robotPosition.linear);
            canvasManager.getContext().fillRect(canvasPosition.x - ROBOT_BOX_SIZE / 2, canvasPosition.y - ROBOT_BOX_SIZE / 2, ROBOT_BOX_SIZE, ROBOT_BOX_SIZE);
            if (showRobotCoordSystem) {
                canvasManager.drawArrow(robotPosition.linear, robotInitialPosition.angular, ROBOT_SYSTEM_ARROW_LENGTH / CANVAS_TO_MAP_FACTOR, '#ff0000');
                canvasManager.drawArrow(robotPosition.linear, robotInitialPosition.angular + Math.PI / 2, ROBOT_SYSTEM_ARROW_LENGTH / CANVAS_TO_MAP_FACTOR, '#ff0000');
            }
            if (showRobotPosition) {
                /* DRAW INITIAL POSITION */
                const endPoint = drawRobotPositionInFrame(canvasManager, robotInitialPosition);
                /* DRAW THE ODOM COMPONENT */
                canvasManager.drawArrow(endPoint, robotInitialPosition.angular, robotOdomPosition.linear.x, '#ffff00');
                canvasManager.drawArrow(endPoint.getAdd(Vec2.fromAngle(robotInitialPosition.angular).mult(robotOdomPosition.linear.x)), robotInitialPosition.angular + Math.PI / 2, robotOdomPosition.linear.y, '#ffff00');
            }
            if (showRobotDirection) {
                /* DRAW ROBOT DIRECTION */
                canvasManager.drawArrow(robotPosition.linear, robotPosition.angular, ROBOT_SYSTEM_ARROW_LENGTH / CANVAS_TO_MAP_FACTOR, '#00ffff');
            }
            positionIdicator.innerHTML = `Current position = {x : ${robotPosition.linear.x.toFixed(2)}, y: ${robotPosition.linear.y.toFixed(2)}}`;
            timeIdicator.innerHTML = `Current Elapsed Time: ${model.getTime().toFixed(2)} seconds`;
            /* DRAW OBSTACLES */
            drawObstacles(model, canvasManager);
            /* DRAW LIDAR RAYS */
            if (showRays)
                drawRays(model, canvasManager);
            /* DRAW OBSTACLES ON SECONDARY CANVAS */
            // updateMapOutput(model, mapCanvasManager);
            /* UPDATE SIM */
            for (let i = 0; i < EPOCH; i++) {
                model.updateSim();
            }
            /* SEND TO BASE NODE SENSOR DATA */
            /* TODO, IMPLEMENT */
            /* RECEIVE NEW COMMAND_VEL */
            const cmdVelReq = yield communitionService.get('');
            model.updateVel(cmdVelReq.linear[0], cmdVelReq.angular);
        }), SECOND_MS / FPS);
    });
}
main();
