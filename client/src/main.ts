import { Communication } from "./communication.js";
import {BUTTON_ID, CANVAS_HEIGHT, CANVAS_TO_MAP_FACTOR, CANVAS_WIDTH, EPOCH, EPSILON, FPS, GET_CMD_VEL_ROUTE, MAIN_SYSTEM_ARROW_LENGTH, MAP_CANVAS_ID, OUTPUT_MAP_CANVAS_ID, POSITON_INDICATOR_ID, POST_LIDAR_DATA_ROUTE, POST_ODOM_DATA_ROUTE, ROBOT_BOX_SIZE, ROBOT_SYSTEM_ARROW_LENGTH, SECOND_MS, SERVER_ADDRESS, SERVER_PORT, SHOW_BASE_COORD_RADIO_ID, SHOW_RAYS_RADIO_ID, SHOW_ROBOT_COORD_RADIO_ID, SHOW_ROBOT_DIRECTION_RADIO_ID, SHOW_ROBOT_POSITION_RADIO_ID, SYNCHRONIZE_ROUTE, TIME_INDICATOR_ID, XINPUT_ID, YINPUT_ID, ZINPUT_ID} from "./constants.js"
import { DrawManager } from "./drawManager.js";
import { CmdVel, LidarScan, Twist } from "./messages.js";
import { Model } from "./model.js";
import { Vec2 } from "./vec2.js";

class InputCheckBox{
    private target: HTMLInputElement;
    private checked = false;

    constructor(target: HTMLInputElement, callback = (target: InputCheckBox) => {}, initial: boolean = false){
        this.target = target;
        this.checked = initial;
        this.target.checked = this.checked;
        target.addEventListener('click', (event: MouseEvent) => {
            this.checked = !this.checked;
            this.target.checked = this.checked;
            callback(this);
        });
    }

    public ischecked() { return this.checked }
}

async function waitForServer(communicationService: Communication){
    let connected = false;
    while(!connected){
        try{
            await communicationService.get<{}>(SYNCHRONIZE_ROUTE);
            connected = true;
        }catch {
            await new Promise(r => setTimeout(r, 2000));
        }
    }
}

function drawRobotPositionInFrame(canvasManager: DrawManager, position: Twist): Vec2{
    /* RENDER POSITION IN ROBOTS OWN CORRDINATE SYSTEM */
    let k1: number = 0;
    let k2: number = 0;
    const orientation = position.angular;
    if(Math.abs(Math.cos(orientation + Math.PI / 2)) < EPSILON){
        k1 = position.linear.x / Math.cos(orientation);
        k2 = (position.linear.y - (position.linear.x * Math.tan(orientation))) / Math.sin(orientation + Math.PI / 2);
    }
    else{
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

function drawObstacles(model: Model, canvasManager: DrawManager){
    const obstacles = model.getObstacles();
    const context = canvasManager.getContext();
    context.strokeStyle = '#800080'
    for(const obstacle of obstacles){
        const segments = obstacle.getSegments();
        for(let i = 0; i < segments.length; i++){
            const first = canvasManager.transformToCanvasCoord(segments[i])
            const second = canvasManager.transformToCanvasCoord(segments[(i + 1) % segments.length]);
            context.beginPath();
            context.moveTo(first.x, first.y);
            context.lineTo(second.x, second.y);
            context.stroke();
        }
    }
}

function drawRays(lidarScan: LidarScan, worldPosition: Twist, canvasManager: DrawManager){
    for(let i = 0; i < lidarScan.distances.length; i++){
        if(lidarScan.distances[i] < 0) continue;
        const deltaAngle = lidarScan.minAngle + i * lidarScan.angleStep;
        canvasManager.drawArrow(worldPosition.linear, worldPosition.angular + deltaAngle, lidarScan.distances[i], '#0000ff');
    }
}

/* USE WHEN READY TO HAVE SECOND CANVAS FOR MAP VIZ */
/*
function updateMapOutput(model: Model, canvasManager: DrawManager){
    const data = model.getLidarData();
    const position = model.getOdomPosition();
    const context = canvasManager.getContext();
    for(let i = 0; i < data.distances.length; i++){
        if(data.distances[i] < 0) continue;
        const deltaAngle = data.minAngle + i * data.angleStep;
        const endPoint = Vec2.fromAngle(position.angular + deltaAngle).mult(data.distances[i]).add(position.linear);
        const canvasEndPoint = canvasManager.transformToCanvasCoord(endPoint);
        context.fillRect(canvasEndPoint.x, canvasEndPoint.y, 10, 10);
    }
}
*/

async function main(){
    console.log("HOLLA");

    const formButton = document.getElementById(BUTTON_ID) as HTMLButtonElement; 
    const mapCanvas = document.getElementById(MAP_CANVAS_ID) as HTMLCanvasElement;
    const xinput = document.getElementById(XINPUT_ID) as HTMLInputElement;
    const yinput = document.getElementById(YINPUT_ID) as HTMLInputElement;
    const zinput = document.getElementById(ZINPUT_ID) as HTMLInputElement;
    const positionIdicator = document.getElementById(POSITON_INDICATOR_ID) as HTMLDivElement;
    const timeIdicator = document.getElementById(TIME_INDICATOR_ID) as HTMLDivElement;

    /* USE WHEN READY TO HAVE SECOND CANVAS FOR MAP VIZ */
    //const outputMapCanvas = document.getElementById(OUTPUT_MAP_CANVAS_ID) as HTMLCanvasElement;
    
    const model = new Model();

    const canvasManager = new DrawManager(mapCanvas);
    mapCanvas.setAttribute("width", CANVAS_WIDTH.toString());
    mapCanvas.setAttribute("height", CANVAS_HEIGHT.toString());

    /* USE WHEN READY TO HAVE SECOND CANVAS FOR MAP VIZ */
    /*
    const mapCanvasManager = new DrawManager(outputMapCanvas);
    outputMapCanvas.setAttribute("height", CANVAS_HEIGHT.toString());
    outputMapCanvas.setAttribute("width", CANVAS_WIDTH.toString());
    */

    formButton.addEventListener('click', (event: MouseEvent) => {
        event.preventDefault();
        model.setInitialPosition(new Vec2(Number.parseFloat(xinput.value), Number.parseFloat(yinput.value)),
            Number.parseFloat(zinput.value));
    });

    let showRays = false;
    let showBaseCoord = true;
    let showRobotCoordSystem = false;
    let showRobotPosition = false;
    let showRobotDirection = true;
    const inputs: InputCheckBox[] = [
        new InputCheckBox(document.getElementById(SHOW_RAYS_RADIO_ID) as HTMLInputElement, (target: InputCheckBox) => {
            showRays = target.ischecked();
        }, showRays),
        new InputCheckBox(document.getElementById(SHOW_BASE_COORD_RADIO_ID) as HTMLInputElement, (target: InputCheckBox) => {
            showBaseCoord = target.ischecked();
        }, showBaseCoord),
        new InputCheckBox(document.getElementById(SHOW_ROBOT_COORD_RADIO_ID) as HTMLInputElement, (target: InputCheckBox) => {
            showRobotCoordSystem = target.ischecked();
        }, showRobotCoordSystem),
        new InputCheckBox(document.getElementById(SHOW_ROBOT_POSITION_RADIO_ID) as HTMLInputElement, (target: InputCheckBox) => {
            showRobotPosition = target.ischecked();
        }, showRobotPosition),
        new InputCheckBox(document.getElementById(SHOW_ROBOT_DIRECTION_RADIO_ID) as HTMLInputElement, (target: InputCheckBox) => {
            showRobotDirection = target.ischecked();
        }, showRobotDirection)
    ];

    const communitionService: Communication = new Communication(SERVER_ADDRESS, SERVER_PORT);
    /* WAIT FOR SERVER CONNEXION */
    await waitForServer(communitionService);

    const run = async () => {
        canvasManager.clearDisplay();

        /* DRAW BASIC COORD SYSTEM ARROWS */
        if(showBaseCoord){
            canvasManager.drawArrow(new Vec2(), 0, MAIN_SYSTEM_ARROW_LENGTH / CANVAS_TO_MAP_FACTOR);
            canvasManager.drawArrow(new Vec2(), Math.PI / 2, MAIN_SYSTEM_ARROW_LENGTH / CANVAS_TO_MAP_FACTOR);
        }
        
        /* SET UP ROBOT IN MAP COORDINATE SYSTEM */
        const robotOdomPosition = model.getOdomPosition(); /* IN ROBOT SPACE */
        const robotInitialPosition = model.getInitialPosition(); /* IN MAP SPACE */
        const robotPosition = model.getWorldPosition();
        const lidarData = model.getLidarData();

        /* DRAW ROBOT */
        canvasManager.getContext().fillStyle = '#000000';
        const canvasPosition = canvasManager.transformToCanvasCoord(robotPosition.linear);
        canvasManager.getContext().fillRect(canvasPosition.x - ROBOT_BOX_SIZE / 2, canvasPosition.y - ROBOT_BOX_SIZE / 2, ROBOT_BOX_SIZE, ROBOT_BOX_SIZE);
        if(showRobotCoordSystem){
            canvasManager.drawArrow(robotPosition.linear, robotInitialPosition.angular, ROBOT_SYSTEM_ARROW_LENGTH / CANVAS_TO_MAP_FACTOR, '#ff0000');
            canvasManager.drawArrow(robotPosition.linear, robotInitialPosition.angular + Math.PI / 2, ROBOT_SYSTEM_ARROW_LENGTH / CANVAS_TO_MAP_FACTOR, '#ff0000');
        }
        if(showRobotPosition){
            /* DRAW INITIAL POSITION */
            const endPoint: Vec2 = drawRobotPositionInFrame(canvasManager, robotInitialPosition);
            /* DRAW THE ODOM COMPONENT */
            canvasManager.drawArrow(endPoint, robotInitialPosition.angular, robotOdomPosition.linear.x, '#ffff00');
            canvasManager.drawArrow(endPoint.getAdd(Vec2.fromAngle(robotInitialPosition.angular).mult(robotOdomPosition.linear.x)),
            robotInitialPosition.angular + Math.PI / 2, robotOdomPosition.linear.y, '#ffff00');
        }
        if(showRobotDirection){
            /* DRAW ROBOT DIRECTION */
            canvasManager.drawArrow(robotPosition.linear, robotPosition.angular, ROBOT_SYSTEM_ARROW_LENGTH / CANVAS_TO_MAP_FACTOR, '#00ffff');
        }
        positionIdicator.innerHTML = `Current position = {x : ${robotPosition.linear.x.toFixed(2)}, y: ${robotPosition.linear.y.toFixed(2)}}`;
        timeIdicator.innerHTML = `Current Elapsed Time: ${model.getTime().toFixed(2)} seconds`

        /* DRAW OBSTACLES */
        drawObstacles(model, canvasManager);

        /* DRAW LIDAR RAYS */
        if(showRays) drawRays(lidarData, robotPosition, canvasManager);

        /* DRAW OBSTACLES ON SECONDARY CANVAS */
        /* USE WHEN READY TO HAVE SECOND CANVAS FOR MAP VIZ */
        // updateMapOutput(model, mapCanvasManager);

        /* UPDATE SIM */
        for(let i = 0; i < EPOCH; i++){
            model.updateSim();
        }

        /* SEND TO BASE NODE SENSOR DATA */
        await communitionService.post<LidarScan, {}>(POST_LIDAR_DATA_ROUTE, lidarData);
        await communitionService.post<{x: number, y: number, angle: number}, {}>(POST_ODOM_DATA_ROUTE, 
            {x: robotOdomPosition.linear.x, y: robotOdomPosition.linear.y, angle: robotPosition.angular});

        /* RECEIVE NEW COMMAND_VEL */
        const cmdVelReq = await communitionService.get<CmdVel>(GET_CMD_VEL_ROUTE);
        model.updateVel(cmdVelReq.linear[0], cmdVelReq.angular);
    };

    let running = true;
    while(running) await run();
}

main();