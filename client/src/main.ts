import {BUTTON_ID, CANVAS_HEIGHT, CANVAS_WIDTH, EPOCH, EPSILON, FPS, MAIN_SYSTEM_ARROW_LENGTH, MAP_CANVAS_ID, OUTPUT_MAP_CANVAS_ID, POSITON_INDICATOR_ID, ROBOT_BOX_SIZE, ROBOT_SYSTEM_ARROW_LENGTH, SECOND_MS, SHOW_BASE_COORD_RADIO_ID, SHOW_RAYS_RADIO_ID, SHOW_ROBOT_COORD_RADIO_ID, XINPUT_ID, YINPUT_ID, ZINPUT_ID} from "./constants.js"
import { DrawManager } from "./drawManager.js";
import { Twist } from "./messages.js";
import { Model } from "./model.js";
import { Vec2 } from "./vec2.js";

class InputCheckBox{
    private target: HTMLInputElement;
    private checked = false;
    private callback: (target: InputCheckBox) => void;

    constructor(target: HTMLInputElement, callback = (target: InputCheckBox) => {}){
        this.target = target;
        this.callback = callback;
        target.addEventListener('click', (event: MouseEvent) => {
            this.checked = !this.checked;
            this.target.checked = this.checked;
            callback(this);
        });
    }

    public ischecked() { return this.checked }
}

function drawRobotPositionInFrame(canvasManager: DrawManager, position: Twist){
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

function drawRays(model: Model, canvasManager: DrawManager){
    const data = model.getLidarData();
    const position = model.getWorldPosition();
    for(let i = 0; i < data.distances.length; i++){
        if(data.distances[i] < 0) continue;
        const deltaAngle = data.minAngle + i * data.angleStep;
        canvasManager.drawArrow(position.linear, position.angular + deltaAngle, data.distances[i], '#0000ff');
    }
}

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

function main(){
    console.log("HOLLA");

    const formButton = document.getElementById(BUTTON_ID) as HTMLButtonElement; 
    const mapCanvas = document.getElementById(MAP_CANVAS_ID) as HTMLCanvasElement;
    const xinput = document.getElementById(XINPUT_ID) as HTMLInputElement;
    const yinput = document.getElementById(YINPUT_ID) as HTMLInputElement;
    const zinput = document.getElementById(ZINPUT_ID) as HTMLInputElement;
    const positionIdicator = document.getElementById(POSITON_INDICATOR_ID) as HTMLDivElement;
    const outputMapCanvas = document.getElementById(OUTPUT_MAP_CANVAS_ID) as HTMLCanvasElement;
    
    mapCanvas.setAttribute("width", CANVAS_WIDTH.toString());
    mapCanvas.setAttribute("height", CANVAS_HEIGHT.toString());

    outputMapCanvas.setAttribute("height", CANVAS_HEIGHT.toString());
    outputMapCanvas.setAttribute("width", CANVAS_WIDTH.toString());

    const model = new Model();
    const canvasManager = new DrawManager(mapCanvas);
    const mapCanvasManager = new DrawManager(outputMapCanvas);

    mapCanvas.addEventListener('click', (event: MouseEvent) => {
        model.setInitialPosition(canvasManager.transformToDomainCoord(new Vec2(event.offsetX, event.offsetY)),
            Number.parseFloat(zinput.value));
    });

    formButton.addEventListener('click', (event: MouseEvent) => {
        event.preventDefault();
        model.setInitialPosition(new Vec2(Number.parseFloat(xinput.value), Number.parseFloat(yinput.value)),
            Number.parseFloat(zinput.value));
    });

    let showRays = false;
    let showBaseCoord = true;
    let showRobotCoord = false;
    const inputs: InputCheckBox[] = [
        new InputCheckBox(document.getElementById(SHOW_RAYS_RADIO_ID) as HTMLInputElement, (target: InputCheckBox) => {
            showRays = target.ischecked();
        }),
        new InputCheckBox(document.getElementById(SHOW_BASE_COORD_RADIO_ID) as HTMLInputElement, (target: InputCheckBox) => {
            showBaseCoord = target.ischecked();
        }),
        new InputCheckBox(document.getElementById(SHOW_ROBOT_COORD_RADIO_ID) as HTMLInputElement, (target: InputCheckBox) => {
            showRobotCoord = target.ischecked();
        })
    ];

    setInterval(() => {
        canvasManager.clearDisplay();

        /* DRAW BASIC COORD SYSTEM ARROWS */
        if(showBaseCoord){
            canvasManager.drawArrow(new Vec2(), 0, MAIN_SYSTEM_ARROW_LENGTH);
            canvasManager.drawArrow(new Vec2(), Math.PI / 2, MAIN_SYSTEM_ARROW_LENGTH);
        }
        
        /* SET UP ROBOT IN MAP COORDINATE SYSTEM */
        const robotOdomPosition = model.getOdomPosition(); /* IN ROBOT SPACE */
        const robotInitialPosition = model.getInitialPosition(); /* IN MAP SPACE */
        const robotPosition = model.getWorldPosition();

        /* DRAW ROBOT */
        canvasManager.getContext().fillStyle = '#000000';
        const canvasPosition = canvasManager.transformToCanvasCoord(robotPosition.linear);
        canvasManager.getContext().fillRect(canvasPosition.x - ROBOT_BOX_SIZE / 2, canvasPosition.y - ROBOT_BOX_SIZE / 2, ROBOT_BOX_SIZE, ROBOT_BOX_SIZE);
        if(showRobotCoord){
            canvasManager.drawArrow(robotPosition.linear, robotPosition.angular, ROBOT_SYSTEM_ARROW_LENGTH, '#ff0000');
            canvasManager.drawArrow(robotPosition.linear, robotPosition.angular + Math.PI / 2, ROBOT_SYSTEM_ARROW_LENGTH, '#ff0000');
        }
        positionIdicator.innerHTML = `Current position = {x : ${robotPosition.linear.x}, y: ${robotPosition.linear.y}}`;
    
        drawRobotPositionInFrame(canvasManager, robotPosition);

        /* DRAW OBSTACLES */
        drawObstacles(model, canvasManager);

        /* DRAW LIDAR RAYS */
        if(showRays) drawRays(model, canvasManager);

        /* DRAW OBSTACLES ON SECONDARY CANVAS */
        // updateMapOutput(model, mapCanvasManager);

        /* UPDATE SIM */
        for(let i = 0; i < EPOCH; i++){
            model.updateSim();
        }

        /* SEND TO BASE NODE SENSOR DATA */
        /* TODO, IMPLEMENT */

    }, SECOND_MS / FPS );
}

main();