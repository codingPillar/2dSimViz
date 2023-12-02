import { CANVAS_TO_MAP_FACTOR } from "./constants.js";
import { Vec2 } from "./vec2.js";

const ARROW_END_POINT_ANGLE = Math.PI / 10;
const ARROW_END_LENGTH_FACTOR = 5;

/* ALL DRAW CALLS ASSUME (0, 0) TO BE THE CENTER OF THE CANVAS */
export class DrawManager{
    
    constructor(canvas: HTMLCanvasElement){
        this.canvas = canvas;
        this.context = canvas.getContext('2d')!;
    }

    /* */
    public drawArrow(origin: Vec2, orientation: number, length: number, color = '#000000'){
        const endPoint = origin.getAdd(Vec2.fromAngle(orientation).mult(length));

        const canvasOrigin = this.transformToCanvasCoord(origin);
        const canvasEndPoint = this.transformToCanvasCoord(endPoint);

        this.context.strokeStyle = color
        this.context.lineWidth = 3;
        this.context.beginPath();
        this.context.moveTo(canvasOrigin.x, canvasOrigin.y);
        this.context.lineTo(canvasEndPoint.x, canvasEndPoint.y);
        this.context.stroke();

        this.context.beginPath();
        const canvasFirstBound = this.transformToCanvasCoord(
            Vec2.fromAngle(orientation).mult(-length / ARROW_END_LENGTH_FACTOR).rotate(ARROW_END_POINT_ANGLE).add(endPoint));
        this.context.moveTo(canvasEndPoint.x, canvasEndPoint.y);
        this.context.lineTo(canvasFirstBound.x, canvasFirstBound.y);
        const canvasSecondBound = this.transformToCanvasCoord(
            Vec2.fromAngle(orientation).mult(-length / ARROW_END_LENGTH_FACTOR).rotate(-ARROW_END_POINT_ANGLE).add(endPoint));
        this.context.moveTo(canvasEndPoint.x, canvasEndPoint.y);
        this.context.lineTo(canvasSecondBound.x, canvasSecondBound.y);
        this.context.stroke();
    }

    public getContext(): CanvasRenderingContext2D{
        return this.context;
    }

    public clearDisplay(color = '#FFFFFF'){
        this.context.fillStyle = color; 
        this.context.fillRect(0, 0, this.canvas.width, this.canvas.height);
    }

    public transformToDomainCoord(position: Vec2): Vec2 {
        return new Vec2(position.x - this.canvas.width / 2, -(position.y - this.canvas.height / 2)).mult(1 / CANVAS_TO_MAP_FACTOR);
    }

    public transformToCanvasCoord(position: Vec2): Vec2 {
        return new Vec2(position.x * CANVAS_TO_MAP_FACTOR + this.canvas.width / 2, -position.y * CANVAS_TO_MAP_FACTOR + this.canvas.height / 2);
    }

    /* PRIVATE MEMBERS */
    private canvas: HTMLCanvasElement;
    private context: CanvasRenderingContext2D;
}