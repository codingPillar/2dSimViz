import { floateq } from "./common.js";
import { ANGLE_VIEW, DEFAULT_OBSTACLE_COUNT, ERROR_DISTANCE, MAP_DOMAIN, NUM_RAYS, dt } from "./constants.js";
import { Vec2 } from "./vec2.js";
import { Twist, LidarScan } from "./messages.js"

export interface Ray{
    origin: Vec2;
    dir: Vec2;
}

export interface Collision{
    valid: boolean;
    position: Vec2;
}

interface Obstacle{
    computeCollision(ray: Ray): Collision;
    getSegments(): Vec2[];
}

class LineObs implements Obstacle {
    public first: Vec2;    
    public second: Vec2;

    constructor(first: Vec2, second: Vec2){
        this.first = first;
        this.second = second;
    }

    computeCollision(ray: Ray): {valid: boolean, position: Vec2} {
        const dir = this.second.getSub(this.first).getUnit();
        ray.dir = ray.dir.getUnit(); 
        const sameDir: boolean = floateq(dir.x, ray.dir.x) && floateq(dir.x, ray.dir.x);
        /* LINES ARE PARALLEL RETURN FALSE */
        if(sameDir) return {valid: false, position: new Vec2()};
        let k2 = 0;
        if(floateq(dir.x, 0) && !floateq(ray.dir.x, 0)) k2 = (this.first.x - ray.origin.x) / ray.dir.x; 
        else if(!floateq(dir.x, 0)) k2 = ((this.first.y - ray.origin.y) + (ray.origin.x - this.first.x) * dir.y / dir.x)
            / (ray.dir.y - (ray.dir.x * dir.y / dir.x));
        const final = ray.origin.copy().add(ray.dir.getMult(k2));
        if(this.inBound(final) && k2 > 0) return {valid: true, position: final};
        return {valid: false, position: new Vec2()};
    }

    getSegments(): Vec2[] {
        return [this.first, this.second];
    }

    private inBound(position: Vec2): boolean {
        const maxLength = this.second.getSub(this.first).length();
        const flength = position.getSub(this.first).length();
        const slength = position.getSub(this.second).length();
        if(flength > maxLength || slength > maxLength) return false;
        return true;
    }
}

/* APP MODEL STATE CLASS */
export class Model{
    /* MAP STATE */
    private domain: Vec2 = new Vec2(-MAP_DOMAIN, MAP_DOMAIN);
    private image: Vec2 = new Vec2(-MAP_DOMAIN, MAP_DOMAIN);

    /* ROBOT STATES */
    /* THIS WOULD BE THE ROBOT'S ODOM VALUE */
    private position: Twist = new Twist();
    /* IN CURRENT MODE, ONLY FOUR_DIFF IS SUPPORTED (TODO, ADD SUPPORT, FOR MECANUM DRIVE) */
    private velocity: Twist = new Twist();
    /* IN MAP COORDINATE SYSTEM */
    private initialPosition: Twist = new Twist();

    /* OBSTACLES */
    private obstacles: Obstacle[] = [];

    constructor(){
        for(let i = 0; i < DEFAULT_OBSTACLE_COUNT; i++) this.addRandomObstacle();
    }

    public updateSim(){
        /* FOUR DIFF */
        const displacement = Vec2.fromAngle(this.position.angular).mult(this.velocity.linear.x * dt);
        this.position.linear.add(displacement);
        this.position.angular += this.velocity.angular * dt;
    }

    public setInitialPosition(position: Vec2, orientation: number){
        this.initialPosition.linear = position;
        this.initialPosition.angular = orientation;
    }

    public getInitialPosition(): Twist{
        return this.initialPosition.copy();
    }

    public getOdomPosition(): Twist{
        return this.position.copy();
    }

    public getWorldPosition(): Twist {
        /* ROBOT ODOM IN MAP SPACE */
        const robotBaseX = Vec2.fromAngle(this.initialPosition.angular);
        const robotBaseY = Vec2.fromAngle(this.initialPosition.angular + Math.PI / 2);
        return new Twist(
            this.initialPosition.linear.getAdd(robotBaseX.mult(this.position.linear.x).add(robotBaseY.mult(this.position.linear.y))),
            this.position.angular + this.initialPosition.angular);
    }

    public addRandomObstacle(){
        /* TODO, MAKE IT DEPEND ON THE DOMAIN AND IMAGE OF MAP */
        this.obstacles.push(new LineObs(
            Vec2.random(-MAP_DOMAIN, MAP_DOMAIN),
            Vec2.random(-MAP_DOMAIN, MAP_DOMAIN)));
    }

    public getObstacles(): Obstacle[] {
        /* TODO, TO SOMETHING ABOUNT NOT RETURNING REFERENCES */
        return this.obstacles;
    }

    public getLidarData(): LidarScan {
        const scan: LidarScan = {minAngle: -ANGLE_VIEW / 2, maxAngle: ANGLE_VIEW / 2, angleStep: ANGLE_VIEW / NUM_RAYS, distances: []};
        const currentPosition = this.getWorldPosition();
        for(let i = 0; i < NUM_RAYS; i++){
            const currentAngle = currentPosition.angular + scan.minAngle + i * scan.angleStep;
            const ray: Ray = {origin: currentPosition.linear.copy(), dir: Vec2.fromAngle(currentAngle)};
            scan.distances.push(this.getCollision(ray));
        }
        return scan;
    }

    private getCollision(ray: Ray): number{
        let minDistance = Infinity;
        for(const obs of this.obstacles){
            const collision = obs.computeCollision(ray);
            const distance = collision.position.getSub(ray.origin).length();
            if(!collision.valid || distance > minDistance) continue;
            minDistance = distance;
        }
        if(minDistance == Infinity) return ERROR_DISTANCE;
        return minDistance;
    }
}