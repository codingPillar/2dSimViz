import { floateq } from "./common.js";
import { ANGLE_VIEW, DEFAULT_OBSTACLE_COUNT, ERROR_DISTANCE, MAP_DOMAIN, NUM_RAYS, dt } from "./constants.js";
import { Vec2 } from "./vec2.js";
import { Twist } from "./messages.js";
class LineObs {
    constructor(first, second) {
        this.first = first;
        this.second = second;
    }
    computeCollision(ray) {
        const dir = this.second.getSub(this.first).getUnit();
        ray.dir = ray.dir.getUnit();
        const sameDir = floateq(dir.x, ray.dir.x) && floateq(dir.x, ray.dir.x);
        /* LINES ARE PARALLEL RETURN FALSE */
        if (sameDir)
            return { valid: false, position: new Vec2() };
        let k2 = 0;
        if (floateq(dir.x, 0) && !floateq(ray.dir.x, 0))
            k2 = (this.first.x - ray.origin.x) / ray.dir.x;
        else if (!floateq(dir.x, 0))
            k2 = ((this.first.y - ray.origin.y) + (ray.origin.x - this.first.x) * dir.y / dir.x)
                / (ray.dir.y - (ray.dir.x * dir.y / dir.x));
        const final = ray.origin.copy().add(ray.dir.getMult(k2));
        if (this.inBound(final) && k2 > 0)
            return { valid: true, position: final };
        return { valid: false, position: new Vec2() };
    }
    getSegments() {
        return [this.first, this.second];
    }
    inBound(position) {
        const maxLength = this.second.getSub(this.first).length();
        const flength = position.getSub(this.first).length();
        const slength = position.getSub(this.second).length();
        if (flength > maxLength || slength > maxLength)
            return false;
        return true;
    }
}
/* APP MODEL STATE CLASS */
export class Model {
    constructor() {
        /* MAP STATE */
        this.domain = new Vec2(-MAP_DOMAIN, MAP_DOMAIN);
        this.image = new Vec2(-MAP_DOMAIN, MAP_DOMAIN);
        /* ROBOT STATES */
        /* THIS WOULD BE THE ROBOT'S ODOM VALUE */
        this.position = new Twist();
        /* IN CURRENT MODE, ONLY FOUR_DIFF IS SUPPORTED (TODO, ADD SUPPORT, FOR MECANUM DRIVE) */
        this.velocity = new Twist();
        /* IN MAP COORDINATE SYSTEM */
        this.initialPosition = new Twist();
        /* OBSTACLES */
        this.obstacles = [];
        for (let i = 0; i < DEFAULT_OBSTACLE_COUNT; i++)
            this.addRandomObstacle();
    }
    updateSim() {
        /* FOUR DIFF */
        const displacement = Vec2.fromAngle(this.position.angular).mult(this.velocity.linear.x * dt);
        this.position.linear.add(displacement);
        this.position.angular += this.velocity.angular * dt;
    }
    setInitialPosition(position, orientation) {
        this.initialPosition.linear = position;
        this.initialPosition.angular = orientation;
    }
    getInitialPosition() {
        return this.initialPosition.copy();
    }
    getOdomPosition() {
        return this.position.copy();
    }
    getWorldPosition() {
        /* ROBOT ODOM IN MAP SPACE */
        const robotBaseX = Vec2.fromAngle(this.initialPosition.angular);
        const robotBaseY = Vec2.fromAngle(this.initialPosition.angular + Math.PI / 2);
        return new Twist(this.initialPosition.linear.getAdd(robotBaseX.mult(this.position.linear.x).add(robotBaseY.mult(this.position.linear.y))), this.position.angular + this.initialPosition.angular);
    }
    addRandomObstacle() {
        /* TODO, MAKE IT DEPEND ON THE DOMAIN AND IMAGE OF MAP */
        this.obstacles.push(new LineObs(Vec2.random(-MAP_DOMAIN, MAP_DOMAIN), Vec2.random(-MAP_DOMAIN, MAP_DOMAIN)));
    }
    getObstacles() {
        /* TODO, TO SOMETHING ABOUNT NOT RETURNING REFERENCES */
        return this.obstacles;
    }
    getLidarData() {
        const scan = { minAngle: -ANGLE_VIEW / 2, maxAngle: ANGLE_VIEW / 2, angleStep: ANGLE_VIEW / NUM_RAYS, distances: [] };
        const currentPosition = this.getWorldPosition();
        for (let i = 0; i < NUM_RAYS; i++) {
            const currentAngle = currentPosition.angular + scan.minAngle + i * scan.angleStep;
            const ray = { origin: currentPosition.linear.copy(), dir: Vec2.fromAngle(currentAngle) };
            scan.distances.push(this.getCollision(ray));
        }
        return scan;
    }
    getCollision(ray) {
        let minDistance = Infinity;
        for (const obs of this.obstacles) {
            const collision = obs.computeCollision(ray);
            const distance = collision.position.getSub(ray.origin).length();
            if (!collision.valid || distance > minDistance)
                continue;
            minDistance = distance;
        }
        if (minDistance == Infinity)
            return ERROR_DISTANCE;
        return minDistance;
    }
}
