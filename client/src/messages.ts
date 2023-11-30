import { Vec2 } from "./vec2.js";

export class Twist{
    linear: Vec2;
    angular: number;

    constructor(linear: Vec2 = new Vec2(), angular: number = 0){
        this.linear = linear;
        this.angular = angular;
    }

    public copy(){
        return new Twist(this.linear.copy(), this.angular);
    }
}

export interface LidarScan{
    minAngle: number;
    maxAngle: number;
    angleStep: number;
    distances: number[];
}