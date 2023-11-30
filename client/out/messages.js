import { Vec2 } from "./vec2.js";
export class Twist {
    constructor(linear = new Vec2(), angular = 0) {
        this.linear = linear;
        this.angular = angular;
    }
    copy() {
        return new Twist(this.linear.copy(), this.angular);
    }
}
