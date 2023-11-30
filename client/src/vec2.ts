import { getRandom } from "./common.js";

export class Vec2{
    x: number = 0;
    y: number = 0;
    
    constructor(x: number = 0, y: number = 0){
        this.x = x;
        this.y = y;
    }

    public getAdd(vect: Vec2) : Vec2 {
        return new Vec2(this.x + vect.x, this.y + vect.y);
    }

    public getSub(vect: Vec2) : Vec2 {
        return new Vec2(this.x - vect.x, this.y - vect.y);
    }

    public add(vect: Vec2) : Vec2 {
        this.x += vect.x;
        this.y += vect.y;
        return this;
    }

    public sub(vect: Vec2) : Vec2 {
        this.x -= vect.x;
        this.y -= vect.y;
        return this;
    }

    public dot(vect: Vec2): number {
        return this.x * vect.x + this.y * vect.y;
    }

    public mult(factor: number): Vec2 {
        this.x *= factor;
        this.y *= factor;
        return this;
    }

    public getMult(factor: number): Vec2 {
        const vect = this.copy();
        vect.mult(factor);
        return vect;
    }

    public length(): number {
        return Math.sqrt(this.x * this.x + this.y * this.y);
    }

    public getUnit(): Vec2 {
        const length = this.length();
        if(length == 0){
            console.log("ASKING UNIT VECTOR ON VECTOR OF LENGTH 0, ABORT");
            return new Vec2();
        }
        return new Vec2(this.x / length, this.y / length);
    }

    public rotate(angle: number): Vec2 {
        /*
        [cos(t) -sin(t)][x] = [x cos(t) - y sin(t)]
        [sin(t)  cos(t)][y] = [x sin(t) + y cos(t)]
        */
       const previousX = this.x;
       this.x = this.x * Math.cos(angle) - this.y * Math.sin(angle);
       this.y = previousX * Math.sin(angle) + this.y * Math.cos(angle);
       return this;
    }

    public getAngle(): number{
        return Math.atan2(this.y, this.x);
    }

    public copy(){
        return new Vec2(this.x, this.y);
    }

    public static fromAngle(angle: number): Vec2 {
        return new Vec2(Math.cos(angle), Math.sin(angle));
    }

    public static random(low: number, high: number): Vec2 {
        const first = (Math.random() * (high - low)) + low;
        const second = (Math.random() * (high - low)) + low;
        return new Vec2(first, second);
    }
}