export class Vec2 {
    constructor(x = 0, y = 0) {
        this.x = 0;
        this.y = 0;
        this.x = x;
        this.y = y;
    }
    getAdd(vect) {
        return new Vec2(this.x + vect.x, this.y + vect.y);
    }
    getSub(vect) {
        return new Vec2(this.x - vect.x, this.y - vect.y);
    }
    add(vect) {
        this.x += vect.x;
        this.y += vect.y;
        return this;
    }
    sub(vect) {
        this.x -= vect.x;
        this.y -= vect.y;
        return this;
    }
    dot(vect) {
        return this.x * vect.x + this.y * vect.y;
    }
    mult(factor) {
        this.x *= factor;
        this.y *= factor;
        return this;
    }
    getMult(factor) {
        const vect = this.copy();
        vect.mult(factor);
        return vect;
    }
    length() {
        return Math.sqrt(this.x * this.x + this.y * this.y);
    }
    getUnit() {
        const length = this.length();
        if (length == 0) {
            console.log("ASKING UNIT VECTOR ON VECTOR OF LENGTH 0, ABORT");
            return new Vec2();
        }
        return new Vec2(this.x / length, this.y / length);
    }
    rotate(angle) {
        /*
        [cos(t) -sin(t)][x] = [x cos(t) - y sin(t)]
        [sin(t)  cos(t)][y] = [x sin(t) + y cos(t)]
        */
        const previousX = this.x;
        this.x = this.x * Math.cos(angle) - this.y * Math.sin(angle);
        this.y = previousX * Math.sin(angle) + this.y * Math.cos(angle);
        return this;
    }
    getAngle() {
        return Math.atan2(this.y, this.x);
    }
    copy() {
        return new Vec2(this.x, this.y);
    }
    static fromAngle(angle) {
        return new Vec2(Math.cos(angle), Math.sin(angle));
    }
    static random(low, high) {
        const first = (Math.random() * (high - low)) + low;
        const second = (Math.random() * (high - low)) + low;
        return new Vec2(first, second);
    }
}
