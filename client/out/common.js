import { EPSILON } from "./constants.js";
export function getRandom() {
    return (Math.random() * 2) - 1;
}
export function floateq(x, y, epsilon = EPSILON) {
    return Math.abs(y - x) <= epsilon;
}
