import { EPSILON } from "./constants.js";

export function getRandom(){
    return (Math.random() * 2) - 1;
}

export function getRandomBound(low: number, high: number){
    return Math.random() * (high - low) + low;
}

export function floateq(x: number, y: number, epsilon = EPSILON){
    return Math.abs(y - x) <= epsilon;
}