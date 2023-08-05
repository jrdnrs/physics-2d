export const HALF_PI = Math.PI / 2;
export const PI = Math.PI;
export const TAU = Math.PI * 2;
export const DEG_TO_RAD = Math.PI / 180;
export const RAD_TO_DEG = 180 / Math.PI;
export const RECIP_36 = 1 / 36;
export const RECIP_24 = 1 / 24;
export const RECIP_12 = 1 / 12;
export const RECIP_6 = 1 / 6;
export const RECIP_4 = 1 / 4;
export const RECIP_2 = 1 / 2;

export function roundDown(x: number, places: number): number {
    const p = Math.pow(10, places);
    return Math.floor(x * p) / p;
}

export function round(x: number, places: number): number {
    const p = Math.pow(10, places);
    return Math.round(x * p) / p;
}

export function roundUp(x: number, places: number): number {
    const p = Math.pow(10, places);
    return Math.ceil(x * p) / p;
}

export function clamp(x: number, min: number, max: number): number {
    return Math.min(Math.max(min, x), max);
}

export function toRadians(degrees: number): number {
    return degrees * DEG_TO_RAD;
}

export function toDegrees(radians: number): number {
    return radians * RAD_TO_DEG;
}

export function average(values: number[]): number {
    return (
        values.reduce((prev, curr) => {
            return (curr += prev);
        }) / values.length
    );
}

export function lerp(start: number, end: number, t: number): number {
    return start + (end - start) * t;
}

export function fullLerp(y1: number, y2: number, x1: number, x2: number, xStart: number, xStop: number): number[] {
    const yDelta = y2 - y1;
    const xDelta = x2 - x1;
    const m = yDelta / xDelta;
    let y = y1 + (xStart - x1) * m;
    let v = [y];
    for (let x = xStart; x < xStop; x++) {
        v.push((y += m));
    }
    return v;
}

export function* fullLerpLazy(
    y1: number,
    y2: number,
    x1: number,
    x2: number,
    xStart: number,
    xStop: number
): Generator<number> {
    const yDelta = y2 - y1;
    const xDelta = x2 - x1;
    const m = yDelta / xDelta;
    let y = y1 + (xStart - x1) * m;
    yield y;
    for (let x = xStart; x < xStop; x++) {
        yield (y += m);
    }
}
