import type { Polygon } from "./poly";
import Vec3 from "./vec3";
import Segment from "./segment";
import { lerp } from "./util";

export default class Vec2 {
    x: number;
    y: number;

    constructor(x: number, y: number) {
        this.x = x;
        this.y = y;
    }

    static zero(): Vec2 {
        return new Vec2(0, 0);
    }

    static one(): Vec2 {
        return new Vec2(1, 1);
    }

    static fromVec3(v: Vec3): Vec2 {
        return new Vec2(v.x, v.y);
    }

    static perpendicular(v: Vec2): Vec2 {
        return new Vec2(-v.y, v.x);
    }

    isNaN(): boolean {
        return isNaN(this.x) || isNaN(this.y);
    }

    isZero(): boolean {
        return this.x === 0 && this.y === 0;
    }

    equals(v: Vec2): boolean {
        return this.x === v.x && this.y === v.y;
    }

    nearEquals(v: Vec2, epsilon: number): boolean {
        return Math.abs(this.x - v.x) < epsilon && Math.abs(this.y - v.y) < epsilon;
    }

    /**
     * Returns a new deep copy
     */
    clone(): Vec2 {
        return new Vec2(this.x, this.y);
    }

    set(v: Vec2): this {
        this.x = v.x;
        this.y = v.y;
        return this;
    }

    static lerp(lhs: Vec2, rhs: Vec2, t: number): Vec2 {
        return lhs.clone().lerp(rhs, t);
    }

    lerp(rhs: Vec2, t: number): this {
        this.x = lerp(this.x, rhs.x, t);
        this.y = lerp(this.y, rhs.y, t);
        return this;
    }

    magnitude(): number {
        return Math.sqrt(this.x * this.x + this.y * this.y);
    }

    magnitudeSquared(): number {
        return this.x * this.x + this.y * this.y;
    }

    static normalise(v: Vec2): Vec2 {
        return v.clone().normalise();
    }

    normalise(): this {
        const m = this.magnitude();
        if (m > 0) {
            this.x /= m;
            this.y /= m;
        }
        return this;
    }

    static dot(lhs: Vec2, rhs: Vec2): number {
        return lhs.dot(rhs);
    }

    dot(rhs: Vec2): number {
        return this.x * rhs.x + this.y * rhs.y;
    }

    static cross(lhs: Vec2, rhs: Vec2): number {
        return lhs.cross(rhs);
    }

    cross(rhs: Vec2): number {
        return this.x * rhs.y - this.y * rhs.x;
    }

    static crossScalar(v: Vec2, s: number): number {
        return v.clone().crossScalar(s);
    }

    crossScalar(s: number): number {
        return this.x * s - this.y * s;
    }

    /**
     * Returns the triple cross product of three vectors and returns the result as a Vec2,
     * where the intermediate result is a Vec3
     *
     * (v1 x v2) x v3
     */
    static tripleCross(v1: Vec2, v2: Vec2, v3: Vec2): Vec2 {
        const _v1 = new Vec3(v1.x, v1.y, 0);
        const _v2 = new Vec3(v2.x, v2.y, 0);
        const _v3 = new Vec3(v3.x, v3.y, 0);

        return _v1.cross(_v2).cross(_v3).intoVec2();
    }

    static add(lhs: Vec2, rhs: Vec2): Vec2 {
        return lhs.clone().add(rhs);
    }

    add(rhs: Vec2): this {
        this.x += rhs.x;
        this.y += rhs.y;
        return this;
    }

    static sub(lhs: Vec2, rhs: Vec2): Vec2 {
        return lhs.clone().sub(rhs);
    }

    sub(rhs: Vec2): this {
        this.x -= rhs.x;
        this.y -= rhs.y;
        return this;
    }

    static mul(lhs: Vec2, rhs: Vec2): Vec2 {
        return lhs.clone().mul(rhs);
    }

    mul(rhs: Vec2): this {
        this.x *= rhs.x;
        this.y *= rhs.y;
        return this;
    }

    static div(lhs: Vec2, rhs: Vec2): Vec2 {
        return lhs.clone().div(rhs);
    }

    div(rhs: Vec2): this {
        this.x /= rhs.x;
        this.y /= rhs.y;
        return this;
    }

    static addScalar(lhs: Vec2, rhs: number): Vec2 {
        return lhs.clone().addScalar(rhs);
    }

    addScalar(rhs: number): this {
        this.x += rhs;
        this.y += rhs;
        return this;
    }

    static subScalar(lhs: Vec2, rhs: number): Vec2 {
        return lhs.clone().subScalar(rhs);
    }

    subScalar(rhs: number): this {
        this.x -= rhs;
        this.y -= rhs;
        return this;
    }

    static mulScalar(lhs: Vec2, rhs: number): Vec2 {
        return lhs.clone().mulScalar(rhs);
    }

    mulScalar(rhs: number): this {
        this.x *= rhs;
        this.y *= rhs;
        return this;
    }

    static divScalar(lhs: Vec2, rhs: number): Vec2 {
        return lhs.clone().divScalar(rhs);
    }

    divScalar(rhs: number): this {
        this.x /= rhs;
        this.y /= rhs;
        return this;
    }

    static sqrt(v: Vec2): Vec2 {
        return v.clone().sqrt();
    }

    sqrt(): this {
        this.x = Math.sqrt(this.x);
        this.y = Math.sqrt(this.y);
        return this;
    }

    square(): this {
        this.x *= this.x;
        this.y *= this.y;
        return this;
    }

    static neg(v: Vec2): Vec2 {
        return v.clone().neg();
    }

    neg(): this {
        this.x = -this.x;
        this.y = -this.y;
        return this;
    }

    static abs(v: Vec2): Vec2 {
        return v.clone().abs();
    }

    abs(): this {
        this.x = Math.abs(this.x);
        this.y = Math.abs(this.y);
        return this;
    }

    static round(v: Vec2): Vec2 {
        return v.clone().round();
    }

    round(): this {
        this.x = Math.round(this.x);
        this.y = Math.round(this.y);
        return this;
    }

    static rotate(
        v: Vec2,
        rad: number,
        origin: Vec2 = Vec2.zero(),
        sin: number = Math.sin(rad),
        cos: number = Math.cos(rad)
    ): Vec2 {
        return v.clone().rotate(rad, origin, sin, cos);
    }

    rotate(rad: number, origin: Vec2 = Vec2.zero(), sin: number = Math.sin(rad), cos: number = Math.cos(rad)): this {
        this.sub(origin);

        const x = this.x;
        const y = this.y;

        this.x = cos * x - sin * y;
        this.y = sin * x + cos * y;

        this.add(origin);

        return this;
    }

    perpendicular(): this {
        [this.x, this.y] = [-this.y, this.x];
        return this;
    }

    /**
     * Returns `true` if _this_ segment is inside of the rect defined by the
     * provided args, and `false` otherwise
     */
    inBounds(xMin: number, xMax: number, yMin: number, yMax: number): boolean {
        return this.x >= xMin && this.x <= xMax && this.y >= yMin && this.y <= yMax;
    }

    /**
     * Returns `true` if _this_ point is inside the provided polygon, otherwise returns `false`.
     * If _this_ point shares coordinates with one of the polygon's points, result can vary
     */
    inPolygon(poly: Polygon): boolean {
        // rough bounds check first
        if (!this.inBounds(poly.getMinX(), poly.getMaxX(), poly.getMinY(), poly.getMaxY())) {
            return false;
        }

        const ray = new Segment(this, new Vec2(0, Number.MAX_SAFE_INTEGER));
        let inside = false;

        for (const seg of poly.getSegments()) {
            if (seg.intersectsSeg(ray)) inside = !inside;
        }

        return inside;
    }
}
