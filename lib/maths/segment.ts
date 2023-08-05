import Vec2 from "./vec2";

const EPSILON = 1e-6;

export default class Segment {
    p1: Vec2;
    p2: Vec2;

    constructor(p1: Vec2, p2: Vec2) {
        this.p1 = p1;
        this.p2 = p2;
    }

    static zero(): Segment {
        return new Segment(Vec2.zero(), Vec2.zero());
    }

    isNaN(): boolean {
        return this.p1.isNaN() || this.p2.isNaN();
    }

    /**
     * Returns a new deep copy
     */
    clone(): Segment {
        return new Segment(this.p1.clone(), this.p2.clone());
    }

    getMinX(): number {
        return Math.min(this.p1.x, this.p2.x);
    }

    getMinY(): number {
        return Math.min(this.p1.y, this.p2.y);
    }

    getMaxX(): number {
        return Math.max(this.p1.x, this.p2.x);
    }

    getMaxY(): number {
        return Math.max(this.p1.y, this.p2.y);
    }

    edge(): Vec2 {
        return Vec2.sub(this.p2, this.p1);
    }

    /**
     * Round _this_ segment's coordinates to the nearest int
     */
    round(): this {
        this.p1.round();
        this.p2.round();
        return this;
    }

    rotate(rad: number, origin: Vec2 = Vec2.zero(), sin: number = Math.sin(rad), cos: number = Math.cos(rad)): this {
        this.p1.rotate(rad, origin, sin, cos);
        this.p2.rotate(rad, origin, sin, cos);
        return this;
    }

    normal(): Vec2 {
        const ab = Vec2.sub(this.p2, this.p1);
        const ao = Vec2.neg(this.p1);

        return Vec2.tripleCross(ab, ao, ab);
    }

    length(): number {
        return Vec2.sub(this.p2, this.p1).magnitude();
    }

    lengthSquared(): number {
        return Vec2.sub(this.p2, this.p1).magnitudeSquared();
    }

    distanceToOrigin(): number {
        return Math.abs(this.normal().dot(this.p1));
    }

    static clip(segment: Segment, point: Vec2, direction: Vec2): Segment {
        return segment.clone().clip(point, direction);
    }

    /**
     * Clips this segment against a line that is defined by a point and a half-space direction.
     *
     * The line runs through the given point and is perpendicular to the given direction.
     * The direction then defines a half-space, and the segment is clipped against that half-space,
     * where everything on the "positive" side of the line is kept, and everything on the "negative"
     * side is discarded.
     *
     * @param point The point through which the line runs
     * @param direction The direction of the half-space (normalised)
     */
    clip(point: Vec2, direction: Vec2): this {
        // This projection represents a distance along the direction vector, which acts as a threshold
        const D = direction.dot(point);

        // These are the signed distances along the direction vector (relative to the threshold)
        const d1 = direction.dot(this.p1) - D + EPSILON;
        const d2 = direction.dot(this.p2) - D + EPSILON;

        if (d1 >= 0 && d2 >= 0) {
            // Both points are on the positive side of the line, so keep the whole segment
            return this;
        }

        if (d1 < 0 && d2 < 0) {
            // Both points are on the negative side of the line, so discard the whole segment
            this.p1.x = NaN;
            this.p1.y = NaN;
            this.p2.x = NaN;
            this.p2.y = NaN;
            return this;
        }

        // One point is on the positive side, and the other is on the negative side, so clip the segment
        // by using the interpolating factor derived from the signed distances along the direction vector.
        if (d1 < 0) {
            this.p1.lerp(this.p2, d1 / (d1 - d2));
        } else {
            this.p2.lerp(this.p1, d2 / (d2 - d1));
        }

        return this;
    }

    /**
     * Returns `true` if _this_ and the `other` segment intersect at some point, and `false` otherwise.
     * Prefer this over `getIntersection()` if you don't actually need the point
     */
    // https://en.wikipedia.org/wiki/Line-line_intersection#Given_two_points_on_each_line_segment
    intersectsSeg(other: Segment): boolean {
        const x1 = this.p1.x;
        const y1 = this.p1.y;
        const x2 = this.p2.x;
        const y2 = this.p2.y;
        const x3 = other.p1.x;
        const y3 = other.p1.y;
        const x4 = other.p2.x;
        const y4 = other.p2.y;

        const denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);

        // den will be 0 when parallel
        if (denom === 0) return false;

        const t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom;
        const u = ((x1 - x3) * (y1 - y2) - (y1 - y3) * (x1 - x2)) / denom;

        return t >= 0 && t <= 1 && u >= 0 && u <= 1;
    }

    /**
     * If _this_ and the `other` segment intersect at some point, the point will be returned,
     * otherwise returns `undefined`
     */
    getIntersectionSeg(other: Segment): Vec2 | undefined {
        const x1 = this.p1.x;
        const y1 = this.p1.y;
        const x2 = this.p2.x;
        const y2 = this.p2.y;
        const x3 = other.p1.x;
        const y3 = other.p1.y;
        const x4 = other.p2.x;
        const y4 = other.p2.y;

        const denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);

        if (denom === 0) return undefined;

        const t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom;
        const u = ((x1 - x3) * (y1 - y2) - (y1 - y3) * (x1 - x2)) / denom;

        if (t < 0 || t > 1 || u < 0 || u > 1) return undefined;

        return new Vec2(x1 + t * (x2 - x1), y1 + t * (y2 - y1));
    }
}
