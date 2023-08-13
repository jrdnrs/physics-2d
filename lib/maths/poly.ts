import Drawer from "../drawer/drawer";
import { StyleID } from "../drawer/style";
import Segment from "./segment";
import { Shape } from "./shape";
import { TAU } from "./util";
import Vec2 from "./vec2";

export class Polygon implements Shape {
    points: Vec2[];

    constructor(points: Vec2[]) {
        this.points = points;
    }

    static fromTriangle(centre: Vec2, base: number, height: number): Polygon {
        const halfBase = base * 0.5;
        const halfHeight = height * 0.5;
        return new Polygon([
            new Vec2(0, -halfHeight),
            new Vec2(-halfBase, halfHeight),
            new Vec2(halfBase, halfHeight),
        ]).translate(centre);
    }

    static fromRect(centre: Vec2, width: number, height: number): Polygon {
        const halfWidth = width * 0.5;
        const halfHeight = height * 0.5;
        return new Polygon([
            new Vec2(-halfWidth, -halfHeight),
            new Vec2(-halfWidth, halfHeight),
            new Vec2(halfWidth, halfHeight),
            new Vec2(halfWidth, -halfHeight),
        ]).translate(centre);
    }

    static fromNGon(centre: Vec2, radius: number, n: number): Polygon {
        if (n < 3) throw new Error("n must be >= 3");

        const points: Vec2[] = [];
        const angle = TAU / n;
        for (let i = 0; i < n; i++) {
            points.push(new Vec2(Math.cos(angle * i) * radius, Math.sin(angle * i) * radius));
        }
        return new Polygon(points).translate(centre);
    }

    draw(drawer: Drawer, style: StyleID) {
        drawer.drawPolygon(this.points, style);
    }

    getArea(): number {
        let area = 0;

        for (let i = 0; i < this.points.length; i++) {
            const p1 = this.points[i];
            const p2 = this.points[(i + 1) % this.points.length];
            area += p1.cross(p2);
        }

        return Math.abs(area) * 0.5;
    }

    clone(): Polygon {
        return new Polygon(this.points.map((p) => p.clone()));
    }

    getPoints(): Vec2[] {
        return this.points;
    }

    getSegments(): Segment[] {
        return Array.from(
            new Array(this.points.length),
            (_, i) => new Segment(this.points[i], this.points[(i + 1) % this.points.length])
        );
    }

    getMinX(): number {
        let min = Number.MAX_SAFE_INTEGER;
        for (const p of this.points) {
            min = Math.min(min, p.x);
        }
        return min;
    }

    getMinY(): number {
        let min = Number.MAX_SAFE_INTEGER;
        for (const p of this.points) {
            min = Math.min(min, p.y);
        }
        return min;
    }

    getMaxX(): number {
        let max = Number.MIN_SAFE_INTEGER;
        for (const p of this.points) {
            max = Math.max(max, p.x);
        }
        return max;
    }

    getMaxY(): number {
        let max = Number.MIN_SAFE_INTEGER;
        for (const p of this.points) {
            max = Math.max(max, p.y);
        }
        return max;
    }

    getCentre(): Vec2 {
        let x = 0;
        let y = 0;

        for (const p of this.points) {
            x += p.x;
            y += p.y;
        }

        return new Vec2(x / this.points.length, y / this.points.length);
    }

    setCentre(centre: Vec2): this {
        const delta = Vec2.sub(centre, this.getCentre());
        return this.translate(delta);
    }

    rotate(
        radians: number,
        origin: Vec2 = this.getCentre(),
        sin: number = Math.sin(radians),
        cos: number = Math.cos(radians)
    ): this {
        for (const p of this.points) {
            p.rotate(radians, origin, sin, cos);
        }

        return this;
    }

    translate(delta: Vec2): this {
        for (const p of this.points) {
            p.add(delta);
        }

        return this;
    }

    scale(scale: number, origin: Vec2 = this.getCentre()): this {
        for (const p of this.points) {
            p.sub(origin).mulScalar(scale).add(origin);
        }

        return this;
    }

    containsPoint(point: Vec2): boolean {
        return point.inPolygon(this);
    }

    getFurthestPoint(direction: Vec2): Vec2 {
        // The farthest point is the one with the largest projection onto the given direction
        let best = -Number.MAX_VALUE;
        let bestPoint = Vec2.zero();

        for (const point of this.points) {
            const projection = point.dot(direction);
            if (projection > best) {
                best = projection;
                bestPoint = point;
            }
        }

        return bestPoint.clone();
    }

    /**
     * Triangulates the polygon using fan triangulation, in O(n) time. However, this only
     * works for convex polygons.
     */
    fanTriangulate(): Polygon[] {
        const triangles: Polygon[] = [];
        for (let i = 1; i < this.points.length - 1; i++) {
            triangles.push(new Polygon([this.points[0], this.points[i], this.points[i + 1]]));
        }
        return triangles;
    }
}
