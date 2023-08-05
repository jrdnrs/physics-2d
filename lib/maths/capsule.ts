import Drawer from "../drawer/drawer";
import { StyleID } from "../drawer/style";
import { Circle } from "./circle";
import { Polygon } from "./poly";
import { Shape } from "./shape";
import Vec2 from "./vec2";

export class Capsule implements Shape {
    rectangle: Polygon;
    circles: [Circle, Circle];
    length: number;
    angle: number;

    constructor(centre: Vec2, radius: number, length: number) {
        const halfLength = length * 0.5;

        this.rectangle = new Polygon([
            new Vec2(centre.x - halfLength, centre.y - radius),
            new Vec2(centre.x + halfLength, centre.y - radius),
            new Vec2(centre.x + halfLength, centre.y + radius),
            new Vec2(centre.x - halfLength, centre.y + radius),
        ]);
        this.circles = [
            new Circle(new Vec2(centre.x + halfLength, centre.y), radius),
            new Circle(new Vec2(centre.x - halfLength, centre.y), radius),
        ];
        this.length = length;
        this.angle = 0;
    }

    draw(drawer: Drawer, style: StyleID): void {
        drawer.drawCapsule(
            this.rectangle.points,
            [this.circles[0].centre, this.circles[1].centre],
            this.circles[0].radius,
            this.angle,
            style
        );
    }

    getFurthestPoint(direction: Vec2): Vec2 {
        // Start at one of the circle's centre points, and add radius in the given direction.
        return Vec2.add(
            Vec2.dot(direction, Vec2.sub(this.circles[1].centre, this.circles[0].centre)) >= 0
                ? this.circles[1].centre
                : this.circles[0].centre,
            Vec2.mulScalar(direction, this.circles[0].radius)
        );
    }

    getArea(): number {
        const radius = this.circles[0].radius;
        return Math.PI * radius * radius + this.length * radius * 2;
    }

    clone(): Shape {
        return new Capsule(this.getCentre(), this.circles[0].radius, this.length);
    }

    containsPoint(point: Vec2): boolean {
        return (
            this.circles[0].containsPoint(point) ||
            this.circles[1].containsPoint(point) ||
            this.rectangle.containsPoint(point)
        );
    }

    getMinX(): number {
        return Math.min(this.circles[0].getMinX(), this.circles[1].getMinX());
    }

    getMinY(): number {
        return Math.min(this.circles[0].getMinY(), this.circles[1].getMinY());
    }

    getMaxX(): number {
        return Math.max(this.circles[0].getMaxX(), this.circles[1].getMaxX());
    }

    getMaxY(): number {
        return Math.max(this.circles[0].getMaxY(), this.circles[1].getMaxY());
    }

    setCentre(centre: Vec2): this {
        const delta = Vec2.sub(centre, this.getCentre());
        this.translate(delta);

        return this;
    }

    getCentre(): Vec2 {
        return Vec2.add(this.circles[0].centre, this.circles[1].centre).mulScalar(0.5);
    }

    rotate(radians: number, origin: Vec2 = this.getCentre()): this {
        const sin = Math.sin(radians);
        const cos = Math.cos(radians);
        this.rectangle.rotate(radians, origin, sin, cos);
        this.circles[0].rotate(radians, origin, sin, cos);
        this.circles[1].rotate(radians, origin, sin, cos);
        this.angle += radians;

        return this;
    }

    translate(delta: Vec2): this {
        this.rectangle.translate(delta);
        this.circles[0].translate(delta);
        this.circles[1].translate(delta);

        return this;
    }

    scale(scale: number, origin: Vec2 = this.getCentre()): this {
        this.rectangle.scale(scale, origin);
        this.circles[0].scale(scale, origin);
        this.circles[1].scale(scale, origin);

        return this;
    }
}
