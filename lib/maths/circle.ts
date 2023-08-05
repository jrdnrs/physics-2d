import Drawer from "../drawer/drawer";
import { StyleID } from "../drawer/style";
import { Shape } from "./shape";
import Vec2 from "./vec2";

export class Circle implements Shape {
    centre: Vec2;
    radius: number;

    constructor(centre: Vec2, radius: number) {
        this.centre = centre;
        this.radius = radius;
    }

    draw(drawer: Drawer, style: StyleID) {
        drawer.drawCircle(this.centre, this.radius, style);
    }

    getArea(): number {
        return Math.PI * this.radius * this.radius;
    }

    clone(): Circle {
        return new Circle(this.centre.clone(), this.radius);
    }

    containsPoint(point: Vec2): boolean {
        return Vec2.sub(point, this.centre).magnitudeSquared() <= this.radius * this.radius;
    }

    getMinX(): number {
        return this.centre.x - this.radius;
    }

    getMinY(): number {
        return this.centre.y - this.radius;
    }

    getMaxX(): number {
        return this.centre.x + this.radius;
    }

    getMaxY(): number {
        return this.centre.y + this.radius;
    }

    setCentre(centre: Vec2): this {
        this.centre = centre.clone();
        return this;
    }

    getCentre(): Vec2 {
        return this.centre;
    }

    rotate(radians: number, origin?: Vec2, sin?: number, cos?: number): this {
        if (origin !== undefined) {
            this.centre.rotate(radians, origin);
        }
        return this;
    }

    translate(delta: Vec2): this {
        this.centre.add(delta);
        return this;
    }

    scale(scale: number, origin = this.centre): this {
        this.centre.sub(origin).mulScalar(scale).add(origin);
        this.radius *= scale;
        return this;
    }

    getFurthestPoint(direction: Vec2): Vec2 {
        return Vec2.add(this.centre, Vec2.mulScalar(direction, this.radius));
    }
}
