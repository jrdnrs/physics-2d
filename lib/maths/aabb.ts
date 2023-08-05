import Drawer from "../drawer/drawer";
import { StyleID } from "../drawer/style";
import { Shape } from "./shape";
import Vec2 from "./vec2";

export class AABB {
    min: Vec2;
    max: Vec2;

    constructor(min: Vec2, max: Vec2) {
        this.min = min;
        this.max = max;
    }

    static fromDimensions(x: number, y: number, width: number, height: number): AABB {
        return new AABB(new Vec2(x, y), new Vec2(x + width, y + height));
    }

    static fromShape(shape: Shape): AABB {
        const minX = shape.getMinX();
        const minY = shape.getMinY();
        const maxX = shape.getMaxX();
        const maxY = shape.getMaxY();

        return new AABB(new Vec2(minX, minY), new Vec2(maxX, maxY));
    }

    clone(): AABB {
        return new AABB(this.min.clone(), this.max.clone());
    }

    draw(drawer: Drawer, style: StyleID) {
        drawer.drawRect(this.min, this.max, style);
    }

    translate(delta: Vec2): this {
        this.min.add(delta);
        this.max.add(delta);
        return this;
    }

    getCentre(): Vec2 {
        const halfWidth = (this.max.x - this.min.x) * 0.5;
        const halfHeight = (this.max.y - this.min.y) * 0.5;

        return new Vec2(this.min.x + halfWidth, this.min.y + halfHeight);
    }

    setCentre(centre: Vec2): this {
        const halfWidth = (this.max.x - this.min.x) * 0.5;
        const halfHeight = (this.max.y - this.min.y) * 0.5;

        this.min.x = centre.x - halfWidth;
        this.min.y = centre.y - halfHeight;
        this.max.x = centre.x + halfWidth;
        this.max.y = centre.y + halfHeight;

        return this;
    }

    getArea(): number {
        return (this.max.x - this.min.x) * (this.max.y - this.min.y);
    }

    containsPoint(point: Vec2): boolean {
        return point.x >= this.min.x && point.x <= this.max.x && point.y >= this.min.y && point.y <= this.max.y;
    }

    intersects(other: AABB): boolean {
        return (
            this.min.x < other.max.x && this.max.x > other.min.x && this.min.y < other.max.y && this.max.y > other.min.y
        );
    }

    contains(other: AABB): boolean {
        return (
            other.min.x >= this.min.x &&
            other.max.x <= this.max.x &&
            other.min.y >= this.min.y &&
            other.max.y <= this.max.y
        );
    }
}
