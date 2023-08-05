import Drawer from "../drawer/drawer";
import { StyleID } from "../drawer/style";
import Vec2 from "./vec2";

export interface Shape {
    clone(): Shape;
    containsPoint(point: Vec2): boolean;
    getMinX(): number;
    getMinY(): number;
    getMaxX(): number;
    getMaxY(): number;
    setCentre(centre: Vec2): this;
    getCentre(): Vec2;
    rotate(radians: number, origin?: Vec2, sin?: number, cos?: number): this;
    translate(delta: Vec2): this;
    scale(scale: number, origin?: Vec2): this;
    getFurthestPoint(direction: Vec2): Vec2;
    getArea(): number;
    draw(drawer: Drawer, style: StyleID): void;
}
