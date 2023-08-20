import { AABB } from "../lib/maths/aabb";
import { Capsule } from "../lib/maths/capsule";
import { Circle } from "../lib/maths/circle";
import { Mat2 } from "../lib/maths/mat2";
import { Polygon } from "../lib/maths/poly";
import { Shape } from "../lib/maths/shape";
import { RECIP_12, RECIP_4, PI, RECIP_2, RECIP_36 } from "../lib/maths/util";
import Vec2 from "../lib/maths/vec2";
import { Island } from "./island";

export class RigidBody {
    static nextID = 0;
    readonly id: number;

    // state
    timeStill: number;
    sleeping: boolean;
    fixed: boolean;
    island: Island | undefined;

    // shape
    collider: Shape;
    bounds: AABB;
    position: Vec2;
    rotation: number;

    // material
    restitution: number;
    friction: number;
    density: number;

    // linear motion
    mass: number;
    inverseMass: number;
    linearAcceleration: Vec2;
    linearVelocity: Vec2;
    // angular motion
    angularMass: number;
    inverseAngularMass: number;
    angularAcceleration: number;
    angularVelocity: number;

    constructor(
        collider: Shape,
        position: Vec2,
        mass: number,
        angularMass: number,
        restitution: number,
        friction: number,
        fixed: boolean
    ) {
        this.id = RigidBody.nextID++;
        this.timeStill = 0;
        this.sleeping = false;
        this.fixed = fixed;
        this.island = undefined;
        this.collider = collider.setCentre(position);
        this.bounds = AABB.fromShape(collider);
        this.position = position;
        this.rotation = 0;
        this.restitution = restitution;
        this.friction = friction;
        this.density = mass / collider.getArea();
        this.mass = mass;
        this.inverseMass = 1 / mass;
        this.linearAcceleration = Vec2.zero();
        this.linearVelocity = Vec2.zero();
        this.angularMass = angularMass;
        this.inverseAngularMass = 1 / this.angularMass;
        this.angularAcceleration = 0;
        this.angularVelocity = 0;

        if (this.fixed) {
            this.inverseMass = 0;
            this.inverseAngularMass = 0;
        }
    }

    static fromTriangle(
        base: number,
        height: number,
        position: Vec2,
        density: number,
        restitution: number,
        friction: number,
        fixed: boolean = false
    ): RigidBody {
        const collider = Polygon.fromTriangle(Vec2.zero(), base, height);
        const area = base * height * 0.5;
        const mass = area * density;
        const angularMass = triangleMMOI(collider.points[0], collider.points[1], collider.points[2]) * mass;
        return new RigidBody(collider, position, mass, angularMass, restitution, friction, fixed);
    }

    static fromRect(
        width: number,
        height: number,
        position: Vec2,
        density: number,
        restitution: number,
        friction: number,
        fixed: boolean = false
    ): RigidBody {
        const collider = Polygon.fromRect(Vec2.zero(), width, height);
        const area = width * height;
        const mass = area * density;
        const angularMass = rectangleMMOI(width, height) * mass;
        return new RigidBody(collider, position, mass, angularMass, restitution, friction, fixed);
    }

    static fromCircle(
        radius: number,
        position: Vec2,
        density: number,
        restitution: number,
        friction: number,
        fixed: boolean = false
    ): RigidBody {
        const collider = new Circle(Vec2.zero(), radius);
        const area = Math.PI * radius * radius;
        const mass = area * density;
        const angularMass = circleMMOI(radius) * mass;
        return new RigidBody(collider, position, mass, angularMass, restitution, friction, fixed);
    }

    static fromCapsule(
        radius: number,
        length: number,
        position: Vec2,
        density: number,
        restitution: number,
        friction: number,
        fixed: boolean = false
    ) {
        const collider = new Capsule(Vec2.zero(), radius, length);
        const area = Math.PI * radius * radius + length * radius * 2;
        const mass = area * density;
        const angularMass = capsuleMMOI(radius, length) * mass;
        return new RigidBody(collider, position, mass, angularMass, restitution, friction, fixed);
    }

    static fromConvexPolygon(
        points: Vec2[],
        position: Vec2,
        density: number,
        restitution: number,
        friction: number,
        fixed: boolean = false
    ) {
        const collider = new Polygon(points);
        const area = collider.getArea();
        const mass = area * density;
        const angularMass = convexPolygonMMOI(collider) * mass;
        return new RigidBody(collider, position, mass, angularMass, restitution, friction, fixed);
    }

    translate(delta: Vec2) {
        this.position.add(delta);
        this.collider.translate(delta);
        this.bounds.translate(delta);
    }

    rotate(delta: number) {
        this.rotation += delta;

        const sin = Math.sin(delta);
        const cos = Math.cos(delta);

        this.collider.rotate(delta, this.position, sin, cos);
        this.bounds.min.x = this.collider.getMinX();
        this.bounds.min.y = this.collider.getMinY();
        this.bounds.max.x = this.collider.getMaxX();
        this.bounds.max.y = this.collider.getMaxY();
    }

    getVelocityAtPoint(radius: Vec2): Vec2 {
        const tangent = Vec2.perpendicular(radius);
        const angularVelocity = Vec2.mulScalar(tangent, this.angularVelocity);
        return Vec2.add(this.linearVelocity, angularVelocity);
    }

    applyForce(force: Vec2, radius: Vec2 = Vec2.zero()) {
        this.linearAcceleration.add(Vec2.mulScalar(force, this.inverseMass));

        const torque = radius.cross(force);
        this.angularAcceleration += torque * this.inverseAngularMass;
    }

    applyImpulse(impulse: Vec2, radius: Vec2 = Vec2.zero()) {
        const linearDelta = Vec2.mulScalar(impulse, this.inverseMass);
        const angularDelta = radius.cross(impulse) * this.inverseAngularMass;

        this.linearVelocity.add(linearDelta);
        this.angularVelocity += angularDelta;
    }

    addToIsland(island: Island) {
        if (!this.fixed) {
            this.island = island;
            island.add(this);
        }
    }

    integrate(deltaSeconds: number) {
        if (this.fixed || this.sleeping) return;

        this.linearVelocity.add(Vec2.mulScalar(this.linearAcceleration, deltaSeconds));
        const translationDelta = Vec2.mulScalar(this.linearVelocity, deltaSeconds);
        this.translate(translationDelta);

        this.angularVelocity += this.angularAcceleration * deltaSeconds;
        const rotationDelta = this.angularVelocity * deltaSeconds;
        this.rotate(rotationDelta);

        const linearDecay = Math.exp(-deltaSeconds * this.friction);
        const angularDecay = Math.exp(-deltaSeconds * this.friction);
        this.linearVelocity.mulScalar(linearDecay);
        this.angularVelocity *= angularDecay;

        this.linearAcceleration.x = 0;
        this.linearAcceleration.y = 0;
        this.angularAcceleration = 0;
    }
}

function rectangleInertiaTensor(width: number, height: number): Mat2 {
    const iXX = height * height * RECIP_12;
    const iYY = width * width * RECIP_12;

    // prettier-ignore
    return new Mat2(
        iXX, 0,
        0,   iYY
    );
}

function circleInertiaTensor(radius: number): Mat2 {
    const i = radius * radius * RECIP_4;

    // prettier-ignore
    return new Mat2(
        i, 0,
        0, i
    );
}

function capsuleInertiaTensor(radius: number, length: number): Mat2 {
    // Mass of entire shape is one, so we need the proportion of masses
    const circleMassContribution = (PI * radius) / (PI * radius + length * 2);
    const rectangleMassContribution = 1 - circleMassContribution;

    // Represents two semi-circles
    const circleTensor = circleInertiaTensor(radius);

    // The semi-circles rotation point needs to be moved from the centre of the rectangle
    // eq. to half-length^2 for each semi-circle
    const parallelAxisDisplacement = length * length * RECIP_2;
    // Displacement on x-axis only affects rotation around y-axis
    circleTensor.d += parallelAxisDisplacement;

    circleTensor.mulScalar(circleMassContribution);
    const rectangleTensor = rectangleInertiaTensor(length, radius * 2).mulScalar(rectangleMassContribution);

    return circleTensor.add(rectangleTensor);
}

function rectangleMMOI(width: number, height: number): number {
    return (width * width + height * height) * RECIP_12;
}

function circleMMOI(radius: number): number {
    return radius * radius * RECIP_2;
}

function capsuleMMOI(radius: number, length: number): number {
    // Mass of entire shape is one, so we need the proportion of masses
    const circleMassContribution = (PI * radius) / (PI * radius + length * 2);
    const rectangleMassContribution = 1 - circleMassContribution;

    // The semi-circles rotation point needs to be moved from the centre of the rectangle
    // `length^2 / 2` is equal to `d^2` for each semi-circle
    const parallelAxisDisplacement = length * length * RECIP_2;

    // Represents two semi-circles
    const circleI = (circleMMOI(radius) + parallelAxisDisplacement) * circleMassContribution;
    const rectangleI = rectangleMMOI(length, radius * 2) * rectangleMassContribution;

    return circleI + rectangleI;
}

function triangleMMOI(p1: Vec2, p2: Vec2, p3: Vec2): number {
    const e1 = Vec2.sub(p2, p1);
    const e2 = Vec2.sub(p3, p1);
    const e3 = Vec2.sub(p2, p3);

    return (e1.magnitudeSquared() + e2.magnitudeSquared() + e3.magnitudeSquared()) * RECIP_36;
}

function convexPolygonMMOI(poly: Polygon): number {
    const area = poly.getArea();
    const centroid = poly.getCentre();
    const triangles = poly.fanTriangulate();

    let polygonI = 0;
    for (const triangle of triangles) {
        const massContribution = triangle.getArea() / area;
        const triangleI = triangleMMOI(triangle.points[0], triangle.points[1], triangle.points[2]);
        const parallelAxisDisplacement = triangle.getCentre().sub(centroid).magnitudeSquared();

        polygonI += (triangleI + parallelAxisDisplacement) * massContribution;
    }

    return polygonI;
}
