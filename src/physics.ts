import { AABB } from "../lib/maths/aabb";
import { Capsule } from "../lib/maths/capsule";
import { Circle } from "../lib/maths/circle";
import { Mat2 } from "../lib/maths/mat2";
import { Polygon } from "../lib/maths/poly";
import { Shape } from "../lib/maths/shape";
import { PI, RECIP_12, RECIP_2, RECIP_36, RECIP_4 } from "../lib/maths/util";
import Vec2 from "../lib/maths/vec2";
import { EPA, GJK, contactPoints } from "./collision/gjk";

export class RigidBody {
    static nextID = 0;
    readonly id: number;

    // state
    asleep: boolean;
    fixed: boolean;

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
        this.asleep = false;
        this.fixed = fixed;
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
        this.linearVelocity.add(Vec2.mulScalar(impulse, this.inverseMass));

        const torque = radius.cross(impulse);
        this.angularVelocity += torque * this.inverseAngularMass;
    }

    update(deltaSeconds: number) {
        if (this.fixed) return;

        this.linearVelocity.add(Vec2.mulScalar(this.linearAcceleration, deltaSeconds));
        const translationDelta = Vec2.mulScalar(this.linearVelocity, deltaSeconds);
        this.translate(translationDelta);

        this.angularVelocity += this.angularAcceleration * deltaSeconds;
        const rotationDelta = this.angularVelocity * deltaSeconds;
        // Only process if more than 0.1 degree per second
        if (Math.abs(this.angularVelocity) > 0.001745) {
            this.rotate(rotationDelta);
        }

        this.linearAcceleration.x = 0;
        this.linearAcceleration.y = 0;
        this.angularAcceleration = 0;
    }
}

export function getCollisionManifold(bodyA: RigidBody, bodyB: RigidBody): CollisionManifold | undefined {
    const colliderA = bodyA.collider;
    const colliderB = bodyB.collider;

    const gjkResult = GJK(colliderA, colliderB);
    if (!gjkResult.collision) return undefined;

    const epaResult = EPA(colliderA, colliderB, gjkResult.simplex!);

    const contactPoint = contactPoints(epaResult.closestEdgeA, epaResult.closestEdgeB, epaResult.normal);
    const manifold = new CollisionManifold(bodyA, bodyB, epaResult.normal, epaResult.depth, contactPoint);

    // manifold.averageContacts();

    return manifold;
}

export class Collision {
    readonly bodyA: RigidBody;
    readonly bodyB: RigidBody;
    readonly restitution: number;
    readonly staticFriction: number;
    readonly dynamicFriction: number;
    readonly manifold: CollisionManifold;

    responseImpulsesMagnitudes: number[];

    constructor(bodyA: RigidBody, bodyB: RigidBody, manifold: CollisionManifold) {
        this.bodyA = bodyA;
        this.bodyB = bodyB;
        this.restitution = bodyA.restitution * bodyB.restitution;
        this.staticFriction = (bodyA.friction + bodyB.friction) * 0.5;
        this.dynamicFriction = bodyA.friction * bodyB.friction;
        this.manifold = manifold;

        this.responseImpulsesMagnitudes = new Array(manifold.contacts.length).fill(0);
    }

    resolve() {
        this.solvePositionLinearOnly();
        this.solveVelocity();
        this.solveFriction();
    }

    solvePositionLinearOnly() {
        const effectiveMass = 1 / (this.bodyA.inverseMass + this.bodyB.inverseMass);

        this.bodyA.translate(Vec2.mulScalar(this.manifold.mtv, -effectiveMass * this.bodyA.inverseMass));
        this.bodyB.translate(Vec2.mulScalar(this.manifold.mtv, effectiveMass * this.bodyB.inverseMass));
    }

    solvePosition() {
        for (const contact of this.manifold.contacts) {
            const effectiveMassNormal =
                this.bodyA.inverseMass +
                this.bodyB.inverseMass +
                this.bodyA.inverseAngularMass * Vec2.cross(contact.localPosA, this.manifold.normal) ** 2 +
                this.bodyB.inverseAngularMass * Vec2.cross(contact.localPosB, this.manifold.normal) ** 2;

            const correctionMagnitude = this.manifold.depth / effectiveMassNormal;
            const correction = Vec2.mulScalar(this.manifold.normal, correctionMagnitude);

            this.bodyA.translate(Vec2.mulScalar(correction, -this.bodyA.inverseMass));
            this.bodyB.translate(Vec2.mulScalar(correction, this.bodyB.inverseMass));

            this.bodyA.rotate(Vec2.cross(contact.localPosA, correction) * -this.bodyA.inverseAngularMass);
            this.bodyB.rotate(Vec2.cross(contact.localPosB, correction) * this.bodyB.inverseAngularMass);
        }
    }

    solveVelocityLinearOnly() {
        const relativeVelocity = Vec2.sub(this.bodyB.linearVelocity, this.bodyA.linearVelocity);
        const relativeNormalVelocity = Vec2.dot(relativeVelocity, this.manifold.normal);
        if (relativeNormalVelocity > 0.001) return; // already separating

        const effectiveMass = this.bodyA.inverseMass + this.bodyB.inverseMass;

        const restitution = this.bodyA.restitution * this.bodyB.restitution;

        const impulseMagnitude = (-(1 + restitution) * relativeNormalVelocity) / effectiveMass;
        const impulse = Vec2.mulScalar(this.manifold.normal, impulseMagnitude);

        console.log(this.manifold.normal);

        this.bodyA.applyImpulse(Vec2.neg(impulse));
        this.bodyB.applyImpulse(impulse);
    }

    solveVelocity() {
        for (const [i, contact] of this.manifold.contacts.entries()) {
            const relativeVelocity = Vec2.sub(
                this.bodyB.getVelocityAtPoint(contact.localPosB),
                this.bodyA.getVelocityAtPoint(contact.localPosA)
            );
            const relativeVelocityDotNormal = Vec2.dot(relativeVelocity, this.manifold.normal);
            if (relativeVelocityDotNormal > 0.001) continue; // already separating

            const effectiveMassNormal =
                this.bodyA.inverseMass +
                this.bodyB.inverseMass +
                this.bodyA.inverseAngularMass * Vec2.cross(contact.localPosA, this.manifold.normal) ** 2 +
                this.bodyB.inverseAngularMass * Vec2.cross(contact.localPosB, this.manifold.normal) ** 2;

            const impulseMagnitude = (-(1 + this.restitution) * relativeVelocityDotNormal) / effectiveMassNormal;
            const impulse = Vec2.mulScalar(this.manifold.normal, impulseMagnitude);

            this.bodyA.applyImpulse(Vec2.neg(impulse), contact.localPosA);
            this.bodyB.applyImpulse(impulse, contact.localPosB);

            // cache impulse magnitude for friction
            this.responseImpulsesMagnitudes[i] = impulseMagnitude;
        }
    }

    solveFriction() {
        for (const [i, contact] of this.manifold.contacts.entries()) {
            const relativeVelocity = Vec2.sub(
                this.bodyB.getVelocityAtPoint(contact.localPosB),
                this.bodyA.getVelocityAtPoint(contact.localPosA)
            );
            const relativeVelocityDotNormal = Vec2.dot(relativeVelocity, this.manifold.normal);

            // Relative Velocity vector to the Normal vector (with magnitude according to projected relative velocity)
            const tangent = Vec2.sub(
                relativeVelocity,
                Vec2.mulScalar(this.manifold.normal, relativeVelocityDotNormal)
            ).normalise();
            const relativeVelocityDotTangent = Vec2.dot(relativeVelocity, tangent);
            if (Math.abs(relativeVelocityDotTangent) < 0.001) continue; // stationary

            const effectiveMassTangent =
                this.bodyA.inverseMass +
                this.bodyB.inverseMass +
                this.bodyA.inverseAngularMass * Vec2.cross(contact.localPosA, tangent) ** 2 +
                this.bodyB.inverseAngularMass * Vec2.cross(contact.localPosB, tangent) ** 2;

            const staticFrictionLimit = this.responseImpulsesMagnitudes[i] * this.staticFriction;
            const dynamicFrictionLimit = this.responseImpulsesMagnitudes[i] * this.dynamicFriction;

            let impulseMagnitude = -relativeVelocityDotTangent / effectiveMassTangent;

            let impulse;
            // If the impulse magnitude is less than the static friction threshold, use static friction
            if (Math.abs(impulseMagnitude) <= staticFrictionLimit) {
                impulse = Vec2.mulScalar(tangent, impulseMagnitude);
            } else {
                // Otherwise use dynamic friction
                impulse = Vec2.mulScalar(tangent, impulseMagnitude * this.dynamicFriction);
            }

            this.bodyA.applyImpulse(Vec2.neg(impulse), contact.localPosA);
            this.bodyB.applyImpulse(impulse, contact.localPosB);
        }
    }
}

export class CollisionManifold {
    readonly normal: Vec2;
    readonly depth: number;
    readonly mtv: Vec2;
    readonly contacts: Contact[];

    constructor(bodyA: RigidBody, bodyB: RigidBody, normal: Vec2, depth: number, contacts: Vec2[]) {
        this.normal = normal;
        this.depth = depth;
        this.mtv = Vec2.mulScalar(normal, depth);
        this.contacts = contacts.map((contact) => new Contact(bodyA, bodyB, contact, this.mtv));
    }

    averageContacts() {
        // Testing of averaging contacts
        if (this.contacts.length > 1) {
            this.contacts[0] = {
                worldPos: this.contacts[0].worldPos.add(this.contacts[1].worldPos).mulScalar(0.5),
                localPosA: this.contacts[0].localPosA.add(this.contacts[1].localPosA).mulScalar(0.5),
                localPosB: this.contacts[0].localPosB.add(this.contacts[1].localPosB).mulScalar(0.5),
            };
            this.contacts.length = 1;
        }
    }
}

export class Contact {
    readonly worldPos: Vec2;
    readonly localPosA: Vec2;
    readonly localPosB: Vec2;

    constructor(bodyA: RigidBody, bodyB: RigidBody, worldPos: Vec2, mtv: Vec2) {
        this.worldPos = worldPos;
        this.localPosA = Vec2.sub(worldPos, bodyA.position);
        this.localPosB = Vec2.sub(Vec2.sub(worldPos, mtv), bodyB.position);
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

    // The semi-circles rotation point needs to be moved to the centre of the rectangle
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

    // The semi-circles rotation point needs to be moved to the centre of the rectangle
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
        const massContribtion = triangle.getArea() / area;
        const triangleI = triangleMMOI(triangle.points[0], triangle.points[1], triangle.points[2]);
        const parallelAxisDisplacement = triangle.getCentre().sub(centroid).magnitudeSquared();

        polygonI += (triangleI + parallelAxisDisplacement) * massContribtion;
    }

    return polygonI;
}
