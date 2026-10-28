import Vec2 from "../lib/maths/vec2";
import { RigidBody } from "./body";

class Spring {
    radius: Vec2;
    body: RigidBody;
    k: number;
    d: number;

    constructor(radius: Vec2, body: RigidBody, k: number, d: number) {
        this.radius = radius;
        this.body = body;
        this.k = k;
        this.d = d;
    }

    apply(anchor: Vec2, anchorVelocity: Vec2, deltaSeconds: number) {
        this.body.sleeping = false;

        // off-centre grab
        let radius = Vec2.rotate(this.radius, this.body.rotation);
        let attach = Vec2.add(this.body.position, radius);
        let vel = this.body.getVelocityAtPoint(radius);

        // centre grab
        // let radius = Vec2.zero();
        // let attach = this.body.position;
        // let vel = this.body.linearVelocity;

        const springForce = Vec2.sub(anchor, attach).mulScalar(this.k);
        const dampingForce = Vec2.sub(anchorVelocity, vel).mulScalar(this.d);

        this.body.applyImpulse(springForce.add(dampingForce).mulScalar(this.body.mass * deltaSeconds), radius);
    }
}


class SpringMutual {
    radiusA: Vec2;
    radiusB: Vec2;
    bodyA: RigidBody;
    bodyB: RigidBody;
    k: number;
    d: number;

    constructor(radiusA: Vec2, radiusB: Vec2, bodyA: RigidBody, bodyB: RigidBody, k: number, d: number) {
        this.radiusA = radiusA;
        this.radiusB = radiusB;
        this.bodyA = bodyA;
        this.bodyB = bodyB;
        this.k = k;
        this.d = d;
    }

    apply(anchor: Vec2, anchorVelocity: Vec2, deltaSeconds: number) {
        this.bodyA.sleeping = false;
        this.bodyB.sleeping = false;

        // off-centre grab
        let radiusA = Vec2.rotate(this.radiusA, this.bodyA.rotation);
        let attachA = Vec2.add(this.bodyA.position, radiusA);
        let velA = this.bodyA.getVelocityAtPoint(radiusA);

        let radiusB = Vec2.rotate(this.radiusB, this.bodyB.rotation);
        let attachB = Vec2.add(this.bodyB.position, radiusB);
        let velB = this.bodyB.getVelocityAtPoint(radiusB);

        const springForceA = Vec2.sub(anchor, attachA).mulScalar(this.k);
        const dampingForceA = Vec2.sub(anchorVelocity, velA).mulScalar(this.d);

        const springForceB = Vec2.sub(anchor, attachB).mulScalar(this.k);
        const dampingForceB = Vec2.sub(anchorVelocity, velB).mulScalar(this.d);

        this.bodyA.applyImpulse(springForceA.add(dampingForceA).mulScalar(this.bodyA.mass * deltaSeconds), radiusA);
        this.bodyB.applyImpulse(springForceB.add(dampingForceB).mulScalar(this.bodyB.mass * deltaSeconds), radiusB);
    }
}