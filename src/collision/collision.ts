import { clamp } from "../../lib/maths/util";
import Vec2 from "../../lib/maths/vec2";
import { RigidBody } from "../body";

export class Collision {
    readonly id: number;
    readonly bodyA: RigidBody;
    readonly bodyB: RigidBody;
    readonly restitution: number;
    readonly friction: number;

    manifold: CollisionManifold;

    constructor(bodyA: RigidBody, bodyB: RigidBody, manifold: CollisionManifold) {
        this.id = bodyA.id * 1e10 + bodyB.id;
        this.bodyA = bodyA;
        this.bodyB = bodyB;
        this.restitution = bodyA.restitution * bodyB.restitution;
        this.friction = (bodyA.friction + bodyB.friction) * 0.5;
        this.manifold = manifold;
    }

    applyAccumulatedImpulses() {
        for (const contact of this.manifold.contacts) {
            const impulse = Vec2.add(
                Vec2.mulScalar(this.manifold.normal, contact.accumulatedNormalMagnitude),
                Vec2.mulScalar(this.manifold.tangent, contact.accumulatedTangentMagnitude)
            );

            this.bodyA.applyImpulse(Vec2.neg(impulse), contact.localPosA);
            this.bodyB.applyImpulse(impulse, contact.localPosB);
        }
    }

    solvePositionLinearOnly() {
        const effectiveMass = 1 / (this.bodyA.inverseMass + this.bodyB.inverseMass);

        const correction = Vec2.mulScalar(this.manifold.normal, Math.max(0, this.manifold.depth - 0.1));

        this.bodyA.translate(Vec2.mulScalar(correction, -effectiveMass * this.bodyA.inverseMass));
        this.bodyB.translate(Vec2.mulScalar(correction, effectiveMass * this.bodyB.inverseMass));
    }

    solveVelocityLinearOnly() {
        const relativeVelocity = Vec2.sub(this.bodyB.linearVelocity, this.bodyA.linearVelocity);
        const relativeVelocityDotNormal = Vec2.dot(relativeVelocity, this.manifold.normal);
        if (relativeVelocityDotNormal > 0.001) return; // already separating

        const effectiveMass = this.bodyA.inverseMass + this.bodyB.inverseMass;

        const restitution = this.bodyA.restitution * this.bodyB.restitution;

        const normalImpulseMagnitude = (-(1 + restitution) * relativeVelocityDotNormal) / effectiveMass;
        const normalImpulse = Vec2.mulScalar(this.manifold.normal, normalImpulseMagnitude);

        const tangent = Vec2.sub(
            relativeVelocity,
            Vec2.mulScalar(this.manifold.normal, relativeVelocityDotNormal)
        ).normalise();
        const relativeVelocityDotTangent = Vec2.dot(relativeVelocity, tangent);
        // if (Math.abs(relativeVelocityDotTangent) < 0.001) return;

        const tangentImpulseMagnitude = -Math.min(
            relativeVelocityDotTangent / effectiveMass,
            normalImpulseMagnitude * this.friction
        );
        const tangentImpulse = Vec2.mulScalar(tangent, tangentImpulseMagnitude);

        this.bodyA.applyImpulse(Vec2.neg(normalImpulse));
        this.bodyB.applyImpulse(normalImpulse);

        console.log(
            this.bodyB.linearVelocity,
            relativeVelocity,
            Vec2.mulScalar(tangentImpulse, this.bodyB.inverseMass)
        );

        this.bodyA.applyImpulse(Vec2.neg(tangentImpulse));
        this.bodyB.applyImpulse(tangentImpulse);
    }

    solvePosition() {
        for (const contact of this.manifold.contacts) {
            const effectiveMassNormal =
                this.bodyA.inverseMass +
                this.bodyB.inverseMass +
                this.bodyA.inverseAngularMass * Vec2.cross(contact.localPosA, this.manifold.normal) ** 2 +
                this.bodyB.inverseAngularMass * Vec2.cross(contact.localPosB, this.manifold.normal) ** 2;

            const correctionMagnitude = Math.max(0, this.manifold.depth - 0.1) / effectiveMassNormal;
            const correction = Vec2.mulScalar(this.manifold.normal, correctionMagnitude);

            this.bodyA.translate(Vec2.mulScalar(correction, -this.bodyA.inverseMass));
            this.bodyB.translate(Vec2.mulScalar(correction, this.bodyB.inverseMass));

            this.bodyA.rotate(Vec2.cross(contact.localPosA, correction) * -this.bodyA.inverseAngularMass);
            this.bodyB.rotate(Vec2.cross(contact.localPosB, correction) * this.bodyB.inverseAngularMass);
        }
    }

    solveVelocity() {
        // Normal Impulse
        for (const contact of this.manifold.contacts) {
            const relativeVelocity = this.bodyB
                .getVelocityAtPoint(contact.localPosB)
                .sub(this.bodyA.getVelocityAtPoint(contact.localPosA));

            const relativeVelocityDotNormal = Vec2.dot(relativeVelocity, this.manifold.normal);
            // if (relativeVelocityDotNormal > 0.001) continue; // already separating

            // const impulseMagnitude = -(1 + this.restitution) * relativeVelocityDotNormal * contact.effectiveMassNormal;
            const impulseMagnitude =
                -(relativeVelocityDotNormal - contact.originalRestitutionBias) * contact.effectiveMassNormal;

            const newAccumulatedMagnitude = Math.max(contact.accumulatedNormalMagnitude + impulseMagnitude, 0);
            const newImpulse = newAccumulatedMagnitude - contact.accumulatedNormalMagnitude;
            contact.accumulatedNormalMagnitude = newAccumulatedMagnitude;

            this.bodyA.applyImpulse(Vec2.mulScalar(this.manifold.normal, -newImpulse), contact.localPosA);
            this.bodyB.applyImpulse(Vec2.mulScalar(this.manifold.normal, newImpulse), contact.localPosB);
        }

        // Tangent Impulse
        for (const contact of this.manifold.contacts) {
            const relativeVelocity = this.bodyB
                .getVelocityAtPoint(contact.localPosB)
                .sub(this.bodyA.getVelocityAtPoint(contact.localPosA));

            const relativeVelocityDotTangent = Vec2.dot(relativeVelocity, this.manifold.tangent);
            // if (Math.abs(relativeVelocityDotTangent) < 0.001) continue;

            const impulseMagnitude = -relativeVelocityDotTangent * contact.effectiveMassTangent;

            const staticFrictionLimit = contact.accumulatedNormalMagnitude * this.friction;

            const newAccumulatedMagnitude = clamp(
                contact.accumulatedTangentMagnitude + impulseMagnitude,
                -staticFrictionLimit,
                staticFrictionLimit
            );
            const newImpulse = newAccumulatedMagnitude - contact.accumulatedTangentMagnitude;
            contact.accumulatedTangentMagnitude = newAccumulatedMagnitude;

            this.bodyA.applyImpulse(Vec2.mulScalar(this.manifold.tangent, -newImpulse), contact.localPosA);
            this.bodyB.applyImpulse(Vec2.mulScalar(this.manifold.tangent, newImpulse), contact.localPosB);
        }
    }
}

export class CollisionManifold {
    normal: Vec2;
    tangent: Vec2;
    depth: number;
    mtv: Vec2;
    contacts: Contact[];

    constructor(normal: Vec2, mtv: Vec2, depth: number, contacts: Contact[]) {
        this.normal = normal;
        this.tangent = Vec2.perpendicular(normal);
        this.depth = depth;
        this.mtv = mtv;
        this.contacts = contacts;
    }
}

export class Contact {
    readonly worldPosA: Vec2;
    readonly worldPosB: Vec2;
    readonly localPosA: Vec2;
    readonly localPosB: Vec2;
    readonly effectiveMassNormal: number;
    readonly effectiveMassTangent: number;

    originalRestitutionBias: number;
    accumulatedNormalMagnitude: number;
    accumulatedTangentMagnitude: number;

    constructor(bodyA: RigidBody, bodyB: RigidBody, worldPosA: Vec2, worldPosB: Vec2, normal: Vec2, tangent: Vec2) {
        this.worldPosA = worldPosA;
        this.worldPosB = worldPosB;
        this.localPosA = Vec2.sub(worldPosA, bodyA.position);
        this.localPosB = Vec2.sub(worldPosB, bodyB.position);
        this.accumulatedNormalMagnitude = 0;
        this.accumulatedTangentMagnitude = 0;
        this.originalRestitutionBias = 0;

        // For iterative solvers, repeated application of restitution loss can cause excessive loss of energy.
        // Thus, we need to store the original restitution loss and just use it to bias future calculations.
        this.updateRestitutionBias(bodyA, bodyB, normal);

        const radiusNormalA = Vec2.cross(this.localPosA, normal);
        const radiusNormalB = Vec2.cross(this.localPosB, normal);
        this.effectiveMassNormal =
            1 /
            (bodyA.inverseMass +
                bodyB.inverseMass +
                bodyA.inverseAngularMass * radiusNormalA * radiusNormalA +
                bodyB.inverseAngularMass * radiusNormalB * radiusNormalB);

        const radiusTangentA = Vec2.cross(this.localPosA, tangent);
        const radiusTangentB = Vec2.cross(this.localPosB, tangent);
        this.effectiveMassTangent =
            1 /
            (bodyA.inverseMass +
                bodyB.inverseMass +
                bodyA.inverseAngularMass * radiusTangentA * radiusTangentA +
                bodyB.inverseAngularMass * radiusTangentB * radiusTangentB);
    }

    updateRestitutionBias(bodyA: RigidBody, bodyB: RigidBody, normal: Vec2) {
        const relativeVelocity = bodyB.getVelocityAtPoint(this.localPosB).sub(bodyA.getVelocityAtPoint(this.localPosA));
        const relativeVelocityDotNormal = Vec2.dot(relativeVelocity, normal);
        if (relativeVelocityDotNormal < -0.1) {
            // Only apply bias if objects are moving towards each other
            this.originalRestitutionBias = -(bodyA.restitution * bodyB.restitution) * relativeVelocityDotNormal;
        } else {
            this.originalRestitutionBias = 0;
        }
    }
}
