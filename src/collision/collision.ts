import { clamp } from "../../lib/maths/util";
import Vec2 from "../../lib/maths/vec2";
import { swapRemove } from "../../lib/util";
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

    updateContactPoints(newContact: Contact) {
        // Value is arbitrary, but basically defines how far apart contact points can be before
        // they are considered different
        const distanceThreshold = 4;

        // 1. First we should remove any contact points that are found to be invalid
        for (let i = this.manifold.contacts.length - 1; i >= 0; i--) {
            const contact = this.manifold.contacts[i];

            const currentWorldPosA = Vec2.add(this.bodyA.position, contact.localPosA);
            const currentWorldPosB = Vec2.add(this.bodyB.position, contact.localPosB);

            // If the contact points are still overlapping, along the collision normal, bodies
            // must still be in contact
            const currentOverlap = Vec2.sub(currentWorldPosB, currentWorldPosA);
            const inContact = this.manifold.normal.dot(currentOverlap) <= 0.01;

            // If the contact points' position has changed significantly from the initial position,
            // we should remove them
            const movementA = Vec2.sub(contact.worldPosA, currentWorldPosA).magnitudeSquared();
            const movementB = Vec2.sub(contact.worldPosB, currentWorldPosB).magnitudeSquared();

            if (!inContact || movementA > distanceThreshold || movementB > distanceThreshold) {
                swapRemove(this.manifold.contacts, i);
            }
        }

        // 2. Determine whether the new contact point is a duplicate of an existing one
        let duplicate = false;
        for (const contact of this.manifold.contacts) {
            const distanceA = Vec2.sub(newContact.localPosA, contact.localPosA).magnitudeSquared();
            const distanceB = Vec2.sub(newContact.localPosB, contact.localPosB).magnitudeSquared();

            if (distanceA < distanceThreshold || distanceB < distanceThreshold) {
                duplicate = true;
                break;
            }
        }
        if (!duplicate) this.manifold.contacts.push(newContact);

        // 3. Reduce number of contact points to 2 by selecting the deepest and furthest apart points
        if (this.manifold.contacts.length > 2) {
            let deepestContact = this.manifold.contacts[0];
            let deepestDepth = 0;

            for (const contact of this.manifold.contacts) {
                const depth = Vec2.sub(contact.worldPosB, contact.worldPosA).magnitudeSquared();
                if (depth > deepestDepth) {
                    deepestContact = contact;
                    deepestDepth = depth;
                }
            }

            let furthestContact = this.manifold.contacts[0];
            let furthestDistance = 0;

            for (const contact of this.manifold.contacts) {
                const distance = Vec2.sub(contact.worldPosA, deepestContact.worldPosA).magnitudeSquared();
                if (distance > furthestDistance) {
                    furthestContact = contact;
                    furthestDistance = distance;
                }
            }

            this.manifold.contacts[0] = deepestContact;
            this.manifold.contacts[1] = furthestContact;
            this.manifold.contacts.length = 2;
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
