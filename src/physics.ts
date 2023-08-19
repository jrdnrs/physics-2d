import Vec2 from "../lib/maths/vec2";
import { QuadTree } from "../lib/collections/quadtree";
import { AABB } from "../lib/maths/aabb";
import { RigidBody } from "./body";
import { Collision, CollisionManifold, Contact } from "./collision/collision";
import { EPA, GJK, getContactPoints } from "./collision/gjk";
import { swapRemove } from "../lib/util";

export class PhysicsEngine {
    static readonly gravity = 981;

    static readonly stepsPerSecond = 500;
    static readonly fixedTimeStep = 1 / PhysicsEngine.stepsPerSecond;

    static readonly maxVelocity = 333;
    static readonly maxAngularVelocity = 33;

    timeElapsed: number;
    stepsElapsed: number;
    updateDuration: number;

    worldBounds: AABB;

    quadtree: QuadTree<RigidBody>;

    bodies: RigidBody[] = [];
    collisions: Map<number, Collision>;

    constructor(worldBounds: AABB) {
        this.timeElapsed = 0;
        this.stepsElapsed = 0;
        this.updateDuration = 0;

        this.worldBounds = worldBounds;
        this.quadtree = new QuadTree(worldBounds);
        this.bodies = [];
        this.collisions = new Map();
    }

    addBody(body: RigidBody) {
        this.bodies.push(body);
        this.quadtree.insert(body);
    }

    removeBody(body: RigidBody) {
        const index = this.bodies.indexOf(body);
        [this.bodies[index], this.bodies[this.bodies.length - 1]] = [
            this.bodies[this.bodies.length - 1],
            this.bodies[index],
        ];
        this.bodies.pop();
        this.quadtree.remove(body);
    }

    update(deltaSeconds: number): number {
        const deltaSteps = Math.floor(this.timeElapsed / PhysicsEngine.fixedTimeStep) - this.stepsElapsed;

        const start = performance.now();
        for (let i = 0; i < deltaSteps; i++) {
            this.step(PhysicsEngine.fixedTimeStep);
        }
        // this.step(PhysicsEngine.fixedTimeStep);
        this.updateDuration = performance.now() - start;

        this.timeElapsed += deltaSeconds;
        this.stepsElapsed += deltaSteps;

        return deltaSteps;
    }

    step(deltaSeconds: number) {
        for (const body of this.bodies) {
            if (body.fixed || body.asleep) continue;

            // body.applyForce(new Vec2(0, PhysicsEngine.gravity * body.mass), Vec2.zero());
            body.linearVelocity.y += PhysicsEngine.gravity * deltaSeconds;
            body.update(deltaSeconds);

            // crude wrap around
            const wrap = new Vec2(0, 0);
            if (body.bounds.min.x > this.worldBounds.max.x) wrap.x = -body.bounds.max.x;
            if (body.bounds.min.y > this.worldBounds.max.y) wrap.y = -body.bounds.max.y;
            if (body.bounds.max.x < 0) wrap.x = this.worldBounds.max.x - body.bounds.min.x;
            if (body.bounds.max.y < 0) wrap.y = this.worldBounds.max.y - body.bounds.min.y;
            body.translate(wrap);

            if (body.linearVelocity.magnitudeSquared() > 0 || Math.abs(body.angularVelocity) > 0) {
                this.quadtree.update(body);
            }
        }

        this.checkCollisions();
        this.resolveCollisions();
    }

    resolveCollisions() {
        for (const collision of this.collisions.values()) {
            collision.solvePositionLinearOnly();
            collision.applyAccumulatedImpulses();

            for (const contact of collision.manifold.contacts) {
                contact.updateRestitutionBias(collision.bodyA, collision.bodyB, collision.manifold.normal);
            }
        }

        for (let i = 0; i < 5; i++) {
            for (const collision of this.collisions.values()) {
                collision.solveVelocity();
            }
        }
    }

    checkCollisions() {
        const resolvedCollisionIds = Array.from(this.collisions.keys());

        for (const bodyA of this.bodies) {
            for (const bodyB of this.quadtree.get(bodyA.bounds)) {
                if (((bodyA.fixed || bodyA.asleep) && (bodyB.fixed || bodyB.asleep)) || bodyB.id <= bodyA.id) continue;
                // if (bodyA.asleep) bodyA.wakeUp();
                // if (bodyB.asleep) bodyB.wakeUp();

                const collision = this.checkCollision(bodyA, bodyB);
                if (collision === undefined) continue;

                const existingCollision = this.collisions.get(collision.id);
                if (existingCollision !== undefined) {
                    swapRemove(resolvedCollisionIds, resolvedCollisionIds.indexOf(collision.id));

                    const newContact = collision.manifold.contacts[0];
                    collision.manifold.contacts = existingCollision.manifold.contacts;

                    // remove invalid points
                    for (let i = collision.manifold.contacts.length - 1; i >= 0; i--) {
                        const contact = collision.manifold.contacts[i];

                        const currentWorldPosA = Vec2.add(bodyA.position, contact.localPosA);
                        const currentWorldPosB = Vec2.add(bodyB.position, contact.localPosB);

                        const currentDisplacement = Vec2.sub(currentWorldPosB, currentWorldPosA);
                        const currentDistanceA = Vec2.sub(contact.worldPosA, currentWorldPosA).magnitudeSquared();
                        const currentDistanceB = Vec2.sub(contact.worldPosB, currentWorldPosB).magnitudeSquared();

                        const inContact = collision.manifold.normal.dot(currentDisplacement) <= 0;

                        if (!inContact || currentDistanceA > 4 || currentDistanceB > 4) {
                            swapRemove(collision.manifold.contacts, i);
                        }
                    }

                    let duplicate = false;
                    // add new point
                    for (const contact of collision.manifold.contacts) {
                        const distanceA = Vec2.sub(newContact.localPosA, contact.localPosA).magnitudeSquared();
                        const distanceB = Vec2.sub(newContact.localPosB, contact.localPosB).magnitudeSquared();

                        if (distanceA < 4 || distanceB < 4) {
                            duplicate = true;
                            break;
                        }
                    }

                    if (!duplicate) {
                        collision.manifold.contacts.push(newContact);
                    }

                    // reduce number of points to 2
                    if (collision.manifold.contacts.length > 2) {
                        let deepestContact = collision.manifold.contacts[0];
                        let deepestDepth = 0;

                        for (const contact of collision.manifold.contacts) {
                            const depth = Vec2.sub(contact.worldPosB, contact.worldPosA).magnitudeSquared();
                            if (depth > deepestDepth) {
                                deepestContact = contact;
                                deepestDepth = depth;
                            }
                        }

                        let furthestContact = collision.manifold.contacts[0];
                        let furthestDistance = 0;

                        for (const contact of collision.manifold.contacts) {
                            const distance = Vec2.sub(contact.worldPosA, deepestContact.worldPosA).magnitudeSquared();
                            if (distance > furthestDistance) {
                                furthestContact = contact;
                                furthestDistance = distance;
                            }
                        }

                        collision.manifold.contacts[0] = deepestContact;
                        collision.manifold.contacts[1] = furthestContact;
                        collision.manifold.contacts.length = 2;
                    }
                }

                this.collisions.set(collision.id, collision);
            }
        }

        // remove resolved collisions
        for (const id of resolvedCollisionIds) {
            this.collisions.delete(id);
        }
    }

    checkCollision(bodyA: RigidBody, bodyB: RigidBody): Collision | undefined {
        const colliderA = bodyA.collider;
        const colliderB = bodyB.collider;

        const gjkResult = GJK(colliderA, colliderB);
        if (!gjkResult.collision) return undefined;

        const epaResult = EPA(colliderA, colliderB, gjkResult.simplex!);
        const newContactPoint = new Contact(
            bodyA,
            bodyB,
            epaResult.worldContactA,
            epaResult.worldContactB,
            epaResult.normal,
            epaResult.tangent
        );

        const manifold = new CollisionManifold(epaResult.normal, epaResult.mtv, epaResult.depth, [newContactPoint]);

        const collision = new Collision(bodyA, bodyB, manifold);

        return collision;
    }
}
