import { QuadTree } from "../lib/collections/quadtree";
import { AABB } from "../lib/maths/aabb";
import { RigidBody } from "./body";
import { Collision, CollisionManifold, Contact } from "./collision/collision";
import { EPA, GJK } from "./collision/gjk";
import { assert, swapRemove } from "../lib/util";
import { Island } from "./island";

export type PhysicsEngineConfig = {
    gravity: number;
    stepsPerSecond: number;
    velocityIterations: number;
    sleepLinearThreshold: number;
    sleepAngularThreshold: number;
    sleepTimeThreshold: number;
};

export const PhysicsEngineConfigDefault: PhysicsEngineConfig = {
    gravity: 981,
    stepsPerSecond: 500,
    velocityIterations: 5,
    sleepLinearThreshold: 0.15,
    sleepAngularThreshold: 0.15,
    sleepTimeThreshold: 0.5,
};

export class PhysicsEngine {
    readonly config: PhysicsEngineConfig;
    readonly fixedTimeStep: number;

    timeElapsed: number;
    stepsElapsed: number;

    worldBounds: AABB;
    quadtree: QuadTree<RigidBody>;
    bodies: RigidBody[];
    collisions: Map<number, Collision>;
    islands: Island[];

    constructor(worldBounds: AABB, config: PhysicsEngineConfig = PhysicsEngineConfigDefault) {
        assert(config.stepsPerSecond > 0, "Steps per second must be greater than 0");

        this.config = config;
        this.fixedTimeStep = 1 / config.stepsPerSecond;

        this.timeElapsed = 0;
        this.stepsElapsed = 0;

        this.worldBounds = worldBounds;
        this.quadtree = new QuadTree(worldBounds);
        this.bodies = [];
        this.collisions = new Map();
        this.islands = [];
    }

    /** Adds body to the simulation, unless it is out of world bounds */
    addBody(body: RigidBody) {
        if (this.quadtree.insert(body)) {
            this.bodies.push(body);
        }
    }

    /**
     * Removes the given body from the simulation
     *
     * This uses a swap remove on the internal list of bodies, so don't rely on the order of bodies!
     */
    removeBody(body: RigidBody) {
        const index = this.bodies.indexOf(body);
        if (index === -1) return;

        swapRemove(this.bodies, index);
        this.quadtree.remove(body);
    }

    /**
     * Updates the simulation by the given time step in seconds, and returns the number of actual
     * fixed steps that were taken
     */
    update(deltaSeconds: number): number {
        const deltaSteps = Math.floor(this.timeElapsed / this.fixedTimeStep) - this.stepsElapsed;

        for (let i = 0; i < deltaSteps; i++) {
            this.step(this.fixedTimeStep);
        }

        this.timeElapsed += deltaSeconds;
        this.stepsElapsed += deltaSteps;

        return deltaSteps;
    }

    private updateMotion(deltaSeconds: number) {
        for (let i = this.bodies.length - 1; i >= 0; i--) {
            const body = this.bodies[i];

            if (body.fixed || body.sleeping) continue;

            body.linearVelocity.y += this.config.gravity * deltaSeconds;
            body.integrate(deltaSeconds);

            if (body.linearVelocity.magnitudeSquared() > 0 || Math.abs(body.angularVelocity) > 0) {
                const inBounds = this.quadtree.update(body);
                if (!inBounds) swapRemove(this.bodies, i);
            }
        }
    }

    private updateIslands(deltaSeconds: number) {
        for (const island of this.islands) {
            let minSleepTime = Number.MAX_VALUE;

            for (const body of island.bodies) {
                const linearMotion = body.linearVelocity.magnitudeSquared();
                const angularMotion = Math.abs(body.angularVelocity);

                if (
                    linearMotion < this.config.sleepLinearThreshold * this.config.sleepLinearThreshold &&
                    angularMotion < this.config.sleepAngularThreshold
                ) {
                    body.timeStill += deltaSeconds;
                    minSleepTime = Math.min(minSleepTime, body.timeStill);
                } else {
                    body.timeStill = 0;
                    minSleepTime = 0;
                }
            }

            // Put the whole island to sleep if all bodies are still for a while
            if (minSleepTime >= this.config.sleepTimeThreshold) {
                for (const body of island.bodies) {
                    body.sleeping = true;
                }
            }
        }
    }

    private clearIslands() {
        for (const body of this.bodies) {
            body.island = undefined;
        }
        this.islands.length = 0;
    }

    private assignIsland(bodyA: RigidBody, bodyB: RigidBody) {
        if (bodyA.island !== undefined && bodyB.island !== undefined) {
            if (bodyA.island !== bodyB.island) {
                bodyA.island.merge(bodyB.island);
            }
        } else if (bodyA.island !== undefined) {
            bodyB.addToIsland(bodyA.island);
        } else if (bodyB.island !== undefined) {
            bodyA.addToIsland(bodyB.island);
        } else {
            const newIsland = new Island();
            bodyA.addToIsland(newIsland);
            bodyB.addToIsland(newIsland);
            this.islands.push(newIsland);
        }
    }

    private step(deltaSeconds: number) {
        this.updateMotion(deltaSeconds);

        this.clearIslands();
        this.checkCollisions();
        this.resolveCollisions();

        this.updateIslands(deltaSeconds);
    }

    private resolveCollisions() {
        for (const collision of this.collisions.values()) {
            collision.solvePositionLinearOnly();
            collision.applyAccumulatedImpulses();

            for (const contact of collision.manifold.contacts) {
                contact.updateRestitutionBias(collision.bodyA, collision.bodyB, collision.manifold.normal);
            }
        }

        for (let i = 0; i < this.config.velocityIterations; i++) {
            for (const collision of this.collisions.values()) {
                collision.solveVelocity();
            }
        }
    }

    private checkCollisions() {
        const existingCollisionIds = Array.from(this.collisions.keys());

        for (const bodyA of this.bodies) {
            for (const bodyB of this.quadtree.get(bodyA.bounds)) {
                // If both bodies are fixed or sleeping, no need to check for collisions
                if (((bodyA.fixed || bodyA.sleeping) && (bodyB.fixed || bodyB.sleeping)) || bodyB.id <= bodyA.id)
                    continue;

                // If collision is no longer valid, no need to resolve
                const collision = this.checkCollision(bodyA, bodyB);
                if (collision === undefined) continue;

                // If at least one body is awake, wake up both bodies
                bodyA.sleeping = false;
                bodyB.sleeping = false;

                this.assignIsland(bodyA, bodyB);

                const existingCollision = this.collisions.get(collision.id);
                if (existingCollision !== undefined) {
                    swapRemove(existingCollisionIds, existingCollisionIds.indexOf(collision.id));

                    const newContact = collision.manifold.contacts[0];
                    collision.manifold.contacts = existingCollision.manifold.contacts;

                    collision.updateContactPoints(newContact);
                }

                this.collisions.set(collision.id, collision);
            }
        }

        // Remove ids in this list indicate that the collision no longer persists
        for (const id of existingCollisionIds) {
            this.collisions.delete(id);
        }
    }

    private checkCollision(bodyA: RigidBody, bodyB: RigidBody): Collision | undefined {
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
