import { RigidBody } from "./body";


/**
 * An island is a set of bodies that are touching each other
 */
export class Island {
    bodies: Set<RigidBody>;

    constructor() {
        this.bodies = new Set();
    }

    add(body: RigidBody) {
        this.bodies.add(body);
    }

    merge(other: Island) {
        for (const body of other.bodies) {
            // Don't need to check if body is fixed, otherwise it wouldn't be in an island
            this.bodies.add(body);
            body.island = this;
        }

        other.clear();
    }

    clear() {
        this.bodies.clear();
    }
}
