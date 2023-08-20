import { RigidBody } from "./body";

export class Island {
    bodies: Set<RigidBody>;

    constructor() {
        this.bodies = new Set();
    }

    add(body: RigidBody) {
        this.bodies.add(body);
    }

    merge(island: Island) {
        for (const body of island.bodies) {
            // Don't need to check if body is fixed, otherwise it wouldn't be in an island
            this.bodies.add(body);
            body.island = this;
        }

        island.clear();
    }

    clear() {
        this.bodies.clear();
    }
}
