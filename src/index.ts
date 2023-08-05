// @ts-ignore
if (process.env.NODE_ENV !== "production") {
    // @ts-ignore
    const hot = module.hot;
    if (hot) {
        hot.dispose(() => {
            window.location.reload();
            throw "hotReload";
        });
    }
}

import { QuadTree, QuadTreeNode } from "../lib/collections/quadtree";
import Drawer, { DrawerConfigDefault } from "../lib/drawer/drawer";
import { Style } from "../lib/drawer/style";
import { Key, KeyState } from "../lib/input/keys";
import { AABB } from "../lib/maths/aabb";
import { Polygon } from "../lib/maths/poly";
import Vec2 from "../lib/maths/vec2";
import { Collision, CollisionManifold, RigidBody, getCollisionManifold } from "./physics";

const WIDTH = 1280;
const HEIGHT = 720;
const root = document.querySelector("main")!;
export const drawer = new Drawer(root, {
    ...DrawerConfigDefault,
    width: WIDTH,
    height: HEIGHT,
});

const rigidBodies = [
    RigidBody.fromRect(900, 10, new Vec2(700, 600), 1, 0.5, 0.25, true),
    RigidBody.fromRect(400, 10, new Vec2(300, 450), 1, 0.5, 0.25, true),
    RigidBody.fromRect(400, 10, new Vec2(900, 200), 1, 0.5, 0.25, true),
];

rigidBodies[1].rotate(0.2);
rigidBodies[2].rotate(-0.2);

const quadtree = new QuadTree<RigidBody>(AABB.fromDimensions(0, 0, WIDTH, HEIGHT));
for (const body of rigidBodies) {
    quadtree.insert(body);
}

for (let i = 0; i < 0; i++) {
    rigidBodies.push(
        RigidBody.fromRect(5, 5, new Vec2(250 + ((i * 33) % 900), ((100 + i * 33) / 900) * 33 + i * 10), 1, 0.3, 0.1)
    );
}

let held: RigidBody | undefined = undefined;

let physicsSteps = 0;
const targetPhysicsStepLength = 1000 / 800;

const RED_STROKE = drawer.getStyleID(new Style(undefined, "darkred", 1));
const GREEN_STROKE = drawer.getStyleID(new Style(undefined, "darkgreen", 1));
const BLUE_FILL_STROKE = drawer.getStyleID(new Style("blue", "darkblue", 2));
const RED_FILL_STROKE = drawer.getStyleID(new Style("red", "darkred", 2));
const GREEN_FILL = drawer.getStyleID(new Style("green", undefined, undefined));
const CYAN_STROKE = drawer.getStyleID(new Style(undefined, "cyan", 1));

drawer.run((dt) => {
    if (dt > 100) {
        console.warn("Discarding step, dt too high:", dt);
        physicsSteps += Math.floor(dt / targetPhysicsStepLength);
        return;
    }

    inputStep();

    const deltaSteps = Math.floor(drawer.getTimeElapsed() / targetPhysicsStepLength) - physicsSteps;
    physicsSteps += deltaSteps;

    const start = performance.now();
    for (let i = 0; i < deltaSteps; i++) {
        physicsStep(dt / 1000 / deltaSteps);
    }
    const physicsDuration = performance.now() - start;

    drawStep();
    drawer.context.font = `1em Consolas`;
    drawer.context.fillText(`Physics time: ${physicsDuration.toFixed(1)}ms`, 10, 20);

    drawer.context.fillText(`Physics steps: ${deltaSteps}`, 10, 40);
    drawer.context.fillText(`Rigid bodies: ${rigidBodies.length}`, 10, 60);
});

function inputStep() {
    const mousePos = drawer.input.getMousePosition();
    const mouseDelta = drawer.input.getMouseMovement();

    switch (drawer.input.getKeyState(Key.MouseLeft)) {
        case KeyState.Pressed:
            for (const body of rigidBodies) {
                if (body.collider.containsPoint(mousePos)) {
                    held = body;
                }
            }
            break;

        case KeyState.Held:
            if (held !== undefined) {
                held.applyForce(Vec2.sub(mousePos, held.position).mulScalar(held.mass * 10), Vec2.zero());
                // held.position.add(mouseDelta);
            }
            break;

        case KeyState.Released:
            held = undefined;
            break;
    }

    if (drawer.input.isKeyPressed(Key.Space)) {
        console.dir(quadtree);
    }

    if (drawer.input.isKeyPressed(Key.Digit1)) {
        const body = RigidBody.fromRect(40, 40, mousePos.clone(), 1, 0.33, 0.33);
        rigidBodies.push(body);
        quadtree.insert(body);
    }

    if (drawer.input.isKeyPressed(Key.Digit2)) {
        const body = RigidBody.fromCircle(35, mousePos.clone(), 1, 0.33, 0.33);
        rigidBodies.push(body);
        quadtree.insert(body);
    }

    if (drawer.input.isKeyPressed(Key.Digit3)) {
        const body = RigidBody.fromRect(90, 30, mousePos.clone(), 1, 0.33, 0.33);
        rigidBodies.push(body);
        quadtree.insert(body);
    }

    if (drawer.input.isKeyPressed(Key.Digit4)) {
        const body = RigidBody.fromCapsule(15, 60, mousePos.clone(), 1, 0.33, 0.33);
        rigidBodies.push(body);
        quadtree.insert(body);
    }

    if (drawer.input.isKeyPressed(Key.Digit5)) {
        const body = RigidBody.fromTriangle(45, 60, mousePos.clone(), 1, 0.33, 0.33);
        rigidBodies.push(body);
        quadtree.insert(body);
    }

    if (drawer.input.isKeyPressed(Key.Digit6)) {
        const body = RigidBody.fromConvexPolygon(
            Polygon.fromHexagon(Vec2.zero(), 25).points,
            mousePos.clone(),
            1,
            0.33,
            0.33
        );
        rigidBodies.push(body);
        quadtree.insert(body);
    }

    if (held !== undefined) {
        if (drawer.input.isKeyHeld(Key.A)) held.applyForce(new Vec2(-held.mass * 1000, 0));
        if (drawer.input.isKeyHeld(Key.D)) held.applyForce(new Vec2(held.mass * 1000, 0));
        if (drawer.input.isKeyHeld(Key.W)) held.applyForce(new Vec2(0, -held.mass * 1000));
        if (drawer.input.isKeyHeld(Key.S)) held.applyForce(new Vec2(0, held.mass * 1000));
        if (drawer.input.isKeyPressed(Key.Space)) held.applyForce(new Vec2(0, -held.mass * 100000));
    }
}

function drawStep() {
    drawer.clear("#eeeeee");

    drawQuadTree(quadtree.root);

    for (const body of rigidBodies) {
        body.collider.draw(drawer, BLUE_FILL_STROKE);
        // body.bounds.draw(drawer, RED_STROKE);
        drawer.drawLine(body.position, Vec2.add(body.position, body.linearVelocity), CYAN_STROKE);
    }
    if (held !== undefined) held.collider.draw(drawer, GREEN_FILL);

    // for (const c of collisions) {
    //     for (const contact of c.contacts) {
    //         drawer.drawCircle(contact.worldPos, 3, RED_FILL_STROKE);
    //         drawer.drawCircle(Vec2.sub(contact.worldPos, c.mtv), 3, RED_FILL_STROKE);
    //     }
    //     if (c.contacts.length === 2) {
    //         drawer.drawLine(c.contacts[0].worldPos, c.contacts[1].worldPos, RED_FILL_STROKE);
    //         drawer.drawLine(Vec2.sub(c.contacts[0].worldPos, c.mtv), Vec2.sub(c.contacts[1].worldPos, c.mtv), RED_FILL_STROKE);
    //     }
    //     drawer.drawLine(c.contacts[0].worldPos, Vec2.add(c.contacts[0].worldPos, c.mtv), RED_FILL_STROKE);
    // }

    drawer.drawFps(2);
    drawer.drawFrametimeGraph(2);
}

const collisions: Collision[] = [];

function physicsStep(deltaSeconds: number) {
    for (const body of rigidBodies) {
        if (body.fixed) continue;
        body.update(deltaSeconds);

        const gravity = new Vec2(0, 981 * body.mass);
        body.applyForce(gravity, Vec2.zero());

        // // crude wrap around
        const wrap = new Vec2(0, 0);
        if (body.bounds.min.x > WIDTH) wrap.x = -body.bounds.max.x;
        if (body.bounds.min.y > HEIGHT) wrap.y = -body.bounds.max.y;
        if (body.bounds.max.x < 0) wrap.x = WIDTH - body.bounds.min.x;
        if (body.bounds.max.y < 0) wrap.y = HEIGHT - body.bounds.min.y;
        body.translate(wrap);

        if (body.linearVelocity.x > 0 || body.linearVelocity.y > 0 || body.angularVelocity > 0) {
            quadtree.update(body);
        }
    }

    for (const bodyA of rigidBodies) {
        for (const bodyB of quadtree.get(bodyA.bounds)) {
            if ((bodyA.fixed && bodyB.fixed) || bodyB.id <= bodyA.id) continue;

            let manifold = getCollisionManifold(bodyA, bodyB);
            if (manifold !== undefined) collisions.push(new Collision(bodyA, bodyB, manifold));
        }
    }

    // for (let i = 0; i < rigidBodies.length; i++) {
    //     for (let j = i + 1; j < rigidBodies.length; j++) {
    //         const bodyA = rigidBodies[i];
    //         const bodyB = rigidBodies[j];

    //         if (bodyA.fixed && bodyB.fixed) continue;
    //         if (!bodyA.bounds.intersects(bodyB.bounds)) continue;

    //         let collision = getCollisionManifold(bodyA, bodyB);
    //         if (collision !== undefined) collisions.push(collision);
    //     }
    // }

    for (const collision of collisions) {
        collision.resolve();
    }

    collisions.length = 0;
}

function drawQuadTree(quadtree: QuadTreeNode<any>) {
    quadtree.bounds.draw(drawer, GREEN_STROKE);

    for (const child of quadtree.children) {
        if (child !== undefined) {
            drawQuadTree(child);
        }
    }
}
