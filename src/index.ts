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

import { QuadTreeNode } from "../lib/collections/quadtree";
import Drawer, { DrawerConfigDefault } from "../lib/drawer/drawer";
import { Style } from "../lib/drawer/style";
import { Key, KeyState } from "../lib/input/keys";
import { AABB } from "../lib/maths/aabb";
import { Polygon } from "../lib/maths/poly";
import Vec2 from "../lib/maths/vec2";
import { RigidBody } from "./body";
import { PhysicsEngine } from "./physics";

const WIDTH = 1280;
const HEIGHT = 720;
const root = document.querySelector("main")!;
export const drawer = new Drawer(root, {
    ...DrawerConfigDefault,
    width: WIDTH,
    height: HEIGHT,
    syncInterval: 1,
});

const rigidBodies = [
    RigidBody.fromRect(900, 10, new Vec2(700, 600), 1, 0.5, 0.5, true),
    RigidBody.fromRect(400, 10, new Vec2(300, 450), 1, 0.5, 0.5, true),
    RigidBody.fromRect(400, 10, new Vec2(900, 200), 1, 0.5, 0.5, true),
];

rigidBodies[1].rotate(0.2);
rigidBodies[2].rotate(-0.2);

for (let i = 0; i < 0; i++) {
    rigidBodies.push(
        RigidBody.fromRect(10, 10, new Vec2(250 + ((i * 33) % 900), ((100 + i * 33) / 900) * 33 + i * 10), 1, 0.9, 0.1)
    );
}

let held: RigidBody | undefined = undefined;

const RED_STROKE = drawer.getStyleID(new Style(undefined, "darkred", 1));
const GREEN_STROKE = drawer.getStyleID(new Style(undefined, "darkgreen", 1));
const BLUE_FILL_STROKE = drawer.getStyleID(new Style("blue", "darkblue", 2));
const DARK_BLUE_FILL_STROKE = drawer.getStyleID(new Style("darkblue", "black", 2));
const GREEN_FILL = drawer.getStyleID(new Style("green", undefined, undefined));
const GREEN_FILL_STROKE = drawer.getStyleID(new Style("green", "darkgreen", 2));
const CYAN_STROKE = drawer.getStyleID(new Style(undefined, "cyan", 1));
const RED_FILL_STROKE = drawer.getStyleID(new Style("red", "darkred", 2));

const physicsEngine = new PhysicsEngine(AABB.fromDimensions(0, 0, WIDTH, HEIGHT));
for (const body of rigidBodies) {
    physicsEngine.addBody(body);
}

drawer.run((dt) => {
    if (dt > 100) {
        console.warn("Discarding step, dt too high:", dt);
        return;
    }

    updateInput();
    const deltaSteps = physicsEngine.update(dt / 1000);
    updateDraw();
    drawer.context.font = `1em Consolas`;
    drawer.context.fillText(`Physics time: ${physicsEngine.updateDuration.toFixed(1)}ms`, 10, 20);

    drawer.context.fillText(`Physics steps: ${deltaSteps}`, 10, 40);
    drawer.context.fillText(`Rigid bodies: ${physicsEngine.bodies.length}`, 10, 60);
    drawer.context.fillText(`Islands: ${physicsEngine.islands.length}`, 10, 80);
});

function updateInput() {
    const mousePos = drawer.input.getMousePosition();
    const mouseDelta = drawer.input.getMouseMovement();

    switch (drawer.input.getKeyState(Key.MouseLeft)) {
        case KeyState.Pressed:
            for (const body of physicsEngine.bodies) {
                if (body.bounds.containsPoint(mousePos) && body.collider.containsPoint(mousePos)) {
                    held = body;
                }
            }
            break;

        case KeyState.Held:
            if (held !== undefined) {
                held.translate(mouseDelta);
                physicsEngine.quadtree.update(held);
            }
            break;

        case KeyState.Released:
            held = undefined;
            break;
    }

    if (drawer.input.isKeyHeld(Key.A)) {
        if (held) {
            held.rotate(-0.01);
            physicsEngine.quadtree.update(held);
        }
    }

    if (drawer.input.isKeyHeld(Key.D)) {
        if (held) {
            held.rotate(0.01);
            physicsEngine.quadtree.update(held);
        }
    }

    if (drawer.input.isKeyPressed(Key.Digit1)) {
        const body = RigidBody.fromRect(40, 40, mousePos.clone(), 1, 0.33, 0.33);
        physicsEngine.addBody(body);
    }

    if (drawer.input.isKeyPressed(Key.Digit2)) {
        const body = RigidBody.fromCircle(35, mousePos.clone(), 1, 0.33, 0.33);
        physicsEngine.addBody(body);
    }

    if (drawer.input.isKeyPressed(Key.Digit3)) {
        const body = RigidBody.fromRect(90, 30, mousePos.clone(), 1, 0.33, 0.33);
        physicsEngine.addBody(body);
    }

    if (drawer.input.isKeyPressed(Key.Digit4)) {
        const body = RigidBody.fromCapsule(15, 60, mousePos.clone(), 1, 0.33, 0.33);
        physicsEngine.addBody(body);
    }

    if (drawer.input.isKeyPressed(Key.Digit5)) {
        const body = RigidBody.fromTriangle(45, 60, mousePos.clone(), 1, 0.33, 0.33);
        physicsEngine.addBody(body);
    }

    if (drawer.input.isKeyPressed(Key.Digit6)) {
        const body = RigidBody.fromConvexPolygon(
            Polygon.fromNGon(Vec2.zero(), 25, 6).points,
            mousePos.clone(),
            1,
            0.33,
            0.75
        );
        physicsEngine.addBody(body);
    }

    if (held !== undefined) {
        console.log(held.linearVelocity, held.angularVelocity);
    }
}

function updateDraw() {
    drawer.clear("#eeeeee");

    // drawQuadTree(physicsEngine.quadtree.root);

    for (const body of physicsEngine.bodies) {
        if (body.sleeping) {
            body.collider.draw(drawer, DARK_BLUE_FILL_STROKE);
        } else {
            body.collider.draw(drawer, BLUE_FILL_STROKE);
            // body.bounds.draw(drawer, RED_STROKE);
            // drawer.drawLine(body.position, Vec2.add(body.position, body.linearVelocity), CYAN_STROKE);
        }
    }
    if (held !== undefined) held.collider.draw(drawer, GREEN_FILL);

    // for (const collision of physicsEngine.collisions.values()) {
    //     const manifold = collision.manifold;

    //     // draw individual contact points
    //     for (const contact of manifold.contacts) {
    //         drawer.drawCircle(contact.worldPosA, 3, RED_FILL_STROKE);
    //         drawer.drawCircle(Vec2.sub(contact.worldPosA, manifold.mtv), 3, RED_FILL_STROKE);
    //     }

    //     // draw lines between multiple contacts
    //     if (manifold.contacts.length === 2) {
    //         drawer.drawLine(manifold.contacts[0].worldPosA, manifold.contacts[1].worldPosA, RED_FILL_STROKE);
    //         drawer.drawLine(
    //             Vec2.sub(manifold.contacts[0].worldPosA, manifold.mtv),
    //             Vec2.sub(manifold.contacts[1].worldPosA, manifold.mtv),
    //             RED_FILL_STROKE
    //         );
    //     }

    //     // draw mtv
    //     drawer.drawLine(
    //         manifold.contacts[0].worldPosA,
    //         Vec2.add(manifold.contacts[0].worldPosA, manifold.mtv),
    //         RED_FILL_STROKE
    //     );
    // }

    drawer.drawFps(2);
    drawer.drawFrametimeGraph(2);
}

function drawQuadTree(quadtree: QuadTreeNode<any>) {
    quadtree.bounds.draw(drawer, GREEN_STROKE);

    for (const child of quadtree.children) {
        if (child !== undefined) {
            drawQuadTree(child);
        }
    }
}
