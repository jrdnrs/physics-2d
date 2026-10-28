import { QuadTreeNode } from "../lib/collections/quadtree";
import Drawer, { DrawerConfigDefault } from "../lib/drawer/drawer";
import { Style, StyleID } from "../lib/drawer/style";
import { Key, KeyState } from "../lib/input/keys";
import { AABB } from "../lib/maths/aabb";
import { Polygon } from "../lib/maths/poly";
import Vec2 from "../lib/maths/vec2";
import { RigidBody } from "./body";
import { PhysicsEngine, PhysicsEngineConfigDefault } from "./physics";

let RED_STROKE: number;
let GREEN_STROKE: number;
let BLUE_FILL_STROKE: number;
let DARK_BLUE_FILL_STROKE: number;
let GREEN_FILL: number;
let GREEN_FILL_STROKE: number;
let CYAN_STROKE: number;
let RED_FILL_STROKE: number;

export class PhysicsDemo {
    container: HTMLElement;
    physicsEngine: PhysicsEngine;
    drawer: Drawer;

    worldHeight: number;
    worldWidth: number;

    showQuadTree: boolean;
    showBounds: boolean;
    showVelocity: boolean;
    showContacts: boolean;

    held: Spring | undefined;

    constructor() {
        this.container = document.querySelector("main")!;

        this.worldHeight = 720;
        this.worldWidth = 1280;

        this.showQuadTree = true;
        this.showBounds = true;
        this.showVelocity = true;
        this.showContacts = true;

        this.physicsEngine = new PhysicsEngine(AABB.fromDimensions(0, 0, this.worldWidth, this.worldHeight), {
            ...PhysicsEngineConfigDefault,
            stepsPerSecond: 333,
            velocityIterations: 5,
        });

        this.drawer = new Drawer(this.container, {
            ...DrawerConfigDefault,
            width: this.worldWidth,
            height: this.worldHeight,
        });

        RED_STROKE = this.drawer.getStyleID(new Style(undefined, "darkred", 1));
        GREEN_STROKE = this.drawer.getStyleID(new Style(undefined, "darkgreen", 1));
        BLUE_FILL_STROKE = this.drawer.getStyleID(new Style("blue", "darkblue", 2));
        DARK_BLUE_FILL_STROKE = this.drawer.getStyleID(new Style("darkblue", "black", 2));
        GREEN_FILL = this.drawer.getStyleID(new Style("green", undefined, undefined));
        GREEN_FILL_STROKE = this.drawer.getStyleID(new Style("green", "darkgreen", 2));
        CYAN_STROKE = this.drawer.getStyleID(new Style(undefined, "cyan", 1));
        RED_FILL_STROKE = this.drawer.getStyleID(new Style("red", "darkred", 2));

        // Some default fixed bodies
        const rigidBodies = [
            RigidBody.fromRect(900, 10, new Vec2(700, 600), 1, 0.5, 0.5, true),
            RigidBody.fromRect(400, 10, new Vec2(300, 450), 1, 0.5, 0.5, true),
            RigidBody.fromRect(400, 10, new Vec2(900, 200), 1, 0.5, 0.5, true),
        ];

        rigidBodies[1].rotate(0.2);
        rigidBodies[2].rotate(-0.2);

        for (const body of rigidBodies) {
            this.physicsEngine.addBody(body);
        }
    }

    run() {
        this.drawer.run((deltaMs) => {
            if (deltaMs > 100) {
                console.warn("Discarding step, dt too high:", deltaMs);
                return;
            }

            this.updateInput();
            this.updateSimulation(deltaMs);
            this.draw();
        });
    }

    private updateSimulation(deltaMs: number) {
        const deltaSeconds = deltaMs / 1000;

        if (this.held !== undefined) {
            const mousePos = this.drawer.input.getMousePosition();
            const mouseDelta = this.drawer.input.getMouseMovement();

            this.held.apply(mousePos, mouseDelta, deltaSeconds);
        }

        const now = performance.now();
        const deltaSteps = this.physicsEngine.update(deltaSeconds);
        const duration = performance.now() - now;
    }

    private updateInput() {
        const mousePos = this.drawer.input.getMousePosition();

        switch (this.drawer.input.getKeyState(Key.MouseLeft)) {
            case KeyState.Pressed:
                for (const body of this.physicsEngine.bodies) {
                    if (!body.fixed && body.bounds.containsPoint(mousePos) && body.collider.containsPoint(mousePos)) {
                        const radius = Vec2.sub(mousePos, body.position).rotate(-body.rotation);
                        this.held = new Spring(radius, body, 500, 30);
                    }
                }
                break;

            case KeyState.Held:
                if (this.held === undefined) break;
                const displacement = Vec2.sub(mousePos, this.held.body.position);
                if (displacement.magnitudeSquared() > (500 * 500)) this.held = undefined;
                break;

            case KeyState.Released:
                this.held = undefined;
                break;
        }

        if (this.drawer.input.isKeyPressed(Key.Digit1)) {
            const body = RigidBody.fromRect(40, 40, mousePos.clone(), 1, 0.33, 0.33);
            this.physicsEngine.addBody(body);
        }

        if (this.drawer.input.isKeyPressed(Key.Digit2)) {
            const body = RigidBody.fromCircle(35, mousePos.clone(), 1, 0.33, 0.33);
            this.physicsEngine.addBody(body);
        }

        if (this.drawer.input.isKeyPressed(Key.Digit3)) {
            const body = RigidBody.fromRect(90, 30, mousePos.clone(), 1, 0.33, 0.33);
            this.physicsEngine.addBody(body);
        }

        if (this.drawer.input.isKeyPressed(Key.Digit4)) {
            const body = RigidBody.fromCapsule(15, 60, mousePos.clone(), 1, 0.33, 0.33);
            this.physicsEngine.addBody(body);
        }

        if (this.drawer.input.isKeyPressed(Key.Digit5)) {
            const body = RigidBody.fromTriangle(45, 60, mousePos.clone(), 1, 0.33, 0.33);
            this.physicsEngine.addBody(body);
        }

        if (this.drawer.input.isKeyPressed(Key.Digit6)) {
            const body = RigidBody.fromConvexPolygon(
                Polygon.fromNGon(Vec2.zero(), 25, 6).points,
                mousePos.clone(),
                1,
                0.33,
                0.75
            );
            this.physicsEngine.addBody(body);
        }
    }

    private draw() {
        this.drawer.clear("#eeeeee");

        if (this.showQuadTree) this.drawQuadTree();
        this.drawShapes();
        if (this.showContacts) this.drawCollisions();
        this.drawUi();


        // temp draw spring
        if (this.held !== undefined) {
            let radius = Vec2.rotate(this.held.radius, this.held.body.rotation);
            let attach = Vec2.add(this.held.body.position, radius);
            this.drawer.drawLine(attach, this.drawer.input.getMousePosition(), RED_FILL_STROKE);
        }
    }

    private drawUi() {
        this.drawer.context.font = `1em Consolas`;

        this.drawer.context.fillStyle = "#dea300";
        // this.drawer.context.fillText(`Physics calc time: ${this.physicsEngine.updateDuration.toFixed(1)}ms`, 10, 20);
        // this.drawer.context.fillText(`Physics steps per frame: ${deltaSteps}`, 10, 40);
        this.drawer.context.fillText(`Rigid bodies: ${this.physicsEngine.bodies.length}`, 10, 60);
        this.drawer.context.fillText(`Active Islands: ${this.physicsEngine.islands.length}`, 10, 80);

        this.drawer.drawFps(2);
        this.drawer.drawFrametimeGraph(2);
    }

    private drawShapes() {
        for (const body of this.physicsEngine.bodies) {
            if (body.sleeping) {
                body.collider.draw(this.drawer, DARK_BLUE_FILL_STROKE);
            } else {
                body.collider.draw(this.drawer, BLUE_FILL_STROKE);

                if (this.showBounds) {
                    body.bounds.draw(this.drawer, RED_STROKE);
                }

                if (this.showVelocity) {
                    this.drawer.drawLine(body.position, Vec2.add(body.position, body.linearVelocity), CYAN_STROKE);
                }
            }
        }
    }

    private drawCollisions() {
        for (const collision of this.physicsEngine.collisions.values()) {
            const manifold = collision.manifold;

            // draw individual contact points
            for (const contact of manifold.contacts) {
                this.drawer.drawCircle(contact.worldPosA, 3, RED_FILL_STROKE);
                this.drawer.drawCircle(Vec2.sub(contact.worldPosA, manifold.mtv), 3, RED_FILL_STROKE);
            }

            // draw lines between multiple contacts
            if (manifold.contacts.length === 2) {
                this.drawer.drawLine(manifold.contacts[0].worldPosA, manifold.contacts[1].worldPosA, RED_FILL_STROKE);
                this.drawer.drawLine(
                    Vec2.sub(manifold.contacts[0].worldPosA, manifold.mtv),
                    Vec2.sub(manifold.contacts[1].worldPosA, manifold.mtv),
                    RED_FILL_STROKE
                );
            }

            // draw mtv
            // this.drawer.drawLine(
            //     manifold.contacts[0].worldPosA,
            //     Vec2.add(manifold.contacts[0].worldPosA, manifold.mtv),
            //     RED_FILL_STROKE
            // );
        }
    }

    private drawQuadTree() {
        const inner = (quadTree: QuadTreeNode<any>) => {
            quadTree.bounds.draw(this.drawer, GREEN_STROKE);
            for (const child of quadTree.children) {
                if (child !== undefined) inner(child);
            }
        };

        inner(this.physicsEngine.quadtree.root);
    }
}

class Spring {
    radius: Vec2;
    body: RigidBody;
    k: number 
    d: number;

    constructor(radius: Vec2, body: RigidBody, k: number, d: number) {
        this.radius = radius;
        this.body = body;
        this.k = k;
        this.d = d;
    }

    apply(anchor: Vec2, anchorVelocity: Vec2, deltaSeconds: number) {

        // off-centre grab
        let radius = Vec2.rotate(this.radius, this.body.rotation);
        let attach = Vec2.add(this.body.position, radius);
        let vel = this.body.getVelocityAtPoint(radius);

        // centre grab
        // let radius = Vec2.zero();
        // let attach = this.body.position;
        // let vel = this.body.linearVelocity;

        const displacement = Vec2.sub(anchor, attach);
        const distance = displacement.magnitude();
        const direction = distance === 0 ? displacement : Vec2.divScalar(displacement, distance);
        const slack = 200;

        if (distance <= slack) {
            return;
        }

        this.body.sleeping = false;


        // const springForce = Vec2.sub(anchor, attach).mulScalar(this.k);
        // const dampingForce = Vec2.sub(anchorVelocity, vel).mulScalar(this.d);

        const springForce = Vec2.mulScalar(direction,  (distance - slack)  * this.k);
        const dampingForce = Vec2.mulScalar(direction, Vec2.sub(anchorVelocity, vel).dot(direction) * this.d);

        this.body.applyImpulse(springForce.add(dampingForce).mulScalar(0.9 * this.body.mass * deltaSeconds), radius);
    }
}
