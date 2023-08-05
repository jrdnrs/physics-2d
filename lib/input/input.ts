import Vec2 from "../maths/vec2";
import { KeyboardCodes, Key, InternalKeyState, KeyState, KeyboardCodeMap, MouseButtonCodes } from "./keys";

const KEYBOARD_KEYS = KeyboardCodes.length;
const MOUSE_BUTTONS = MouseButtonCodes.length;

export default class Input {
    private canvas: HTMLCanvasElement;
    private keyStates: InternalKeyState[];
    private mousePos: Vec2;
    private mouseDelta: Vec2;
    private mouseInCanvas: boolean;
    private shouldLockPointer: boolean;
    private pointerLocked: boolean;

    constructor(canvas: HTMLCanvasElement, shouldLockPointer: boolean = false) {
        this.canvas = canvas;
        this.pointerLocked = false;
        this.mouseInCanvas = false;
        this.mousePos = Vec2.zero();
        this.mouseDelta = Vec2.zero();
        this.shouldLockPointer = shouldLockPointer;
        this.keyStates = Array.from(new Array(KEYBOARD_KEYS + MOUSE_BUTTONS), () => {
            return {
                held: false,
                pressed: false,
                released: false,
            };
        });

        document.addEventListener("keydown", this.handleKeyDown.bind(this));
        document.addEventListener("keyup", this.handleKeyUp.bind(this));
        this.canvas.addEventListener("mouseenter", (ev) => (this.mouseInCanvas = true));
        this.canvas.addEventListener("mouseleave", (ev) => (this.mouseInCanvas = false));

        if (this.shouldLockPointer) {
            document.addEventListener("pointerlockchange", (ev) => {
                this.pointerLocked = document.pointerLockElement === this.canvas;
            });

            this.canvas.addEventListener("mousedown", (ev) => {
                if (!this.pointerLocked) {
                    this.canvas.requestPointerLock();
                    return;
                }
                this.handleMouseDown(ev);
            });

            this.canvas.addEventListener("mouseup", (ev) => {
                if (!this.pointerLocked) return;
                this.handleMouseUp(ev);
            });

            this.canvas.addEventListener("mousemove", (ev) => {
                if (!this.pointerLocked) return;
                this.handleMouseMove(ev);
            });
        } else {
            this.canvas.addEventListener("mousedown", this.handleMouseDown.bind(this));
            this.canvas.addEventListener("mouseup", this.handleMouseUp.bind(this));
            this.canvas.addEventListener("mousemove", this.handleMouseMove.bind(this));
        }
    }

    private handleMouseDown(ev: MouseEvent) {
        this.keyStates[KEYBOARD_KEYS + ev.button].held = true;
        this.keyStates[KEYBOARD_KEYS + ev.button].pressed = true;
    }

    private handleMouseUp(ev: MouseEvent) {
        this.keyStates[KEYBOARD_KEYS + ev.button].held = false;
        this.keyStates[KEYBOARD_KEYS + ev.button].released = true;
    }

    private handleMouseMove(ev: MouseEvent) {
        this.mouseDelta.x += ev.movementX;
        this.mouseDelta.y += ev.movementY;

        this.mousePos.x = ev.clientX;
        this.mousePos.y = ev.clientY;
    }

    private handleKeyDown(ev: KeyboardEvent) {
        ev.preventDefault();
        // multiple events will be fires if a key is held down, but we just want the first one
        if (ev.repeat) return;

        // Unfortunaterly, `keyCode` is deprecated, so we have to use `code` instead
        // which is a string, so we are relying on a Map for the index
        const i = KeyboardCodeMap.get(ev.code)!;

        this.keyStates[i].held = true;
        this.keyStates[i].pressed = true;
    }

    private handleKeyUp(ev: KeyboardEvent) {
        ev.preventDefault();
        const i = KeyboardCodeMap.get(ev.code)!;
        this.keyStates[i].held = false;
        this.keyStates[i].released = true;
    }

    /**
     * Resets the `pressed` and `released` states, which denotes whether the key was first pressed/released
     * since the previous frame, for every key.
     *
     * This should be called at the _end_ of every frame, after input has been processed.
     */
    update() {
        for (const keyState of this.keyStates) {
            keyState.pressed = false;
            keyState.released = false;
        }

        this.mouseDelta.x = 0;
        this.mouseDelta.y = 0;
    }

    isPointerLocked(): boolean {
        return this.pointerLocked;
    }

    /**
     * Returns `true` if the specified key is currently held down, and `false` otherwise
     */
    isKeyHeld(key: Key): boolean {
        return this.keyStates[key].held;
    }

    /**
     * Returns `true` if the specified key was pressed since the last frame, and `false` otherwise
     */
    isKeyPressed(key: Key): boolean {
        return this.keyStates[key].pressed;
    }

    /**
     * Returns `true` if the specified key was released since the last frame, and `false` otherwise
     */
    isKeyReleased(key: Key): boolean {
        return this.keyStates[key].released;
    }

    getKeyState(key: Key): KeyState {
        const internalState = this.keyStates[key];
        if (internalState.pressed) return KeyState.Pressed;
        if (internalState.held) return KeyState.Held;
        if (internalState.released) return KeyState.Released;
        return KeyState.Idle;
    }

    getMouseMovement(): Vec2 {
        return this.mouseDelta.clone();
    }

    getMousePosition(): Vec2 {
        const bounds = this.canvas.getBoundingClientRect();
        const offset = new Vec2(bounds.left, bounds.top);

        return Vec2.sub(this.mousePos, offset);
    }

    isMouseInCanvas(): boolean {
        return this.mouseInCanvas;
    }
}
