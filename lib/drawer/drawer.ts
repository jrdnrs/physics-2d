import { TAU, average } from "../maths/util";
import Input from "../input/input";
import Vec2 from "../maths/vec2";
import {
    DrawCapsuleCommand,
    DrawCircleCommand,
    DrawCommand,
    DrawCommandList,
    DrawPolyLineCommand,
    DrawPolygonCommand,
    DrawRectCommand,
} from "./command";
import { Style, StyleID } from "./style";

/**
 *  number of previous frametimes to cache
 **/
const FT_SIZE = 64;

export default class Drawer {
    private userMainLoop: ((dt: number) => void) | undefined;

    private drawCommandListPool: DrawCommandList[];
    private styleRegistry: Map<Style, StyleID>;
    private defaultStyle: StyleID;

    readonly config: DrawerConfig;
    canvas: HTMLCanvasElement;
    context: CanvasRenderingContext2D;
    input: Input;

    private currentSync: number;
    private timeElapsed: number;
    private frames: number;
    private fps: number;
    private frametimes: number[];
    private averageFrametime: number;
    private targetFrametime: number;

    constructor(container: HTMLElement, config: DrawerConfig = DrawerConfigDefault) {
        this.config = config;
        this.canvas = document.createElement("canvas");
        container.replaceChildren(this.canvas);

        // disable right click context menu
        this.canvas.addEventListener("contextmenu", (ev) => ev.preventDefault());

        this.canvas.width = this.config.width;
        this.canvas.height = this.config.height;

        // set scaling
        this.canvas.style.transformOrigin = "top left";
        if (this.config.nearestScaling) this.canvas.style.imageRendering = "pixelated";
        this.canvas.style.scale = this.config.scale.toString();

        this.context = this.canvas.getContext("2d", {
            alpha: this.config.alpha,
        })!;
        this.input = new Input(this.canvas, this.config.lockPointer);

        this.currentSync = 0;
        this.timeElapsed = 0;
        this.frames = 0;
        this.fps = 0;
        this.frametimes = new Array(FT_SIZE).fill(0);
        this.averageFrametime = 0;
        this.targetFrametime = this.config.fpsLimit <= 0 ? 0.1 : 1000 / this.config.fpsLimit;

        this.drawCommandListPool = [];
        this.styleRegistry = new Map();
        this.defaultStyle = this.getStyleID({
            fill: undefined,
            stroke: "#dea300",
            strokeWidth: 1,
        });
    }

    private main(dt: number) {
        this.userMainLoop!(dt);

        this.executeDrawCommands(this.context);
        this.clearDrawCommands();

        this.input.update();
        this.frames += 1;
    }

    private updateTimings(time: number): number {
        const dt = time - this.timeElapsed;
        this.frametimes[this.frames % FT_SIZE] = dt;
        this.averageFrametime = average(this.frametimes);
        this.fps = 1000 / this.averageFrametime;
        this.timeElapsed = time;

        return dt;
    }

    private loop(time: number) {
        const dt = this.updateTimings(time);
        this.main(dt);

        const sleep = time + this.targetFrametime - performance.now();
        setTimeout(() => this.loop(performance.now()), sleep);
    }

    private loopSync(time: number) {
        this.currentSync += 1;

        if (this.currentSync === this.config.syncInterval) {
            this.currentSync = 0;
            const dt = this.updateTimings(time);
            this.main(dt);
        }

        requestAnimationFrame((time) => this.loopSync(time));
    }

    private clearDrawCommands() {
        for (const list of this.drawCommandListPool) {
            list.clear();
        }
    }

    private executeDrawCommands(context: CanvasRenderingContext2D) {
        for (const list of this.drawCommandListPool) {
            list.execute(context);
        }
    }

    run(mainLoop: (dt: number) => void) {
        this.userMainLoop = mainLoop;

        requestAnimationFrame((time) => {
            if (this.config.syncInterval == 0) {
                this.loop(time);
            } else {
                this.loopSync(time);
            }
        });
    }

    getTimeElapsed() {
        return this.timeElapsed;
    }

    getFramesElapsed() {
        return this.frames;
    }

    clear(colour: string) {
        this.context.fillStyle = colour;
        this.context.fillRect(0, 0, this.config.width, this.config.height);
    }

    getStyleID(style: Style): StyleID {
        if (this.styleRegistry.has(style)) {
            return this.styleRegistry.get(style)!;
        }

        const id = this.styleRegistry.size;
        this.styleRegistry.set(style, id);
        this.drawCommandListPool.push(new DrawCommandList(style));

        return id;
    }

    submitDrawCommand(style: StyleID, command: DrawCommand) {
        this.drawCommandListPool[style].add(command);
    }

    drawLine(start: Vec2, end: Vec2, style: StyleID) {
        const command = new DrawPolyLineCommand([start, end]);
        this.submitDrawCommand(style, command);
    }

    drawPolyLine(points: Vec2[], style: StyleID) {
        const command = new DrawPolyLineCommand(points);
        this.submitDrawCommand(style, command);
    }

    drawPolygon(points: Vec2[], style: StyleID) {
        const command = new DrawPolygonCommand(points);
        this.submitDrawCommand(style, command);
    }

    drawCircle(centre: Vec2, radius: number, style: StyleID) {
        const command = new DrawCircleCommand(centre, radius);
        this.submitDrawCommand(style, command);
    }

    drawRect(min: Vec2, max: Vec2, style: StyleID) {
        const command = new DrawRectCommand(min, max);
        this.submitDrawCommand(style, command);
    }

    drawCapsule(rectangle: Vec2[], circleCentres: Vec2[], radius: number, angle: number, style: StyleID) {
        const command = new DrawCapsuleCommand(rectangle, circleCentres, radius, angle);
        this.submitDrawCommand(style, command);
    }

    drawFps(scale: number = 1) {
        this.context.font = `${scale * 0.5}em Consolas`;
        this.context.fillStyle = "#dea300";

        const x = this.config.width - 20 * scale;
        const y = 10 * scale;

        this.context.fillText(this.fps.toFixed(), x, y);
    }

    drawFrametimeGraph(scale: number = 1) {
        this.context.font = `${scale * 0.3}em Consolas`;
        this.context.fillStyle = "#dea300";

        const axisX = this.config.width - 100 * scale;

        // Axis labels
        this.context.fillText(" 0-", axisX, 12 * scale);
        this.context.fillText("10-", axisX, 22 * scale);
        this.context.fillText("20-", axisX, 32 * scale);
        this.context.fillText("30-", axisX, 42 * scale);
        this.context.fillText("40-", axisX, 52 * scale);

        // Average frametime
        this.context.fillText(this.averageFrametime.toFixed(1) + "ms", this.config.width - 20 * scale, 18 * scale);

        const plotX = this.config.width - 90 * scale;
        const plotY = 10 * scale;

        let FTpoints = Array.from(new Array(60), (_, i) => {
            const x = plotX + i * scale;
            // Cap frametime to 40ms so it doesn't go off the axis
            const y = plotY + Math.min(40, this.frametimes[(this.frames - i) % FT_SIZE]) * scale;
            return new Vec2(x, y);
        });

        this.drawPolyLine(FTpoints, this.defaultStyle);
    }
}

export type DrawerConfig = {
    width: number;
    height: number;
    /**
     * Scale the canvas by this factor.
     */
    scale: number;
    /**
     * Use nearest neighbour scaling (bilinear by default).
     */
    nearestScaling: boolean;
    /**
     * Enable alpha channel for canvas background.
     */
    alpha: boolean;
    /**
     * Lock the mouse pointer to the canvas on Left Mouse click.
     */
    lockPointer: boolean;
    /**
     * Sync every n frames.
     *
     * 0 = no sync
     */
    syncInterval: number;
    /**
     * Limit FPS when syncInterval is set to 0. This relies on `setTimeout`,
     * so it's not very accurate! Use syncInterval instead.
     *
     * 0 = no limit
     */
    fpsLimit: number;
};

export const DrawerConfigDefault: DrawerConfig = {
    width: 800,
    height: 600,
    scale: 1,
    nearestScaling: false,
    alpha: false,
    lockPointer: false,
    syncInterval: 1,
    fpsLimit: 0,
};
