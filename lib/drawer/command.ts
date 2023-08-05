import Segment from "../maths/segment";
import { HALF_PI, PI, TAU } from "../maths/util";
import Vec2 from "../maths/vec2";
import { Style } from "./style";

export interface DrawCommand {
    execute(context: CanvasRenderingContext2D): void;
}

export class DrawPolyLineCommand implements DrawCommand {
    readonly points: Vec2[];

    constructor(points: Vec2[]) {
        this.points = points;
    }

    execute(context: CanvasRenderingContext2D) {
        context.moveTo(this.points[0].x, this.points[0].y);

        for (let i = 1; i < this.points.length; i++) {
            context.lineTo(this.points[i].x, this.points[i].y);
        }
    }
}

export class DrawPolygonCommand implements DrawCommand {
    readonly points: Vec2[];

    constructor(points: Vec2[]) {
        this.points = points;
    }

    execute(context: CanvasRenderingContext2D) {
        context.moveTo(this.points[0].x, this.points[0].y);

        for (let i = 1; i < this.points.length; i++) {
            context.lineTo(this.points[i].x, this.points[i].y);
        }

        context.lineTo(this.points[0].x, this.points[0].y);
    }
}

export class DrawCircleCommand implements DrawCommand {
    readonly centre: Vec2;
    readonly radius: number;

    constructor(centre: Vec2, radius: number) {
        this.centre = centre;
        this.radius = radius;
    }

    execute(context: CanvasRenderingContext2D) {
        context.moveTo(this.centre.x + this.radius, this.centre.y);
        context.arc(this.centre.x, this.centre.y, this.radius, 0, TAU);
    }
}

export class DrawCapsuleCommand implements DrawCommand {
    readonly rectangle: Vec2[];
    readonly circleCentres: Vec2[];
    readonly radius: number;
    readonly angle: number;

    constructor(rectangle: Vec2[], circleCentres: Vec2[], radius: number, angle: number) {
        this.rectangle = rectangle;
        this.circleCentres = circleCentres;
        this.radius = radius;
        this.angle = angle;
    }

    execute(context: CanvasRenderingContext2D) {
        context.moveTo(this.rectangle[0].x, this.rectangle[0].y);
        context.lineTo(this.rectangle[1].x, this.rectangle[1].y);
        context.arc(
            this.circleCentres[0].x,
            this.circleCentres[0].y,
            this.radius,
            this.angle - HALF_PI,
            this.angle + HALF_PI
        );
        context.lineTo(this.rectangle[3].x, this.rectangle[3].y);
        context.arc(
            this.circleCentres[1].x,
            this.circleCentres[1].y,
            this.radius,
            this.angle + HALF_PI,
            this.angle - HALF_PI
        );
    }
}

export class DrawRectCommand implements DrawCommand {
    readonly min: Vec2;
    readonly max: Vec2;

    constructor(min: Vec2, max: Vec2) {
        this.min = min;
        this.max = max;
    }

    execute(context: CanvasRenderingContext2D) {
        const width = this.max.x - this.min.x;
        const height = this.max.y - this.min.y;

        context.moveTo(this.min.x, this.min.y);
        context.lineTo(this.min.x + width, this.min.y);
        context.lineTo(this.min.x + width, this.min.y + height);
        context.lineTo(this.min.x, this.min.y + height);
        context.lineTo(this.min.x, this.min.y);
    }
}

export class DrawCommandList {
    readonly style: Style;
    commands: DrawCommand[];

    constructor(style: Style) {
        this.style = style;
        this.commands = [];
    }

    add(command: DrawCommand) {
        this.commands.push(command);
    }

    clear() {
        this.commands.length = 0;
    }

    execute(context: CanvasRenderingContext2D) {
        context.beginPath();
        for (const command of this.commands) {
            command.execute(context);
        }

        if (this.style.fill !== undefined) {
            context.fillStyle = this.style.fill;
            context.fill();
        }

        if (this.style.stroke !== undefined) {
            context.strokeStyle = this.style.stroke;
            context.lineWidth = this.style.strokeWidth;
            context.stroke();
        }
    }
}
