export type StyleID = number;

export class Style {
    readonly fill?: string;
    readonly stroke?: string;
    readonly strokeWidth: number;

    constructor(fill?: string, stroke?: string, strokeWidth: number = 1) {
        this.fill = fill;
        this.stroke = stroke;
        this.strokeWidth = strokeWidth;
    }
}
