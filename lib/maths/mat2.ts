import Vec2 from "./vec2";

export class Mat2 {
    a: number;
    b: number;
    c: number;
    d: number;

    constructor(a: number, b: number, c: number, d: number) {
        this.a = a;
        this.b = b;
        this.c = c;
        this.d = d;
    }

    static zero(): Mat2 {
        // prettier-ignore
        return new Mat2(
            0, 0,
            0, 0
        );
    }

    static identity(): Mat2 {
        // prettier-ignore
        return new Mat2(
            1, 0,
            0, 1
        );
    }

    static clone(lhs: Mat2): Mat2 {
        // prettier-ignore
        return new Mat2(
            lhs.a, lhs.b,
            lhs.c, lhs.d
        );
    }

    static rotation(radians: number, sin: number = Math.sin(radians), cos: number = Math.cos(radians)): Mat2 {
        // prettier-ignore
        return new Mat2(
            cos, -sin,
            sin, cos
        );
    }

    static scale(scale: Vec2): Mat2 {
        // prettier-ignore
        return new Mat2(
            scale.x, 0,
            0, scale.y
        );
    }

    static mul(lhs: Mat2, rhs: Mat2): Mat2 {
        // prettier-ignore
        return new Mat2(
            lhs.a * rhs.a + lhs.b * rhs.c, lhs.a * rhs.b + lhs.b * rhs.d,
            lhs.c * rhs.a + lhs.d * rhs.c, lhs.c * rhs.b + lhs.d * rhs.d,
        );
    }

    static mulVec2(lhs: Mat2, rhs: Vec2): Vec2 {
        return new Vec2(lhs.a * rhs.x + lhs.b * rhs.y, lhs.c * rhs.x + lhs.d * rhs.y);
    }

    static mulScalar(lhs: Mat2, rhs: number): Mat2 {
        // prettier-ignore
        return new Mat2(
            lhs.a * rhs, lhs.b * rhs,
            lhs.c * rhs, lhs.d * rhs
        );
    }

    static add(lhs: Mat2, rhs: Mat2): Mat2 {
        // prettier-ignore
        return new Mat2(
            lhs.a + rhs.a, lhs.b + rhs.b,
            lhs.c + rhs.c, lhs.d + rhs.d
        );
    }

    static addScalar(lhs: Mat2, rhs: number): Mat2 {
        // prettier-ignore
        return new Mat2(
            lhs.a + rhs, lhs.b + rhs,
            lhs.c + rhs, lhs.d + rhs
        );
    }

    static sub(lhs: Mat2, rhs: Mat2): Mat2 {
        // prettier-ignore
        return new Mat2(
            lhs.a - rhs.a, lhs.b - rhs.b,
            lhs.c - rhs.c, lhs.d - rhs.d
        );
    }

    static subScalar(lhs: Mat2, rhs: number): Mat2 {
        // prettier-ignore
        return new Mat2(
            lhs.a - rhs, lhs.b - rhs,
            lhs.c - rhs, lhs.d - rhs
        );
    }

    static divScalar(lhs: Mat2, rhs: number): Mat2 {
        // prettier-ignore
        return new Mat2(
            lhs.a / rhs, lhs.b / rhs,
            lhs.c / rhs, lhs.d / rhs
        );
    }

    static reciprocal(lhs: Mat2): Mat2 {
        // prettier-ignore
        return new Mat2(
            1 / lhs.a, 1 / lhs.b,
            1 / lhs.c, 1 / lhs.d
        );
    }

    static transpose(lhs: Mat2): Mat2 {
        // prettier-ignore
        return new Mat2(
            lhs.a, lhs.c,
            lhs.b, lhs.d
        );
    }

    static determinant(lhs: Mat2): number {
        return lhs.a * lhs.d - lhs.b * lhs.c;
    }

    static inverse(lhs: Mat2): Mat2 {
        const determinant = Mat2.determinant(lhs);

        if (determinant === 0) throw new Error("Matrix is not invertible");

        // prettier-ignore
        return new Mat2(
            lhs.d / determinant, -lhs.b / determinant,
            -lhs.c / determinant, lhs.a / determinant
        );
    }

    mul(rhs: Mat2): this {
        const a = this.a * rhs.a + this.b * rhs.c;
        const b = this.a * rhs.b + this.b * rhs.d;
        const c = this.c * rhs.a + this.d * rhs.c;
        const d = this.c * rhs.b + this.d * rhs.d;
        this.a = a;
        this.b = b;
        this.c = c;
        this.d = d;

        return this;
    }

    mulScalar(rhs: number): this {
        this.a *= rhs;
        this.b *= rhs;
        this.c *= rhs;
        this.d *= rhs;

        return this;
    }

    add(rhs: Mat2): this {
        this.a += rhs.a;
        this.b += rhs.b;
        this.c += rhs.c;
        this.d += rhs.d;

        return this;
    }

    addScalar(rhs: number): this {
        this.a += rhs;
        this.b += rhs;
        this.c += rhs;
        this.d += rhs;

        return this;
    }

    sub(rhs: Mat2): this {
        this.a -= rhs.a;
        this.b -= rhs.b;
        this.c -= rhs.c;
        this.d -= rhs.d;

        return this;
    }

    subScalar(rhs: number): this {
        this.a -= rhs;
        this.b -= rhs;
        this.c -= rhs;
        this.d -= rhs;

        return this;
    }

    divScalar(rhs: number): this {
        this.a /= rhs;
        this.b /= rhs;
        this.c /= rhs;
        this.d /= rhs;

        return this;
    }

    reciprocal(): this {
        this.a = 1 / this.a;
        this.b = 1 / this.b;
        this.c = 1 / this.c;
        this.d = 1 / this.d;

        return this;
    }

    transpose(): this {
        [this.b, this.c] = [this.c, this.b];
        return this;
    }
}
