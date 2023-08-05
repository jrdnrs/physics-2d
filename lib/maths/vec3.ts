import Vec2 from "./vec2";

export default class Vec3 {
    x: number;
    y: number;
    z: number;

    constructor(x: number, y: number, z: number) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    static zero(): Vec3 {
        return new Vec3(0, 0, 0);
    }

    intoVec2(): Vec2 {
        return new Vec2(this.x, this.y);
    }

    clone(): Vec3 {
        return new Vec3(this.x, this.y, this.z);
    }

    magnitude(): number {
        return Math.sqrt(this.x * this.x + this.y * this.y + this.z * this.z);
    }

    magnitudeSquared(): number {
        return this.x * this.x + this.y * this.y + this.z * this.z;
    }

    static normalise(v: Vec3): Vec3 {
        return v.clone().normalise();
    }

    normalise(): this {
        const m = this.magnitude();
        if (m > 0) {
            this.x /= m;
            this.y /= m;
            this.z /= m;
        }
        return this;
    }

    static dot(lhs: Vec3, rhs: Vec3): number {
        return lhs.dot(rhs);
    }

    dot(rhs: Vec3): number {
        return this.x * rhs.x + this.y * rhs.y + this.z * rhs.z;
    }

    static cross(lhs: Vec3, rhs: Vec3): Vec3 {
        return lhs.cross(rhs);
    }

    cross(rhs: Vec3): Vec3 {
        return new Vec3(
            this.y * rhs.z - this.z * rhs.y,
            this.z * rhs.x - this.x * rhs.z,
            this.x * rhs.y - this.y * rhs.x
        );
    }

    static add(lhs: Vec3, rhs: Vec3): Vec3 {
        return lhs.clone().add(rhs);
    }

    add(rhs: Vec3): this {
        this.x += rhs.x;
        this.y += rhs.y;
        this.z += rhs.z;
        return this;
    }

    static sub(lhs: Vec3, rhs: Vec3): Vec3 {
        return lhs.clone().sub(rhs);
    }

    sub(rhs: Vec3): this {
        this.x -= rhs.x;
        this.y -= rhs.y;
        this.z -= rhs.z;
        return this;
    }

    static mul(lhs: Vec3, rhs: Vec3): Vec3 {
        return lhs.clone().mul(rhs);
    }

    mul(rhs: Vec3): this {
        this.x *= rhs.x;
        this.y *= rhs.y;
        this.z *= rhs.z;
        return this;
    }

    static div(lhs: Vec3, rhs: Vec3): Vec3 {
        return lhs.clone().div(rhs);
    }

    div(rhs: Vec3): this {
        this.x /= rhs.x;
        this.y /= rhs.y;
        this.z /= rhs.z;
        return this;
    }


}
