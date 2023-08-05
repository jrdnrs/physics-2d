import { Polygon } from "../../lib/maths/poly";
import Vec2 from "../../lib/maths/vec2";

export function SAT(poly1: Polygon, poly2: Polygon): boolean {
    for (let i = 0; i < 2; i++) {
        for (const seg of poly1.getSegments()) {
            // I'm not sure if we need to normalise this??
            const normal = new Vec2(-(seg.p2.y - seg.p1.y), seg.p2.x - seg.p1.x);

            let min1 = Number.MAX_VALUE;
            let max1 = -Number.MAX_VALUE;
            for (const point of poly1.points) {
                const q = point.dot(normal);
                min1 = Math.min(min1, q);
                max1 = Math.max(max1, q);
            }

            let min2 = Number.MAX_VALUE;
            let max2 = -Number.MAX_VALUE;
            for (const point of poly2.points) {
                const q = point.dot(normal);
                min2 = Math.min(min2, q);
                max2 = Math.max(max2, q);
            }

            if (max1 < min2 || min1 > max2) return false;
        }
        [poly1, poly2] = [poly2, poly1];
    }

    return true;
}
