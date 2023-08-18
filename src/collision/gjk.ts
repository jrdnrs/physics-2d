import Segment from "../../lib/maths/segment";
import { Shape } from "../../lib/maths/shape";
import Vec2 from "../../lib/maths/vec2";

/**
 * GJK (Gilbert-Johnson-Keerthi) algorithm for collision detection.
 *
 * This algorithm works by finding the Minkowski difference of the two polygons,
 * and then checking if the origin is contained within the Minkowski difference.
 *
 * It does so by iteratively checking intelligently chosen simplices, made of
 * support points of the Minkowski difference, that hone in on the origin.
 */
export function GJK(shapeA: Shape, shapeB: Shape): GjkResult {
    const simplex: MinkowskiDiff[] = [];

    // First direction points from poly1 to poly2
    let direction = Vec2.sub(shapeA.getCentre(), shapeB.getCentre());
    // ...Unless they are on top of each other, in which case we just use the x-axis
    if (direction.isZero()) direction.x = 1;
    simplex.push(new MinkowskiDiff(shapeA, shapeB, direction));

    // Second direction points towards the origin
    direction = Vec2.neg(simplex[0].result);

    while (true) {
        const point = new MinkowskiDiff(shapeA, shapeB, direction);
        // If the point is not past the origin, then the origin cannot be contained
        // in the Minkowski difference
        if (point.result.dot(direction) < 0) return GjkResult.noCollision();
        simplex.push(point);

        if (checkSimplex(simplex, direction)) return GjkResult.collision(simplex);
    }
}

/**
 * Returns true when the origin is contained in the simplex, otherwise it mutates
 * the direction vector to point towards the next support point.
 */
function checkSimplex(simplex: MinkowskiDiff[], direction: Vec2): boolean {
    switch (simplex.length) {
        case 2:
            return checkSimplexLine(simplex, direction);

        case 3:
            return checkSimplexTriangle(simplex, direction);

        default:
            // Should be unreachable
            throw new Error("Invalid simplex length");
    }
}

/**
 * This always mutates the direction vector to point towards the next support point,
 * and it will always return false.
 */
function checkSimplexLine(simplex: MinkowskiDiff[], direction: Vec2): boolean {
    const a = simplex[1].result;
    const b = simplex[0].result;

    const ab = Vec2.sub(b, a);
    const ao = Vec2.neg(a);

    // The normal is the vector perpendicular to the edge, pointing towards the origin
    let normal = Vec2.tripleCross(ab, ao, ab);
    // `tripleCross` can return a zero vector if the origin is on the edge, so we default
    // to the perpendicular vector
    if (normal.isZero()) normal = Vec2.perpendicular(ab);

    direction.set(normal);
    return false;
}

/**
 * Returns true when the origin is contained in the simplex, otherwise it mutates
 * the direction vector to point towards the next support point.
 */
function checkSimplexTriangle(simplex: MinkowskiDiff[], direction: Vec2): boolean {
    // This ordering is important as we can exclude searching certain regions due to
    // the way in which these points were found.
    // See https://youtu.be/ajv46BSqcK4?t=1287

    const a = simplex[2].result;
    const b = simplex[1].result;
    const c = simplex[0].result;

    const ab = Vec2.sub(b, a);
    const ac = Vec2.sub(c, a);
    const ao = Vec2.neg(a);

    // `abNormal`/`acNormal` point towards region parallel to the edge AB/AC, outside the simplex.
    //
    // If the origin is in one of these regions, it is not in the simplex.
    // This must be the case if the angle between `ao` and `abNormal`/`acNormal` is less
    // than 90 degrees (i.e. the dot product is positive)
    //
    // If the origin is in the region AB/AC, we should remove C/B from the simplex,
    // and continue searching for support points in the direction of the normal.

    const abNormal = Vec2.tripleCross(ac, ab, ab);
    if (abNormal.dot(ao) > 0) {
        // simplex.splice(0, 1);
        simplex[0] = simplex[1];
        simplex[1] = simplex[2];
        simplex.length = 2;

        direction.set(abNormal);
        return false;
    }

    const acNormal = Vec2.tripleCross(ab, ac, ac);
    if (acNormal.dot(ao) > 0) {
        // simplex.splice(1, 1);
        simplex[1] = simplex[2];
        simplex.length = 2;

        direction.set(acNormal);
        return false;
    }

    // If the origin is not in either of those regions, it must be in the simplex!
    return true;
}

/**
 * EPA (Expanding Polytope Algorithm) is used to find the collision normal and penetration depth
 * of two polygons that are known to be colliding.
 *
 * It works by iteratively expanding the simplex in the direction of the current closest edge
 * to the origin, until that edge is found to be on the boundary of the Minkowski difference, which
 * means it truly is the closest edge to the origin.
 */
export function EPA(shapeA: Shape, shapeB: Shape, simplex: MinkowskiDiff[]): EpaResult {
    for (let i = 0; i < 100; i++) {
        // Iterate over all edges of the simplex, and find the one closest to the origin,
        // and store the normal that points away from the origin

        let minIndexA = 0;
        let minIndexB = 0;
        let minNormal = Vec2.zero();
        let minDistance = Number.MAX_VALUE;

        for (let i = 0; i < simplex.length; i++) {
            const j = (i + 1) % simplex.length;

            const a = simplex[i].result;
            const b = simplex[j].result;

            const ab = Vec2.sub(b, a);
            const oa = a;

            let normal = Vec2.tripleCross(ab, oa, ab);
            if (normal.isZero()) normal = Vec2.perpendicular(ab);
            normal.normalise();

            // This can sometimes be negative due to floating point errors, so we clamp it
            const distance = Math.max(0, normal.dot(a));

            if (distance < minDistance) {
                minIndexA = i;
                minIndexB = j;
                minNormal = normal;
                minDistance = distance;
            }
        }

        // Check if the support point in the direction of the normal (of the closest simplex edge)
        // is part of the same edge by checking if the distance to the origin is the same (with epsilon).
        //
        // If it is, this means it is not only the closest edge of the simplex, but also the closest
        // edge of the Minkowski difference.

        // Projecting the normal works for either point on the edge to get the distance of the edge
        const point = new MinkowskiDiff(shapeA, shapeB, minNormal);
        const distance = minNormal.dot(point.result);

        if (Math.abs(distance - minDistance) < 1e-3) {
            const mtv = Vec2.mulScalar(minNormal, distance);

            let contactA;
            if (Vec2.sub(simplex[minIndexB].a, simplex[minIndexA].a).magnitudeSquared() < 1) {
                contactA = simplex[minIndexA].a;
            } else {
                // Project origin onto edge (using first support point), and compare length to edge length
                // to get interpolation factor
                const edge = Vec2.sub(simplex[minIndexB].result, simplex[minIndexA].result);
                const t = -simplex[minIndexA].result.dot(edge) / edge.magnitudeSquared();

                contactA = Vec2.add(simplex[minIndexA].a, edge.mulScalar(t));
            }

            const contactB = Vec2.sub(contactA, mtv);

            // const contactPoints = getContactPoints(
            //     new Segment(simplex[minIndexA].a.clone(), simplex[minIndexB].a.clone()),
            //     new Segment(simplex[minIndexA].b.clone(), simplex[minIndexB].b.clone()),
            //     minNormal
            // );

            return new EpaResult(minNormal, mtv, distance, contactA, contactB);
        }

        // If the point is not on the edge, we need to insert it into the simplex to expand
        // it, and continue searching for the closest edge
        simplex.splice(minIndexA + 1, 0, point);
    }

    throw new Error("EPA failed to converge");
}

export function getContactPoints(edgeA: Segment, edgeB: Segment, normal: Vec2): Vec2[] {
    const points: Vec2[] = [];

    let reference = edgeB;
    let incident = edgeA;

    // We now clip the incident edge twice. Once for the threshold that passes through
    // the reference edge's first point, and once for the threshold that passes through
    // the reference edge's second point.
    //
    // We also need to check after if the incident edge is completely clipped away,
    // in which case we can return an empty array, as there are no contact points.

    const refEdge = reference.edge();
    const clipDirection = new Vec2(-normal.y, normal.x);
    const check = refEdge.dot(clipDirection);
    if (check < 0) clipDirection.neg();

    incident.clip(reference.p1, clipDirection);
    if (incident.isNaN()) {
        return points;
    }

    incident.clip(reference.p2, clipDirection.neg());
    if (incident.isNaN()) {
        return points;
    }

    // Now we clip against the reference edge itself. However, we are actually going to
    // discard points instead of clipping them, as these are now the contact points.

    const D = normal.dot(
        reference.p1.magnitudeSquared() >= reference.p2.magnitudeSquared() ? reference.p1 : reference.p2
    );

    if (incident.p1.nearEquals(incident.p2, 1)) {
        const average = new Vec2(incident.p1.x + incident.p2.x, incident.p1.y + incident.p2.y).mulScalar(0.5);
        const d = normal.dot(average) - D;
        if (d >= 0) points.push(average);
    } else {
        const d1 = normal.dot(incident.p1) - D;
        const d2 = normal.dot(incident.p2) - D;
        if (d1 >= 0) points.push(incident.p1);
        if (d2 >= 0) points.push(incident.p2);
    }

    return points;
}

export class MinkowskiDiff {
    readonly a: Vec2;
    readonly b: Vec2;
    readonly result: Vec2;

    constructor(shapeA: Shape, shapeB: Shape, direction: Vec2) {
        this.a = shapeA.getFurthestPoint(direction);
        this.b = shapeB.getFurthestPoint(Vec2.neg(direction));
        this.result = Vec2.sub(this.a, this.b);
    }
}

export class EpaResult {
    readonly normal: Vec2;
    readonly tangent: Vec2;
    readonly mtv: Vec2;
    readonly depth: number;
    readonly worldContactA: Vec2;
    readonly worldContactB: Vec2;

    constructor(normal: Vec2, mtv: Vec2, depth: number, worldContactA: Vec2, worldContactB: Vec2) {
        this.normal = normal;
        this.tangent = Vec2.perpendicular(normal);
        this.mtv = mtv;
        this.depth = depth;
        this.worldContactA = worldContactA;
        this.worldContactB = worldContactB;
    }
}

export class GjkResult {
    readonly collision: boolean;
    readonly simplex: MinkowskiDiff[] | undefined;

    constructor(collision: boolean, simplex?: MinkowskiDiff[]) {
        this.collision = collision;
        this.simplex = simplex;
    }

    static noCollision(): GjkResult {
        return new GjkResult(false);
    }

    static collision(simplex: MinkowskiDiff[]): GjkResult {
        return new GjkResult(true, simplex);
    }
}
