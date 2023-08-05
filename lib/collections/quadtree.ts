import { AABB } from "../maths/aabb";

const MAX_DEPTH = 4;

interface Item {
    bounds: AABB;
}

class ItemLocation<T extends Item> {
    readonly node: QuadTreeNode<T>;
    readonly index: number;

    constructor(node: QuadTreeNode<T>, index: number) {
        this.node = node;
        this.index = index;
    }
}

export class QuadTreeNode<T extends Item> {
    readonly depth: number;
    readonly bounds: AABB;
    readonly childBounds: AABB[];
    children: (QuadTreeNode<T> | undefined)[];
    items: T[];

    constructor(bounds: AABB, depth = 0) {
        const halfWidth = (bounds.max.x - bounds.min.x) * 0.5;
        const halfHeight = (bounds.max.y - bounds.min.y) * 0.5;

        this.depth = depth;
        this.bounds = bounds;
        this.childBounds = [
            AABB.fromDimensions(bounds.min.x, bounds.min.y, halfWidth, halfHeight),
            AABB.fromDimensions(bounds.min.x + halfWidth, bounds.min.y, halfWidth, halfHeight),
            AABB.fromDimensions(bounds.min.x, bounds.min.y + halfHeight, halfWidth, halfHeight),
            AABB.fromDimensions(bounds.min.x + halfWidth, bounds.min.y + halfHeight, halfWidth, halfHeight),
        ];
        this.children = new Array(4);
        this.items = [];
    }

    clear() {
        this.items.length = 0;
        for (const child of this.children) {
            child?.clear();
        }
    }

    prune(): boolean {
        let hasChildren = false;
        for (let i = 0; i < 4; i++) {
            if (this.children[i] !== undefined) {
                const prunable = this.children[i]!.prune();
                hasChildren ||= !prunable;

                if (prunable) {
                    this.children[i] = undefined;
                }
            }
        }

        return this.items.length === 0 && !hasChildren;
    }

    // This does no check on the initial bounds, so it's up to the caller to make sure it's valid.
    insert(item: T): ItemLocation<T> {
        if (this.depth < MAX_DEPTH) {
            for (let i = 0; i < 4; i++) {
                if (this.childBounds[i].contains(item.bounds)) {
                    if (this.children[i] === undefined) {
                        this.children[i] = new QuadTreeNode(this.childBounds[i], this.depth + 1);
                    }
                    return this.children[i]!.insert(item);
                }
            }
        }

        this.items.push(item);

        return new ItemLocation(this, this.items.length - 1);
    }

    getAll(result: T[]) {
        result.push(...this.items);
        for (const child of this.children) {
            child?.getAll(result);
        }
    }

    // This does no check on the initial bounds, so it's up to the caller to make sure it's valid.
    get(bounds: AABB, result: T[]) {
        for (const item of this.items) {
            if (item.bounds.intersects(bounds)) {
                result.push(item);
            }
        }

        for (const child of this.children) {
            if (child !== undefined) {
                if (bounds.contains(child.bounds)) child.getAll(result);
                else if (bounds.intersects(child.bounds)) child.get(bounds, result);
            }
        }
    }
}

export class QuadTree<T extends Item> {
    readonly root: QuadTreeNode<T>;
    itemLocations: Map<T, ItemLocation<T>> = new Map();

    constructor(bounds: AABB) {
        this.root = new QuadTreeNode(bounds);
        this.itemLocations = new Map();
    }

    clear() {
        this.root.clear();
        this.itemLocations.clear();
    }

    prune() {
        this.root.prune();
    }

    insert(item: T): boolean {
        if (!this.root.bounds.contains(item.bounds)) return false;

        const location = this.root.insert(item);
        this.itemLocations.set(item, location);

        return true;
    }

    get(bounds: AABB): T[] {
        const result: T[] = [];
        this.root.get(bounds, result);
        return result;
    }

    remove(item: T): boolean {
        const location = this.itemLocations.get(item);
        if (location === undefined) return false;

        swapRemove(location.node.items, location.index);
        this.itemLocations.delete(item);

        // check if the removed item was the last item in the node
        if (location.node.items.length > location.index) {
            // update the location of the item that was swapped with the removed item
            const swappedItem = location.node.items[location.index];
            this.itemLocations.set(swappedItem, location);
        }

        location.node.prune();

        return true;
    }

    update(item: T): boolean {
        this.remove(item);
        return this.insert(item);
    }
}

/**
 * Swaps the item at the given index with the last item in the array and returns the last item.
 */
function swapRemove<T>(array: T[], index: number): T | undefined {
    [array[index], array[array.length - 1]] = [array[array.length - 1], array[index]];
    return array.pop();
}
