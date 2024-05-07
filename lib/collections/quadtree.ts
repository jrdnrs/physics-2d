import { AABB } from "../maths/aabb";
import { swapRemove } from "../util";

const MAX_DEPTH = 4;

interface Bounded {
    bounds: AABB;
}

class ItemLocation<T extends Bounded> {
    /** Node that full contains this item */
    readonly node: QuadTreeNode<T>;
    /** Index within the node's item list */
    readonly index: number;

    constructor(node: QuadTreeNode<T>, index: number) {
        this.node = node;
        this.index = index;
    }
}

export class QuadTreeNode<T extends Bounded> {
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

    /** Clears all items from this node and its children */
    clear() {
        this.items.length = 0;
        for (const child of this.children) {
            child?.clear();
        }
    }

    /** Recursively prunes empty nodes from the tree, and returns true if this node is prunable */
    prune(): boolean {
        let hasChildren = false;

        for (const [i, child] of this.children.entries()) {
            if (child === undefined) continue;

            const prunable = child.prune();
            if (prunable) {
                this.children[i] = undefined;
            } else {
                hasChildren = true;
            }
        }

        return this.items.length === 0 && !hasChildren;
    }

    /**
     * Inserts the given item into the tree, returning the location of the item in the tree.
     *
     * This does no check to ensure the given bounds are even relevant for this particular node,
     * so it's up to the caller to make sure it's valid!
     */
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

    /** Extends the given array with all items from this node and its children */
    getAll(output: T[]) {
        output.push(...this.items);
        for (const child of this.children) {
            child?.getAll(output);
        }
    }

    /**
     * Extends the given array with all items from this node and its children that intersect
     * the given bounds.
     *
     * This does no check to ensure the given bounds are even relevant for this particular node,
     * so it's up to the caller to make sure it's valid!
     */
    get(bounds: AABB, output: T[]) {
        for (const item of this.items) {
            if (item.bounds.intersects(bounds)) {
                output.push(item);
            }
        }

        for (const child of this.children) {
            if (child === undefined) continue;

            if (bounds.contains(child.bounds)) {
                child.getAll(output);
            } else if (bounds.intersects(child.bounds)) {
                child.get(bounds, output);
            }
        }
    }
}

export class QuadTree<T extends Bounded> {
    readonly root: QuadTreeNode<T>;
    itemLocations: Map<T, ItemLocation<T>> = new Map();

    constructor(bounds: AABB) {
        this.root = new QuadTreeNode(bounds);
        this.itemLocations = new Map();
    }

    /** Clears all items from the tree, but retains node structure */
    clear() {
        this.root.clear();
        this.itemLocations.clear();
    }

    /** Prunes empty nodes from the tree */
    prune() {
        this.root.prune();
    }

    /**
     * Inserts the given item into the tree and returns true, but returns false if the item
     * is out of bounds
     */
    insert(item: T): boolean {
        if (!this.root.bounds.intersects(item.bounds)) return false;

        const location = this.root.insert(item);
        this.itemLocations.set(item, location);

        return true;
    }

    /** Returns all items in the tree that intersect the given bounds */
    get(bounds: AABB): T[] {
        const result: T[] = [];
        this.root.get(bounds, result);
        return result;
    }

    /** Removes the given item from the tree and returns true if it was removed */
    remove(item: T): boolean {
        const location = this.itemLocations.get(item);
        if (location === undefined) return false;

        swapRemove(location.node.items, location.index);
        this.itemLocations.delete(item);

        // If the removed item wasn't at the end of the list, we need to update the corresponding
        // ItemLocation for the item that was incidentally swapped with the removed item
        if (location.node.items.length > location.index) {
            const swappedItem = location.node.items[location.index];
            this.itemLocations.set(swappedItem, location);
        }

        location.node.prune();

        return true;
    }

    /** Updates the position of the given item in the tree, but returns false if the item is out of bounds */
    update(item: T): boolean {
        this.remove(item);
        return this.insert(item);
    }
}
