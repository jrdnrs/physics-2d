/**
 * Swaps the item at the given index with the last item in the array and returns the last item.
 */
export function swapRemove<T>(array: T[], index: number): T | undefined {
    [array[index], array[array.length - 1]] = [array[array.length - 1], array[index]];
    return array.pop();
}
