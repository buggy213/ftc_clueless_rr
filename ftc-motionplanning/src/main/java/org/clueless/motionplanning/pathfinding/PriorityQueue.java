package org.clueless.motionplanning.pathfinding;

import java.util.HashMap;
import java.util.Map;


// Port of RedBlobGame's C# implementation of a priority queue (standard Java implementation uses int comparator)
public class PriorityQueue<T>
{
    // I'm using an unsorted array for this example, but ideally this
    // would be a binary heap. There's an open issue for adding a binary
    // heap to the standard C# library: https://github.com/dotnet/corefx/issues/574
    //
    // Until then, find a binary heap class:
    // * https://github.com/BlueRaja/High-Speed-Priority-Queue-for-C-Sharp
    // * http://visualstudiomagazine.com/articles/2012/11/01/priority-queues-with-c.aspx
    // * http://xfleury.github.io/graphsearch.html
    // * http://stackoverflow.com/questions/102398/priority-queue-in-net

    private HashMap<T, Double> elements = new HashMap<T, Double>();

    private int Count;

    public int getCount() {
        return elements.size();
    }

    public void Enqueue(T item, double priority)
    {
        elements.put(item, priority);
    }

    public T Dequeue()
    {
        Map.Entry<T, Double> bestEntry = null;

        for (Map.Entry<T, Double> entry: elements.entrySet()) {
            if (bestEntry == null || entry.getValue() < bestEntry.getValue()) {
                bestEntry = entry;
            }
        }

        T bestItem = bestEntry.getKey();
        elements.remove(bestItem);
        return bestItem;
    }
}
