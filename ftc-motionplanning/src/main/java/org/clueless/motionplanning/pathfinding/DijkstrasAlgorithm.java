package org.clueless.motionplanning.pathfinding;

import java.util.Comparator;
import java.util.HashMap;


// Implementation of Dijkstra's Algorithm, based on explanation / tutorial by RedBlobGames (https://www.redblobgames.com/pathfinding/a-star/introduction.html)
public class DijkstrasAlgorithm {

    static PriorityQueue<Node> frontier;
    static HashMap<Node, Node> cameFrom;
    static HashMap<Node, Double> costSoFar;
    public static HashMap<Node, Node> Dijkstras(Node start, Node goal) {
        frontier = new PriorityQueue<>();
        frontier.Enqueue(start, 0);

        cameFrom = new HashMap<>();
        cameFrom.put(start, null);

        costSoFar = new HashMap<>();
        costSoFar.put(start, 0d);

        while (!(frontier.getCount() == 0)) {
            Node current = frontier.Dequeue();

            if (current.equals(goal)) {
                return cameFrom;
            }

            for (Node next : current.getPassableNeighbors()) {
                double new_cost = costSoFar.get(current) + current.edges.get(next);
                if (!costSoFar.keySet().contains(next) || new_cost < costSoFar.get(next)) {
                    costSoFar.put(next, new_cost);
                    frontier.Enqueue(next, new_cost);
                    cameFrom.put(next, current);
                }
            }
        }

        return null;

    }
}
