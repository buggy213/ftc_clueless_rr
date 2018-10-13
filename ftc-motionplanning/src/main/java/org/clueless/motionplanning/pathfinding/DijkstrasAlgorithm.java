package org.clueless.motionplanning.pathfinding;

import java.util.Comparator;
import java.util.HashMap;


// Implementation of Dijkstra's Algorithm, based on explanation / tutorial by RedBlobGames (https://www.redblobgames.com/pathfinding/a-star/introduction.html)
public class DijkstrasAlgorithm {

    PriorityQueue<Node> frontier;
    HashMap<Node, Node> cameFrom;
    HashMap<Node, Double> costSoFar;
    void Dijkstras(Node start, Node goal) {
        frontier = new PriorityQueue<>();
        frontier.Enqueue(start, 0);

        cameFrom = new HashMap<>();
        cameFrom.put(start, null);

        costSoFar = new HashMap<>();
        costSoFar.put(start, 0d);

        while (!(frontier.getCount() == 0)) {
            Node current = frontier.Dequeue();

            if (current.equals(goal)) {
                break;
            }

            for (Node next : current.edges.keySet()) {
                double new_cost = costSoFar.get(current) + current.edges.get(next);
                if (!costSoFar.keySet().contains(next) || new_cost < costSoFar.get(next)) {
                    costSoFar.put(next, new_cost);
                    frontier.Enqueue(next, new_cost);
                    cameFrom.put(next, current);
                }
            }
        }



    }
}
