package org.clueless.motionplanning.pathfinding;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class Node {
    public boolean passable = true;
    public HashMap<Node, Double> edges = new HashMap<>();

    public SquareGraph.Location physicalLocation;
    public SquareGraph.Location positionInGraph;

    public List<Node> getPassableNeighbors() {
        List<Node> nodes = new ArrayList<>();
        for (Map.Entry<Node, Double> entry : edges.entrySet()) {
            if (entry.getKey().passable) {
                nodes.add(entry.getKey());
            }
        }

        return nodes;
    }
}
