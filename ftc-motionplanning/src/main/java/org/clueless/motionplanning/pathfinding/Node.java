package org.clueless.motionplanning.pathfinding;

import java.util.HashMap;

public class Node {
    boolean passable;
    HashMap<Node, Double> edges;
}
