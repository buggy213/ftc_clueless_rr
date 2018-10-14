package org.clueless.motionplanning.pathfinding;

public class SquareGraph {
    class Location {
        public int x, y;

        public Location(int x, int y) {
            this.x = x;
            this.y = y;
        }

        @Override
        public String toString() {
            return x + ", " + y;
        }
    }

    public double magnitude(Location l) {
        return Math.sqrt(Math.pow(l.x, 2) + Math.pow(l.y, 2));
    }

    // Diagonal movement allowed as well
    Location[] movementAxes = {
            new Location(-1, 1),
            new Location(0, 1),
            new Location(1, 1),
            new Location(-1, 0),
            new Location(1, 0),
            new Location(-1, -1),
            new Location(0, -1),
            new Location(1, -1)
    };

    Node[][] nodes;

    public SquareGraph(double sideLength, double cellSize) {
        int cellCount = (int)Math.floor(sideLength / cellSize);
        nodes = new Node[cellCount][cellCount];
        // First pass to populate nodes with correct physical locations
        for (int i = 0; i < cellCount; i++) {
            for (int j = 0; j < cellCount; j++) {
                nodes[i][j] = new Node();
                nodes[i][j].physicalLocation = new Location((int)(i * cellSize), (int)(j * cellSize));
                nodes[i][j].positionInGraph = new Location(i, j);
            }
        }

        // Second pass to add connections between nodes
        for (int i = 0; i < cellCount; i++) {
            for (int j = 0; j < cellCount; j++) {
                for (Location location : movementAxes) {
                    Node neighbor;

                    // TODO fix this shit
                    try {
                        neighbor = nodes[i + location.x][j + location.y];
                    } catch (IndexOutOfBoundsException e) {
                        continue;
                    }
                    nodes[i][j].edges.put(neighbor, magnitude(location));
                }
            }
        }
    }

    public Node[][] getNodes() {
        return nodes;
    }
}
