import org.clueless.motionplanning.math.Units;
import org.clueless.motionplanning.pathfinding.DijkstrasAlgorithm;
import org.clueless.motionplanning.pathfinding.Node;
import org.clueless.motionplanning.pathfinding.SquareGraph;
import org.junit.jupiter.api.Test;

import java.util.HashMap;
import java.util.Map;

public class PathfindingTest {
    @Test
    void TestSquareGraph() {
        SquareGraph graph = new SquareGraph(Units.ftToMm(18), Units.ftToMm(0.5));
        Node[][] nodes = graph.getNodes();

        for (int y = 3; y < 9; y++) {
            nodes[4][y].passable = false;
        }

        for (int y = 10; y < 18; y++) {
            nodes[13][y].passable = false;
        }

        for (int x = 4; x < 25; x++) {
            nodes[x][3].passable = false;
        }
        Node start = nodes[16][12];
        Node goal = nodes[1][1];
        HashMap<Node, Node> breadcrumbs = DijkstrasAlgorithm.Dijkstras(start, goal);

        while (goal != null) {
            System.out.println(goal.positionInGraph);
            goal = breadcrumbs.get(goal);

        }
    }
}
