package sample;

import com.acmerobotics.roadrunner.Pose2d;
import sample.serialize.Options;

import java.util.ArrayList;
import java.util.List;

public class MovementNode {
    public Pose2d position;
    public List<Options> actions;
    public List<List<Object>> values;

    public MovementNode() {
        actions = new ArrayList<>();
        values = new ArrayList<>();
    }
}
