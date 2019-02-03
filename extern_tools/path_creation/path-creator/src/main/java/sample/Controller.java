package sample;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.fasterxml.jackson.databind.ObjectMapper;

import javafx.event.EventHandler;
import javafx.fxml.FXML;
import javafx.fxml.Initializable;
import javafx.geometry.Point2D;
import javafx.scene.canvas.Canvas;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.control.Button;
import javafx.scene.control.ChoiceBox;
import javafx.scene.control.TextField;
import javafx.scene.input.KeyCode;
import javafx.scene.input.KeyEvent;
import javafx.scene.input.MouseEvent;
import javafx.scene.layout.GridPane;
import javafx.scene.layout.Pane;
import javafx.scene.paint.Color;
import javafx.scene.shape.Circle;
import sample.serialize.InputType;
import sample.serialize.Options;
import sample.serialize.TrajectoryBuilderWrapper;

import java.io.*;
import java.net.URL;
import java.util.*;

public class Controller implements Initializable {

    @FXML
    public GridPane main;

    @FXML
    public TextField pathName;

    @FXML
    public TextField base64;

    @FXML
    public ChoiceBox choice;

    @FXML
    public Button button;

    @FXML
    private Canvas fieldCanvas;

    @FXML
    private Pane fieldPane;

    private GraphicsContext graphicsContext;

    private final double CANVAS_SIZE = 424;

    private DriveConstraints constraints = new DriveConstraints(24, 12, Math.PI / 2, Math.PI / 4);

    private ObjectMapper objectMapper = new ObjectMapper();

    private double mouseX;
    private double mouseY;

    private List<Circle> nodes;
    private List<MovementNode> movementNodes = new ArrayList<>();

    private boolean newPose = true;

    private int currentlySelectedIndex = -1;

    private TrajectoryBuilder trajectoryBuilder;
    @Override
    public void initialize(URL url, ResourceBundle resourceBundle) {
        graphicsContext = fieldCanvas.getGraphicsContext2D();
        Options[] optionValues = Options.values();
        String[] optionStrings = new String[optionValues.length];

        for (int i = 0; i < optionValues.length; i++) {
            optionStrings[i] = optionValues[i].toString();
        }

        choice.getItems().addAll(optionStrings);
        choice.setValue(optionStrings[0]);

        button.setOnMouseClicked(new EventHandler<MouseEvent>() {
            @Override
            public void handle(MouseEvent mouseEvent) {
                export();
            }
        });

        main.setOnMouseMoved(new EventHandler<MouseEvent>() {
            @Override
            public void handle(MouseEvent mouseEvent) {
                mouseX = mouseEvent.getX();
                mouseY = mouseEvent.getY();
                // System.out.println(mouseX + "," + mouseY);
            }
        });
        main.setOnKeyPressed(new EventHandler<KeyEvent>() {
            @Override
            public void handle(KeyEvent keyEvent) {
                if (keyEvent.getCode() == KeyCode.ESCAPE) {
                    if (currentlySelectedIndex != -1) {
                        deselect(nodes.get(currentlySelectedIndex));
                    }
                }

                if (keyEvent.getCode() == KeyCode.G) {
                    if (currentlySelectedIndex != -1) {
                        Circle location = nodes.get(currentlySelectedIndex);
                        moveToCursor(location, true, true);
                        update();
                    }
                }

                if (keyEvent.getCode() == KeyCode.X) {
                    if (currentlySelectedIndex != -1) {
                        Circle location = nodes.get(currentlySelectedIndex);
                        moveToCursor(location, true, false);
                        update();
                    }
                }

                if (keyEvent.getCode() == KeyCode.Y) {
                    if (currentlySelectedIndex != -1) {
                        Circle location = nodes.get(currentlySelectedIndex);
                        moveToCursor(location, false, true);
                        update();
                    }
                }

                if (keyEvent.getCode() == KeyCode.R) {
                    if (currentlySelectedIndex != -1) {
                        Circle location = nodes.get(currentlySelectedIndex);
                        rotateToCursor(location);
                        update();
                    }
                }

                if (keyEvent.getCode() == KeyCode.N) {
                    newPose = true;
                }
            }
        });

        fieldPane.setOnMouseClicked(new EventHandler<MouseEvent>() {
            @Override
            public void handle(MouseEvent mouseEvent) {
                if (newPose) {
                    Options o = (Options) (choice.getValue());
                    if (o.movementAction) {
                        Circle location = new Circle(4);
                        location.setStroke(Color.BLUE);
                        location.setFill(Color.BLUE);
                        location.relocate(mouseEvent.getX(), mouseEvent.getY());
                        fieldPane.getChildren().add(location);
                        location.setOnMouseClicked(new EventHandler<MouseEvent>() {
                            @Override
                            public void handle(MouseEvent mouseEvent) {
                                select(location);
                            }
                        });
                        nodes.add(location);
                        Pose2d pos = canvasToFieldSpace(mouseEvent.getX(), mouseEvent.getY());
                        MovementNode node = new MovementNode();
                        node.position = pos;
                        node.actions.add(o);
                        ArrayList<Object> tmp = new ArrayList<>();
                        tmp.add(pos);
                        node.values.add(tmp);
                        movementNodes.add(node);

                        update();

                        select(location);

                        newPose = false;
                    } else {
                        MovementNode n = movementNodes.get(currentlySelectedIndex);
                        n.actions.add(o);
                        ArrayList<Object> tmp = new ArrayList<>();
                        for (InputType type : o.types) {
                            tmp.add(InputType.getDefault(type));
                        }
                        n.values.add(tmp);
                    }
                }

            }
        });
    }

    private void select(Circle location) {
        if (currentlySelectedIndex != -1) {
            deselect(nodes.get(currentlySelectedIndex));
        }
        currentlySelectedIndex = nodes.indexOf(location);
        location.setFill(Color.RED);
    }

    private void deselect(Circle location) {
        currentlySelectedIndex = -1;
        location.setFill(Color.BLUE);
    }

    public Pose2d canvasToFieldSpace(double x, double y) {
        double fieldY = -(x - CANVAS_SIZE / 2) * (144 / CANVAS_SIZE);
        double fieldX = -(y - CANVAS_SIZE / 2) * (144 / CANVAS_SIZE);
        return new Pose2d(fieldX, fieldY, 0);
    }

    public Point2D fieldToCanvasSpace(Pose2d pose) {
        double canvasY = -(pose.getX() * (CANVAS_SIZE / 144)) + CANVAS_SIZE / 2;
        double canvasX = -(pose.getY() * (CANVAS_SIZE / 144)) + CANVAS_SIZE / 2;
        return new Point2D(canvasX, canvasY);
    }

    private void moveToCursor(Circle location, boolean x, boolean y) {
        if (x) {
            location.setLayoutX(fieldPane.sceneToLocal(mouseX, 0).getX());
        }
        if (y) {
            location.setLayoutY(fieldPane.sceneToLocal(0, mouseY).getY());
        }
    }

    private void rotateToCursor(Circle location) {
        Point2D localMouse = location.sceneToLocal(mouseX, mouseY);
        double rotation = -(Math.atan2(localMouse.getY() - location.getCenterY(), localMouse.getX() - location.getCenterX()) + Math.PI / 2);
        if (rotation <  -Math.PI) {
             rotation += 2 * Math.PI;
        }
        location.setRotate(rotation);
    }

    private void load() {

    }

    private void export() {
        try {
            TrajectoryBuilderWrapper wrapper = new TrajectoryBuilderWrapper(poses, options, constraints, pathName.getText());
            String json = objectMapper.writeValueAsString(wrapper);
            String encoded = Base64.getEncoder().encodeToString(json.getBytes());
            base64.setText(encoded);
            writeFile(pathName.getText() + ".trj", json);
        }
        catch (Exception e){
            System.err.println(e.toString());
        }
    }

    private void writeFile(String fileName, String contents) throws IOException{
        File f = new File("trajectories/" + fileName);
        BufferedWriter writer = new BufferedWriter(new FileWriter(f));
        writer.write(contents);
        writer.close();
    }

    private void update() {

        int i = 0;
        for (Circle location : nodes) {
            movementNodes.get(i).position = new Pose2d(canvasToFieldSpace(location.getLayoutX(), location.getLayoutY()).pos(), location.getRotate());
            i++;
        }
        i = 0;

        trajectoryBuilder = new TrajectoryBuilder(poses.get(0), constraints, 2500);

        if (options != null && options.size() > 0) {
            for (Options o : options) {
                switch(o) {
                    case lineTo:
                        trajectoryBuilder.lineTo(poses.get(i + 1).pos());
                        break;
                    case splineTo:
                        trajectoryBuilder.splineTo(poses.get(i + 1));
                        break;
                    case turnTo:
                        trajectoryBuilder.turnTo(poses.get(i + 1).getHeading());
                        break;
                    case reverse:
                        trajectoryBuilder.reverse();
                        break;
                    case strafeTo:
                        trajectoryBuilder.strafeTo(poses.get(i + 1).pos());
                        break;
                }
                i++;
            }
            Trajectory t = trajectoryBuilder.build();

            graphicsContext.clearRect(0,0,fieldCanvas.getWidth(), fieldCanvas.getHeight());

            double length = t.duration();

            double[] x = new double[100];
            double[] y = new double[100];
            for (int j = 0; j < 100; j++) {
                Pose2d pose = t.get(j * length / 100);
                Point2D canvasSpace = fieldToCanvasSpace(pose);
                x[j] = canvasSpace.getX();
                y[j] = canvasSpace.getY();
            }
            graphicsContext.strokePolyline(x, y, 100);
        }
    }
}
