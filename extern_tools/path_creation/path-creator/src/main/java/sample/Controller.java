package sample;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;

import javafx.collections.FXCollections;
import javafx.event.EventHandler;
import javafx.fxml.FXML;
import javafx.fxml.Initializable;
import javafx.geometry.Point2D;
import javafx.scene.canvas.Canvas;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.control.Button;
import javafx.scene.control.ChoiceBox;
import javafx.scene.control.TextField;
import javafx.scene.image.Image;
import javafx.scene.input.KeyCode;
import javafx.scene.input.KeyEvent;
import javafx.scene.input.MouseDragEvent;
import javafx.scene.input.MouseEvent;
import javafx.scene.layout.GridPane;
import javafx.scene.layout.Pane;
import javafx.scene.paint.Color;
import javafx.scene.shape.Arc;
import javafx.scene.shape.Circle;
import javafx.scene.shape.Polygon;

import java.io.*;
import java.net.URL;
import java.util.ArrayList;
import java.util.Base64;
import java.util.List;
import java.util.ResourceBundle;

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

    private DriveConstraints constraints = new DriveConstraints(15, 10, Math.PI / 2, Math.PI / 4);

    private ObjectMapper objectMapper = new ObjectMapper();

    private double mouseX;
    private double mouseY;

    private List<Pose2d> poses;
    private List<Options> options;
    private List<Circle> nodes;

    private boolean newPose = true;

    private int currentlySelectedIndex = -1;

    private TrajectoryBuilder trajectoryBuilder;
    @Override
    public void initialize(URL url, ResourceBundle resourceBundle) {
        graphicsContext = fieldCanvas.getGraphicsContext2D();

        choice.getItems().addAll("lineTo", "splineTo", "turnTo", "reverse", "strafeTo");
        choice.setValue("lineTo");

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
                System.out.println(mouseX + "," + mouseY);
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
                if (poses == null) {
                    poses = new ArrayList<>();
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
                    nodes = new ArrayList<>();
                    nodes.add(location);

                    poses.add(canvasToFieldSpace(mouseEvent.getX(), mouseEvent.getY()));

                    select(location);
                }
                else if (newPose){
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
                    poses.add(canvasToFieldSpace(mouseEvent.getX(), mouseEvent.getY()));

                    if (options == null) {
                        options = new ArrayList<>();
                    }

                    options.add(Options.valueOf(choice.getValue().toString()));
                    update();

                    select(location);

                    newPose = false;
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

    private void export() {
        try {
            TrajectoryBuilderWrapper wrapper = new TrajectoryBuilderWrapper(poses, options, constraints, pathName.getText());
            String json = objectMapper.writeValueAsString(wrapper);
            String encoded = Base64.getEncoder().encodeToString(json.getBytes());
            base64.setText(encoded);
            writeFile(pathName.getText() + ".trj", encoded);
        }
        catch (Exception e){
            System.err.println(e.toString());
        }
    }

    private void writeFile(String fileName, String contents) throws IOException{
        File f = new File("/trajectories" + fileName);
        BufferedWriter writer = new BufferedWriter(new FileWriter(f));
        writer.write(contents);
        writer.close();
    }

    private void update() {

        int i = 0;
        for (Circle location : nodes) {
            poses.set(i, new Pose2d(canvasToFieldSpace(location.getLayoutX(), location.getLayoutY()).pos(), location.getRotate()));
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
