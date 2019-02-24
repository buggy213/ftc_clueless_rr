package sample;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
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

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.net.URL;
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

    private double mouseX;
    private double mouseY;

    private boolean newPose = true;

    private int currentlySelectedIndex = -1;
    @Override
    public void initialize(URL url, ResourceBundle resourceBundle) {
        graphicsContext = fieldCanvas.getGraphicsContext2D();

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

    }

    private void writeFile(String fileName, String contents) throws IOException{
        File f = new File("trajectories/" + fileName);
        BufferedWriter writer = new BufferedWriter(new FileWriter(f));
        writer.write(contents);
        writer.close();
    }

    private void update() {

    }

    private void drawSampledTrajectory(Trajectory t) {
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
