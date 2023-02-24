/*
 * Copyright (c) 2023 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.robot;

import static io.github.tigerbotics7125.robot.constants.OIConstants.OI_TAB;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.*;
import io.github.tigerbotics7125.robot.constants.VisionConstants;
import io.github.tigerbotics7125.tigerlib.math.Tuple;
import java.util.Map;

import org.photonvision.PhotonCamera;

public class OperatorInterface {

    private static String mMatchTime = "N/A";
    private static double mSegmentStartTime = 0.0;

    private static boolean[][] mOperatorGrid = new boolean[9][3];
    private static Tuple<Integer, Integer> mSelectedNode = Tuple.of(4, 1);

    // private static UsbCamera mDriverCam = CameraServer.startAutomaticCapture();
    private static PhotonCamera mDriverCam = new PhotonCamera(VisionConstants.DRIVER_CAM_NAME);

    // Displayes the time remianing in the match
    private static final SuppliedValueWidget<String> mMatchTimeWidget =
            OI_TAB.addString("Match Time", () -> mMatchTime)
                    .withWidget(BuiltInWidgets.kTextView)
                    .withPosition(0, 0)
                    .withSize(2, 1);
    // Shows the desired game piece drop off location
    private static ShuffleboardLayout mOperatorSelector =
            OI_TAB.getLayout("Operator Control", BuiltInLayouts.kGrid)
                    .withPosition(4, 0)
                    .withSize(5, 2)
                    .withProperties(
                            Map.of(
                                    "Number of columns",
                                    9,
                                    "Number of rows",
                                    3,
                                    "Label position",
                                    "HIDDEN"));

    /** Initialize dashboard values. */
    public static void init() {
        initOperatorSelector();
        OI_TAB.addString(
                "selectedNode",
                () -> "x:" + mSelectedNode.getFirst() + " y:" + mSelectedNode.getSecond());

        mDriverCam.setDriverMode(true);

        // OI_TAB.add("Driver cam", SendableCameraWrapper.wrap(mDriverCam)).withPosition(0, 1).withSize(4, 3).withWidget(BuiltInWidgets.kCameraStream).withProperties(Map.of("Show crosshair", false, "Show controls", false));
        OI_TAB.add(new PowerDistribution(1, ModuleType.kRev));
    }

    /** Call periodically to update dashboard values. */
    public static void update() {
        updateMatchTime();
    }

    /** Update the match time variable */
    public static void updateMatchTime() {
        int maxSegmentTime = 0;

        if (RobotState.isDisabled()) {
            maxSegmentTime = 0;
            mMatchTime = "Disabled";
            return;
        } else if (RobotState.isEStopped()) {
            maxSegmentTime = 0;
            mMatchTime = "EStopped";
            return;
        } else if (RobotState.isTest()) {
            maxSegmentTime = 0;
            mMatchTime = "Test";
            return;
        } else if (RobotState.isAutonomous()) {
            maxSegmentTime = 15;
            mMatchTime = "Auto: %.1f";
        } else if (RobotState.isTeleop()) {
            maxSegmentTime = 135;
            mMatchTime = "Tele: %.1f";
        }

        double timeElapsed = Timer.getFPGATimestamp() - mSegmentStartTime;
        double timeRemaining = maxSegmentTime - timeElapsed;

        mMatchTime = String.format(mMatchTime, timeRemaining);
    }

    /** Reset the time that the current match segment starts on. */
    public static void startMatchSegment() {
        mSegmentStartTime = Timer.getFPGATimestamp();
    }

    /** Fills the operator selector layout with booleans to show the selection */
    public static void initOperatorSelector() {
        String unselectedCone = "#40361c";
        String selectedCone = "#FFD871";
        String unselectedCube = "#251b40";
        String selectedCube = "#976eff";
        String unselectedHybrid = "#403339";
        String selectedHybrid = "#CBA3B8";

        for (int column = 0; column < mOperatorGrid.length; column++) {
            for (int row = 0; row < mOperatorGrid[0].length; row++) {
                String gridName, columnName, rowName, trueColor, falseColor;
                trueColor = falseColor = "";

                gridName =
                        switch (column / 3) {
                            case 0 -> "L";
                            case 1 -> "Co";
                            case 2 -> "R";
                            default -> "N/A";
                        };
                columnName =
                        switch (column % 3) {
                            case 0 -> {
                                falseColor = unselectedCone;
                                trueColor = selectedCone;
                                yield "L-Cone";
                            }
                            case 1 -> {
                                falseColor = unselectedCube;
                                trueColor = selectedCube;
                                yield "M-Cube";
                            }
                            case 2 -> {
                                falseColor = unselectedCone;
                                trueColor = selectedCone;
                                yield "R-Cone";
                            }
                            default -> "N/A";
                        };
                rowName =
                        switch (row % 3) {
                            case 0 -> "L3";
                            case 1 -> "L2";
                            case 2 -> {
                                falseColor = unselectedHybrid;
                                trueColor = selectedHybrid;
                                yield "L1";
                            }
                            default -> "N/A";
                        };

                String nodeName = gridName + " " + rowName + ": " + columnName;
                final int x = column;
                final int y = row;
                mOperatorSelector
                        .addBoolean(nodeName, () -> mOperatorGrid[x][y])
                        .withPosition(column, row)
                        .withWidget(BuiltInWidgets.kBooleanBox)
                        .withProperties(
                                Map.of(
                                        "Color when true",
                                        trueColor,
                                        "Color when false",
                                        falseColor));
            }
        }
    }

    /**
     * @return The currently selected Node, first value is column, second value is row, origin is
     *     top left.
     */
    public static Tuple<Integer, Integer> getSelectedNode() {
        return mSelectedNode;
    }

    public static void setSelectedNode(Tuple<Integer, Integer> node) {
        int x = node.getFirst();
        int y = node.getSecond();
        if (x < 0 || x > 8 || y < 0 || y > 2) {
            // Out of bounds
            return;
        }
        // update the selected node on dashboard
        mOperatorGrid[mSelectedNode.getFirst()][mSelectedNode.getSecond()] = false;
        mOperatorGrid[x][y] = true;
        mSelectedNode = node;
    }

    public static void selectLeft() {
        var curr = getSelectedNode();
        var leftX = curr.getFirst() - 1;
        if (leftX < 0) return;
        else setSelectedNode(Tuple.of(leftX, curr.getSecond()));
    }

    public static void selectUp() {
        var curr = getSelectedNode();
        var upY = curr.getSecond() - 1;
        if (upY < 0) return;
        else setSelectedNode(Tuple.of(curr.getFirst(), upY));
    }

    public static void selectRight() {
        var curr = getSelectedNode();
        var rightX = curr.getFirst() + 1;
        if (rightX > 8) return;
        else setSelectedNode(Tuple.of(rightX, curr.getSecond()));
    }

    public static void selectDown() {
        var curr = getSelectedNode();
        var downY = curr.getSecond() + 1;
        if (downY > 2) return;
        else setSelectedNode(Tuple.of(curr.getFirst(), downY));
    }
}
