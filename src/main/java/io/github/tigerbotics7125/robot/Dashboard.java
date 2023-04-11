/*
 * Copyright (c) 2023 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.robot;

import static io.github.tigerbotics7125.robot.constants.DashboardConstants.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.SendableCameraWrapper;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import io.github.tigerbotics7125.robot.constants.AutoPilotConstants.AutoPilotPoint;
import io.github.tigerbotics7125.robot.constants.VisionConstants;
import io.github.tigerbotics7125.robot.subsystem.SuperStructure;
import java.util.Map;
import java.util.stream.Stream;
import org.photonvision.PhotonCamera;

public class Dashboard {

    public enum OpZone {
        NODES,
        SUBSTATIONS
    }

    private static OpZone mCurrentZone = OpZone.NODES;

    // Match time variables.
    private static String mMatchTime = "N/A";
    private static double mSegmentStartTime = 0.0;

    // Nodes are a 2d array, size 9x3, with a selected node being a 1d, size 2, {x, y} index pair.
    private static boolean[][] mNodes = new boolean[9][3];
    private static int[] mSelectedNode = {4, 1};

    // Substations are a 1d array, size 2, with a selected substation being the index.
    private static boolean[] mSubstations = new boolean[2];
    private static int mSelectedSubstation = 0;

    // Photon driver cam object, instantiated to force into driver mode in case it isn't.
    private static PhotonCamera mDriverCam = new PhotonCamera(VisionConstants.DRIVER_CAM_NAME);

    // Shows the desired game piece drop off location.
    private static ShuffleboardLayout mNodeSelector;
    // Shows the desired game piece load location.
    private static ShuffleboardLayout mSubstationSelector;

    /** Initialize dashboard values. */
    public static void init() {
        highlight(mCurrentZone);

        // init and add node & substation selector widgets.
        initNodeSelector();
        initSubstationSelector();

        // Add match time widget.
        MAIN_TAB.addString("Match Time", () -> mMatchTime)
                .withWidget(BuiltInWidgets.kTextView)
                .withPosition(0, 0)
                .withSize(1, 1);

        // Dont let photon use computation on it
        mDriverCam.setDriverMode(true);
        // Add camera widgets.
        MAIN_TAB.add(
                        "Driver Cam",
                        SendableCameraWrapper.wrap(
                                "Driver Cam",
                                "http://10.71.25.12:1184/stream.mjpg",
                                "http://photonvision.local:1184/stream.mjpg"))
                .withPosition(0, 1)
                .withSize(4, 4)
                .withProperties(Map.of("Show crosshair", false, "Show controls", false));
        /*
                MAIN_TAB.add(
                                "Vision Cam",
                                SendableCameraWrapper.wrap(
                                        "Vision Cam",
                                        "http://10.71.25.11:1182/stream.mjpg",
                                        "http://photonvision.local:1182/stream.mjpg"))
                        .withPosition(4, 2)
                        .withSize(3, 2)
                        .withProperties(Map.of("Show crosshair", false, "Show controls", false));
        */
        // Add widget showing if autopilot is available.
        MAIN_TAB.addBoolean(
                        "Auto Pilot",
                        () -> AutoPilot.isViable(Robot.mDrivetrain.getPose(), getAutoPilotPoint()))
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withPosition(1, 0)
                .withSize(1, 1);
        MAIN_TAB.add(Robot.mAutoChooser)
                .withPosition(2, 0)
                .withSize(2, 1)
                .withWidget(BuiltInWidgets.kComboBoxChooser);
    }

    /** Call periodically to update dashboard values. */
    public static void update() {
        updateMatchTime();
    }

    /** @return The desired SuperStructure state based on the dashboard selector. */
    public static SuperStructure.State getSuperStrucState() {
        if (getZone() == OpZone.NODES) {
            int column = getSelectedNode()[0] % 3;
            int row = getSelectedNode()[1] % 3;
            // hybrid is all the same.
            if (row == 2) return SuperStructure.State.HYBRID;
            // if is cone
            if (column == 0 || column == 2) {
                if (row == 0) return SuperStructure.State.HIGH_CONE;
                if (row == 1) return SuperStructure.State.MID_CONE;
            } else {
                // cube
                if (row == 0) return SuperStructure.State.HIGH_CUBE;
                if (row == 1) return SuperStructure.State.MID_CUBE;
            }
        } else if (getZone() == OpZone.SUBSTATIONS) {
            return SuperStructure.State.SUBSTATION;
        }

        // Unknown desired state
        return SuperStructure.State.DISABLE;
    }

    /** @return The desired AutoPilotPoint as determined by the dashboard. */
    public static AutoPilotPoint getAutoPilotPoint() {
        if (getZone() == OpZone.NODES) {
            // Node selector.
            int columnIndex = getSelectedNode()[0];
            if (DriverStation.getAlliance() == Alliance.Red) {
                // Red alliance selection.
                return switch (columnIndex) {
                    case 0 -> AutoPilotPoint.FAR_FAR_CONE;
                    case 1 -> AutoPilotPoint.FAR_CUBE;
                    case 2 -> AutoPilotPoint.FAR_BARRIER_CONE;
                    case 3 -> AutoPilotPoint.COOP_FAR_CONE;
                    case 4 -> AutoPilotPoint.COOP_CUBE;
                    case 5 -> AutoPilotPoint.COOP_BARRIER_CONE;
                    case 6 -> AutoPilotPoint.BARRIER_FAR_CONE;
                    case 7 -> AutoPilotPoint.BARRIER_CUBE;
                    case 8 -> AutoPilotPoint.BARRIER_BARRIER_CONE;
                    default -> AutoPilotPoint.COOP_CUBE;
                };
            } else {
                // Blue alliance selection
                return switch (columnIndex) {
                    case 0 -> AutoPilotPoint.BARRIER_BARRIER_CONE;
                    case 1 -> AutoPilotPoint.BARRIER_CUBE;
                    case 2 -> AutoPilotPoint.BARRIER_FAR_CONE;
                    case 3 -> AutoPilotPoint.COOP_BARRIER_CONE;
                    case 4 -> AutoPilotPoint.COOP_CUBE;
                    case 5 -> AutoPilotPoint.COOP_FAR_CONE;
                    case 6 -> AutoPilotPoint.FAR_BARRIER_CONE;
                    case 7 -> AutoPilotPoint.FAR_CUBE;
                    case 8 -> AutoPilotPoint.FAR_FAR_CONE;
                    default -> AutoPilotPoint.COOP_CUBE;
                };
            }
        } else {
            // Substation selector.
            int substationIndex = getSelectedSubstation();
            if (DriverStation.getAlliance() == Alliance.Red) {
                // Red alliance selection.
                return switch (substationIndex) {
                    case 0 -> AutoPilotPoint.DOUBLE_SUBSTATION_BARRIER;
                    case 1 -> AutoPilotPoint.DOUBLE_SUBSTATION_FAR;
                    case 2 -> AutoPilotPoint.SINGLE_SUBSTATION;
                    default -> AutoPilotPoint.DOUBLE_SUBSTATION_FAR;
                };
            } else {
                // Blue alliance selection.
                return switch (substationIndex) {
                    case 0 -> AutoPilotPoint.DOUBLE_SUBSTATION_FAR;
                    case 1 -> AutoPilotPoint.DOUBLE_SUBSTATION_BARRIER;
                    case 2 -> AutoPilotPoint.SINGLE_SUBSTATION;
                    default -> AutoPilotPoint.DOUBLE_SUBSTATION_FAR;
                };
            }
        }
    }

    /** Initialize and fill the node selector */
    public static void initNodeSelector() {
        // Initialize layout.
        mNodeSelector =
                MAIN_TAB.getLayout("Node Selector", BuiltInLayouts.kGrid)
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

        // Iterate over every node to instantiate it.
        for (int column = 0; column < mNodes.length; column++) {
            for (int row = 0; row < mNodes[0].length; row++) {
                String trueColor, falseColor;
                trueColor = falseColor = new String();

                // if column is cube column
                if (column % 3 == 1) {
                    trueColor = SELECTED_CUBE_COLOR;
                    falseColor = UNSELECTED_CUBE_COLOR;
                } else {
                    trueColor = SELECTED_CONE_COLOR;
                    falseColor = UNSELECTED_CONE_COLOR;
                }

                // Set bottom row as hybrid color
                if (row % 3 == 2) {
                    trueColor = SELECTED_HYBRID_COLOR;
                    falseColor = UNSELECTED_HYBRID_COLOR;
                }

                String nodeName = "[" + column + "," + row + "]";
                // must use final variables in lamda >:(.
                final int x = column;
                final int y = row;
                // Add boolean box to node selector layout.
                mNodeSelector
                        .addBoolean(nodeName, () -> mNodes[x][y])
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

    /** Initialize and fill the substation selector. */
    public static void initSubstationSelector() {
        // Initialize layout.
        mSubstationSelector =
                MAIN_TAB.getLayout("Substation", BuiltInLayouts.kGrid)
                        .withPosition(7, 2)
                        .withSize(2, 1)
                        .withProperties(
                                Map.of(
                                        "Number of columns",
                                        2,
                                        "Number of rows",
                                        1,
                                        "Label position",
                                        "HIDDEN"));

        // Add both substations to the selector.
        mSubstationSelector
                .addBoolean("Left Substation", () -> mSubstations[0])
                .withPosition(0, 0)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withProperties(
                        Map.of(
                                "Color when true",
                                SELECTED_SUBSTATION_COLOR,
                                "Color when false",
                                UNSELECTED_SUBSTATION_COLOR));
        mSubstationSelector
                .addBoolean("Right Substation", () -> mSubstations[1])
                .withPosition(1, 0)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withProperties(
                        Map.of(
                                "Color when true",
                                SELECTED_SUBSTATION_COLOR,
                                "Color when false",
                                UNSELECTED_SUBSTATION_COLOR));
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

    /**
     * @return The currently selected Node, first value is column, second value is row, origin is
     *     top left.
     */
    public static int[] getSelectedNode() {
        return mSelectedNode;
    }

    public static int getSelectedSubstation() {
        return mSelectedSubstation;
    }

    private static void setSelectedNode(int[] node) {
        int x = node[0];
        int y = node[1];
        if (x < 0 || x > 8 || y < 0 || y > 2) {
            // Out of bounds
            return;
        }
        // update the selected node on dashboard
        mNodes[mSelectedNode[0]][mSelectedNode[1]] = false;
        mNodes[x][y] = true;
        mSelectedNode = node;
    }

    private static void setSelectedSubstation(int substation) {
        mSubstations[mSelectedSubstation] = false;
        mSubstations[substation] = true;
        mSelectedSubstation = substation;
    }

    private static void highlight(OpZone zone) {
        // highlight
        switch (zone) {
            case NODES -> mNodes[mSelectedNode[0]][mSelectedNode[1]] = true;
            case SUBSTATIONS -> mSubstations[mSelectedSubstation] = true;
        }

        // unhighlight
        for (OpZone w : Stream.of(OpZone.values()).filter(w -> !zone.equals(w)).toList()) {
            switch (w) {
                case NODES -> mNodes[mSelectedNode[0]][mSelectedNode[1]] = false;
                case SUBSTATIONS -> mSubstations[mSelectedSubstation] = false;
            }
        }

        // Set the current zone
        mCurrentZone = zone;
    }

    public static Command toggleZone() {
        return Commands.runOnce(
                        () -> {
                            int newZoneOrdinal =
                                    (mCurrentZone.ordinal() + 1) % OpZone.values().length;
                            OpZone newZone = OpZone.values()[newZoneOrdinal];
                            highlight(newZone);
                            mCurrentZone = newZone;
                        })
                .ignoringDisable(true);
    }

    public static OpZone getZone() {
        return mCurrentZone;
    }

    public static Command selectLeft() {
        return Commands.runOnce(
                        () -> {
                            int[] curr = getSelectedNode();
                            int leftX = (curr[0] - 1);
                            setSelectedNode(new int[] {leftX, curr[1]});
                        })
                .alongWith(selectLeftSubstation())
                .andThen(Commands.runOnce(() -> highlight(mCurrentZone)))
                .ignoringDisable(true);
    }

    public static Command selectUp() {
        return Commands.runOnce(
                        () -> {
                            int[] curr = getSelectedNode();
                            int upY = (curr[1] - 1);
                            setSelectedNode(new int[] {curr[0], upY});
                        })
                .andThen(Commands.runOnce(() -> highlight(mCurrentZone)))
                .ignoringDisable(true);
    }

    public static Command selectRight() {
        return Commands.runOnce(
                        () -> {
                            int[] curr = getSelectedNode();
                            int rightX = (curr[0] + 1);
                            setSelectedNode(new int[] {rightX, curr[1]});
                        })
                .alongWith(selectRightSubstation())
                .andThen(Commands.runOnce(() -> highlight(mCurrentZone)))
                .ignoringDisable(true);
    }

    public static Command selectDown() {
        return Commands.runOnce(
                        () -> {
                            int[] curr = getSelectedNode();
                            int downY = (curr[1] + 1);
                            setSelectedNode(new int[] {curr[0], downY});
                        })
                .andThen(Commands.runOnce(() -> highlight(mCurrentZone)))
                .ignoringDisable(true);
    }

    private static Command selectLeftSubstation() {
        return Commands.runOnce(
                () -> {
                    setSelectedSubstation(0);
                });
    }

    private static Command selectRightSubstation() {
        return Commands.runOnce(
                () -> {
                    setSelectedSubstation(1);
                });
    }
}
