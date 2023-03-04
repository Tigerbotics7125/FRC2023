/*
 * Copyright (c) 2022 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.robot.constants;

import static io.github.tigerbotics7125.robot.constants.AutoPilotConstants.AutoPilotBoundary.*;
import static io.github.tigerbotics7125.robot.constants.FieldConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import io.github.tigerbotics7125.lib.AllianceFlipUtil;
import io.github.tigerbotics7125.tigerlib.util.MathUtil;
import java.util.ArrayList;
import java.util.List;

/**
 * This class stores all measurement constants as the blue alliance and flips them upon retrieving
 * if the alliance given by driverstation is red.
 */
public class AutoPilotConstants {

    public static final double MAX_VELOCITY_MPS = 4;
    public static final double MAX_ACCELERATION_MPSPS = 2;

    public static final PIDController X_PID = new PIDController(5, 0, 0);
    public static final PIDController Y_PID = new PIDController(15, 0, 0);
    public static final PIDController THETA_PID = new PIDController(1, 0, 0);

    private static final Translation2d mBarrierConeColumn = new Translation2d(-0.351, -0.559);
    private static final Translation2d mCubeColumn = new Translation2d(-0.351, 0.0);
    private static final Translation2d mFarConeColumn = new Translation2d(-0.351, .559);
    private static final Translation2d mLeftDoubleSubstation = new Translation2d(0.0, -.722);
    private static final Translation2d mRightDoubleSubstation = new Translation2d(0.0, .722);
    private static final Translation2d mSingleSubstation =
            new Translation2d(1.954, -1.337); // From Double substation tag.

    public enum AutoPilotPoint {
        BARRIER_BARRIER_CONE(
                COMMUNITY, TAG6.pose.getTranslation().toTranslation2d().plus(mBarrierConeColumn)),
        BARRIER_CUBE(COMMUNITY, TAG6.pose.getTranslation().toTranslation2d().plus(mCubeColumn)),
        BARRIER_FAR_CONE(
                COMMUNITY, TAG6.pose.getTranslation().toTranslation2d().plus(mFarConeColumn)),
        COOP_BARRIER_CONE(
                COMMUNITY, TAG7.pose.getTranslation().toTranslation2d().plus(mBarrierConeColumn)),
        COOP_CUBE(COMMUNITY, TAG7.pose.getTranslation().toTranslation2d().plus(mCubeColumn)),
        COOP_FAR_CONE(COMMUNITY, TAG7.pose.getTranslation().toTranslation2d().plus(mFarConeColumn)),
        FAR_BARRIER_CONE(
                COMMUNITY, TAG8.pose.getTranslation().toTranslation2d().plus(mBarrierConeColumn)),
        FAR_CUBE(COMMUNITY, TAG8.pose.getTranslation().toTranslation2d().plus(mCubeColumn)),
        FAR_FAR_CONE(COMMUNITY, TAG8.pose.getTranslation().toTranslation2d().plus(mFarConeColumn)),
        DOUBLE_SUBSTATION_BARRIER(
                LOADING_ZONE,
                TAG4.pose.getTranslation().toTranslation2d().plus(mRightDoubleSubstation)),
        DOUBLE_SUBSTATION_FAR(
                LOADING_ZONE,
                TAG4.pose.getTranslation().toTranslation2d().plus(mLeftDoubleSubstation)),
        SINGLE_SUBSTATION(
                LOADING_ZONE, TAG4.pose.getTranslation().toTranslation2d().plus(mSingleSubstation));

        public final AutoPilotBoundary mBoundary;
        public final Translation2d mPose;

        /**
         * Create a point for the blue alliance, use {@link AllianceFlipUtil} to fix for red
         * alliance.
         *
         * @param boundary The bounding that autopilot is willing to drive within for this point.
         * @param pose The pose of the point.
         */
        private AutoPilotPoint(AutoPilotBoundary boundary, Translation2d pose) {
            mBoundary = boundary;
            mPose = pose;
        }

        /** @return The pose of this point. */
        public Translation2d getPose() {
            return AllianceFlipUtil.apply(mPose);
        }
    }

    public enum AutoPilotBoundary {
        COMMUNITY(
                new double[] {0.0, 0.0},
                new double[] {0.0, 5.49},
                new double[] {3.36, 5.49},
                new double[] {3.36, 5.49},
                new double[] {3.36, 4.03},
                new double[] {4.91, 4.03},
                new double[] {4.91, 0.0}),
        LOADING_ZONE(
                new double[] {13.18, 5.50},
                new double[] {13.18, 6.73},
                new double[] {9.83, 6.73},
                new double[] {9.83, 8.01},
                new double[] {16.54, 8.01},
                new double[] {16.54, 5.40});

        // List of double, {x, y} pairs.
        private final List<double[]> mVerticies;

        private AutoPilotBoundary(double[]... verticies) {
            mVerticies = List.of(verticies);
        }

        /**
         * @param pose The pose to check.
         * @return Wheter this boundary confines the provided pose.
         */
        public boolean confines(Translation2d pose) {
            int numVerts = mVerticies.size();
            double[] vertXs, vertYs;
            vertXs = vertYs = new double[numVerts];

            for (int i = 0; i < numVerts; i++) {
                double[] vertex = mVerticies.get(i);
                vertXs[i] = vertex[0];
                vertYs[i] = vertex[1];
            }

            double testX = pose.getX();
            double testY = pose.getY();

            return MathUtil.pnpoly(numVerts, vertXs, vertYs, testX, testY);
        }

        /**
         * @param pose The pose to check.
         * @return Wheter this boundary confines the provided pose.
         */
        public boolean confines(Pose2d pose) {
            return confines(pose.getTranslation());
        }

        public List<Pose2d> getPoses() {
            ArrayList<Pose2d> poses = new ArrayList<>();

            for (double[] xy : mVerticies) {
                poses.add(new Pose2d(xy[0], xy[1], new Rotation2d()));
            }

            return poses;
        }
    }
}
