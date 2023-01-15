/*
 * Copyright (c) 2022 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.robot.constants.field;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * This class represents a grid location based on its row and column.
 *
 * @author Jeffrey Morris | Tigerbotics 7125
 */
public class GridLocation {

    /*
     * Measurements for height only.
     */
    public enum Row {
        TOP(new Translation3d()),
        MID(new Translation3d()),
        BOTTOM(new Translation3d());

        private final Translation3d mPose;

        private Row(Translation3d pose) {
            mPose = pose;
        }

        public Translation2d getPose2d() {
            return mPose.toTranslation2d();
        }

        public Translation3d getPose3d() {
            return mPose;
        }
    }

    /** Measurements for x & y only. */
    public enum Column {
        LEFT_CONE(new Translation3d(0.0, -0.5588, 0.0)),
        MID_CUBE(new Translation3d(0.0, 0.0, 0.0)),
        RIGHT_CONE(new Translation3d(0.0, 0.5588, 0.0));

        private final Translation3d mPose;

        private Column(Translation3d pose) {
            mPose = pose;
        }

        public Translation2d getPose2d() {
            return getPose3d().toTranslation2d();
        }

        public Translation3d getPose3d() {
            if (!(DriverStation.getAlliance() == Alliance.Red) || this == MID_CUBE) return mPose;

            Translation3d pose = mPose;
            if (this == LEFT_CONE) pose = RIGHT_CONE.mPose;
            else if (this == RIGHT_CONE) pose = LEFT_CONE.mPose;

            return pose;
        }
    }

    public Row mRow;
    public Column mColumn;

    public GridLocation(Row row, Column column) {
        mRow = row;
        mColumn = column;
    }
}
