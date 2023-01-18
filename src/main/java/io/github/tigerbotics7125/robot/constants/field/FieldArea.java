/*
 * Copyright (c) 2022 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.robot.constants.field;

import static io.github.tigerbotics7125.robot.constants.field.FieldConstants.Community.*;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import io.github.tigerbotics7125.tigerlib.util.MathUtil;
import java.util.ArrayList;
import java.util.List;

/**
 * A FieldArea is a taped zone on the playing field, this class represents ways of detecting them,
 * and the robot relative to them
 *
 * @author Jeffrey Morris | Tigerbotics 7125
 */
public enum FieldArea {
    // TODO: complete areas
    COMMUNITY(
            Pair.of(0.0, 0.0),
            Pair.of(0.0, 5.49),
            Pair.of(3.36, 5.49),
            Pair.of(3.36, 4.03),
            Pair.of(4.91, 4.03),
            Pair.of(4.91, 0.0)),
    LOADING_ZONE(),
    CHARGING_STATION(
            Pair.of(2.92, 1.51), Pair.of(2.92, 3.98), Pair.of(4.86, 1.51), Pair.of(4.86, 3.98));

    private final int numVerts;
    // list of x and y coordinates.
    private final double[] vertXs;
    private final double[] vertYs;

    /** @param verticies double array of size two, representing (x, y) coordinats. */
    @SafeVarargs
    private FieldArea(Pair<Double, Double>... verticies) {
        numVerts = verticies.length;
        vertXs = new double[numVerts];
        vertYs = new double[numVerts];
        // split (x, y) pair into both lists.
        for (int i = 0; i < numVerts; i++) {
            vertXs[i] = verticies[i].getFirst();
            vertYs[i] = verticies[i].getSecond();
        }
    }

    /** @return The X value of each vertex, flipped based on alliance. */
    public double[] getVertXs() {
        if (!(DriverStation.getAlliance() == Alliance.Red)) return vertXs;

        double[] ret = new double[numVerts];
        for (int i = 0; i < numVerts; i++) {
            ret[i] = FieldConstants.fieldLength - vertXs[i];
        }
        return ret;
    }

    /** @return A list of Pose2d objects representing the verticies of this area. */
    public List<Pose2d> getPoses() {
        List<Translation2d> poses = new ArrayList<>();
        for (int i = 0; i < numVerts; i++) {
            poses.add(new Translation2d(getVertXs()[i], vertYs[i]));
        }
        return poses.stream().map(pose -> new Pose2d(pose, new Rotation2d())).toList();
    }

    /**
     * Point Inclusion in Polygon Test
     * https://web.archive.org/web/20161108113341/https://www.ecse.rpi.edu/Homepages/wrf/Research/Short_Notes/pnpoly.html
     *
     * @return Whether the supplied (x, y) coordinate falls within this enclosing polygon.
     */
    public boolean pnpoly(double testX, double testY) {
        return MathUtil.pnpoly(numVerts, getVertXs(), vertYs, testX, testY);
    }

    /**
     * @param pose The pose to check.
     * @return Whether the supplied pose lies within this enclosing polygon.
     */
    public boolean contains(Pose2d pose) {
        return pnpoly(pose.getX(), pose.getY());
    }
}
