/*
 * Copyright (c) 2022 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.robot.constants.field;

import static io.github.tigerbotics7125.robot.constants.field.FieldConstants6328.Community.*;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
            Pair.of(4.86, 3.98), Pair.of(2.92, 3.98), Pair.of(2.92, 1.51), Pair.of(4.86, 1.51));

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
    public double[] getVertXs(Alliance alliance) {
        if (!(alliance == Alliance.Red)) return vertXs;

        double[] ret = new double[numVerts];
        for (int i = 0; i < numVerts; i++) {
            ret[i] = FieldConstants6328.fieldLength - vertXs[i];
        }
        return ret;
    }

    /** @return A list of Pose2d objects representing the verticies of this area. */
    public List<Pose2d> getPoses() {
        List<Translation2d> poses = new ArrayList<>();
        for (int i = 0; i < numVerts; i++) {
            poses.add(new Translation2d(getVertXs(Alliance.Red)[i], vertYs[i]));
            poses.add(new Translation2d(getVertXs(Alliance.Blue)[i], vertYs[i]));
        }
        return poses.stream().map(pose -> new Pose2d(pose, new Rotation2d())).toList();
    }

    /**
     * @param pose The pose to check.
     * @return Whether the supplied pose lies within this enclosing polygon.
     */
    public boolean contains(Pose2d pose) {
        return contains(pose.getTranslation());
    }

    /**
     * @param pose The pose to check.
     * @return Whether the supplied pose lies within this enclosing polygon.
     */
    public boolean contains(Translation2d pose) {
        return MathUtil.pnpoly(numVerts, getVertXs(Alliance.Red), vertYs, pose.getX(), pose.getY())
                | MathUtil.pnpoly(
                        numVerts, getVertXs(Alliance.Blue), vertYs, pose.getX(), pose.getY());
    }
}
