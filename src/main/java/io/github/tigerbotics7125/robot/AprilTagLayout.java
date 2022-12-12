/*
 * Copyright (c) 2022 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

import java.util.Collection;
import java.util.HashMap;
import java.util.Map;

/** Class used to house info for an entire field of AprilTags. */
public class AprilTagLayout {

    /** Class used to house info for an individual AprilTag. */
    public static class AprilTag {
        private final int mID;
        private final Pose3d mPose;

        /**
         * @param id The fiducial id of this AprilTag.
         * @param pose The 3d position of this AprilTag.
         */
        public AprilTag(int id, Pose3d pose) {
            mID = id;
            mPose = pose;
        }
        /**
         * @param id The fiducial id of this AprilTag.
         * @param x The x coordinate. meters.
         * @param y The y coordinate. meters.
         * @param z The z coordinate. meters.
         * @param yaw The yaw of the tag. degrees.
         * @param pitch The pitch of the tag. degrees.
         */
        public AprilTag(int id, double x, double y, double z, double yaw, double pitch) {
            this(
                    id,
                    new Pose3d(
                            x,
                            y,
                            z,
                            new Rotation3d(
                                    0,
                                    Units.degreesToRadians(pitch),
                                    Units.degreesToRadians(yaw))));
        }

        /** @return The fiducial id of this tag. */
        public int getID() {
            return mID;
        }

        /** @return The 3d position of this tag. */
        public Pose3d getPose() {
            return mPose;
        }

        @Override
        public boolean equals(Object o) {
            if (o instanceof AprilTag tag)
                if (mID == tag.mID && mPose.equals(tag.mPose)) return true;
            return false;
        }
    }

    /** enum to define different tag layouts. */
    public enum Layout {
        TEST,
        RAPID_REACT,
        CHARGED_UP,
        ;
    }

    // contains the tag objects mapped to their fiducial id.
    private Map<Integer, AprilTag> mTags = new HashMap<>();

    /** @param layout The layout to generate tags from. */
    public AprilTagLayout(Layout layout) {
        switch (layout) {
            case TEST -> {
                mTags.put(0, new AprilTag(0, 7, 7, 0, 0, 0));
            }
            case RAPID_REACT -> {
                mTags.put(0, new AprilTag(0, -0.004, 7.579, 0.886, 0, 0));
                mTags.put(1, new AprilTag(1, 3.233, 5.487, 1.725, 0, 0));
                mTags.put(2, new AprilTag(2, 3.068, 5.331, 1.376, -90, 0));
                mTags.put(3, new AprilTag(3, 0.004, 5.059, 0.806, 0, 0));
                mTags.put(4, new AprilTag(4, 0.004, 3.512, 0.806, 0, 0));
                mTags.put(5, new AprilTag(5, 0.121, 1.718, 0.891, 46.25, 0));
                mTags.put(6, new AprilTag(6, 0.873, 0.941, 0.891, 46.25, 0));
                mTags.put(7, new AprilTag(7, 1.615, 0.157, 0.891, 46.25, 0));
                mTags.put(10, new AprilTag(10, 16.463, 0.651, 0.886, 180, 0));
                mTags.put(11, new AprilTag(11, 13.235, 2.743, 1.725, 180, 0));
                mTags.put(12, new AprilTag(12, 13.391, 2.90, 1.376, 90, 0));
                mTags.put(13, new AprilTag(13, 16.455, 3.176, 0.806, 180, 0));
                mTags.put(14, new AprilTag(14, 16.455, 4.717, 0.806, 180, 0));
                mTags.put(15, new AprilTag(15, 16.335, 6.515, 0.894, 223.8, 0));
                mTags.put(16, new AprilTag(16, 15.59, 7.293, 0.891, 223.8, 0));
                mTags.put(17, new AprilTag(17, 14.847, 8.069, 0.891, 223.8, 0));
                mTags.put(40, new AprilTag(40, 7.874, 4.913, 0.703, 114, 0));
                mTags.put(41, new AprilTag(41, 7.431, 3.759, 0.703, 204, 0));
                mTags.put(42, new AprilTag(42, 8.585, 3.316, 0.703, -66, 0));
                mTags.put(43, new AprilTag(43, 9.028, 4.47, 0.703, 24, 0));
                mTags.put(50, new AprilTag(50, 7.679, 4.326, 2.418, 159, 26.75));
                mTags.put(51, new AprilTag(51, 8.018, 3.564, 2.418, 339 - 90, 26.75));
                mTags.put(52, new AprilTag(52, 8.78, 3.903, 2.418, 249 + 90, 26.75));
                mTags.put(53, new AprilTag(53, 8.441, 4.665, 2.418, 69, 26.75));
            }
            case CHARGED_UP -> {
            }
        }
    }

    /** @return All tags housed in this layout. */
    public Collection<AprilTag> getTags() {
        return mTags.values();
    }

    /**
     * @param tagID The tag id to lookup.
     * @return the 3d position of that tag.
     */
    public Pose3d getTagPose(int tagID) {
        return mTags.get(tagID).mPose;
    }
}
