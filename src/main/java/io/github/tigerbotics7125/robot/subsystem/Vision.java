/*
 * Copyright (c) 2022 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.robot.subsystem;

import static io.github.tigerbotics7125.robot.Constants.NetworkTables.kTargetLockHeadingTopic;
import static io.github.tigerbotics7125.robot.Constants.Vision.*;

import io.github.tigerbotics7125.tigerlib.vision.SnakeEyes;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.ArrayList;
import java.util.List;
import org.photonvision.PhotonTargetSortMode;
import org.photonvision.SimVisionSystem;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SnakeEyes implements Subsystem {

    private final SimVisionSystem mSimVision = new SimVisionSystem(kCameraName, kCamDiagFOVDegrees, kCamToRobot,
	    kMaxLEDRangeMeters, kCamResWidth, kCamResHeight, kMinTargetArea);

    private final StringPublisher mTargetLockPub = kTargetLockHeadingTopic.publish();

    public Vision() {
	super(kCameraName, kCamToRobot.inverse());
    }

    public void updateCameraPose(Pose2d robotPose) {
	mSimVision.processFrame(robotPose);
    }

    /**
     * TODO: Add to Tigerlib.
     *
     * @param targetToFace
     * @param robotPose
     * @return The heading the robot should be at if it were to face the target
     *         head-on.
     */
    public Rotation2d getFaceTargetAngle(PhotonTrackedTarget targetToFace, Pose2d robotPose) {
	// get tags pose
	Pose2d tagPose = getTagPose(targetToFace.getFiducialId()).toPose2d();

	// get the triangle from robot to tag
	Transform2d robotToTag = new Transform2d(robotPose, tagPose);

	// determine the reference angle
	double angleToRotate = Math.atan2(robotToTag.getY(), robotToTag.getX());

	// the heading which will make the robot face the tag is its current
	// rotation
	// rotated by the reference.
	return robotPose.getRotation().rotateBy(new Rotation2d(angleToRotate));
    }

    /**
     * TODO: Add to Tigerlib.
     *
     * @param sortMode
     * @param ambiguityThreshold
     * @return The best tag
     */
    public PhotonTrackedTarget getBestTag(PhotonTargetSortMode sortMode, double ambiguityThreshold) {
	List<PhotonTrackedTarget> targets = getTargets();

	// remove tags which are too ambiguous
	targets = removeAmbiguousTags(targets, ambiguityThreshold);

	// sort targets in our predefined manner.
	targets.sort(kSortingMode.getComparator());

	// after sorting, the best target is first in the list.
	return targets.get(0);
    }

    /**
     * TODO: Add to Tigerlib.
     *
     * @return If any targets are seen.
     */
    public boolean hasTargets() {
	return !getTargets().isEmpty();
    }

    public List<PhotonTrackedTarget> removeAmbiguousTags(List<PhotonTrackedTarget> tags, double ambiguityThreshold) {
	List<PhotonTrackedTarget> ambiguousTags = new ArrayList<>();
	tags.forEach((t) -> {
	    if (t.getPoseAmbiguity() > ambiguityThreshold)
		ambiguousTags.add(t);
	});
	ambiguousTags.forEach(tags::remove);
	return tags;
    }

    /**
     * TODO: Add to Tigerlib.
     *
     * @return A list of estimated robot poses.
     */
    public List<Pose3d> getRobotPoseEstimates() {
	List<PhotonTrackedTarget> targets = getTargets();
	targets = removeAmbiguousTags(targets, kAmbiguityThreshold);

	List<Pose3d> robotPoses = new ArrayList<>();
	targets.forEach((target) -> {
	    robotPoses.add(this.getRobotPose(target));
	});

	return robotPoses;
    }

    @Override
    public void periodic() {
	// cache latest result
	this.getLatestResult();
    }

}
