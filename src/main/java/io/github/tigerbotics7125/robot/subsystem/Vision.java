/*
 * Copyright (c) 2022 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.robot.subsystem;

import static io.github.tigerbotics7125.robot.constants.VisionConstants.*;
import static io.github.tigerbotics7125.robot.constants.VisionConstants.Sim.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Subsystem;
import io.github.tigerbotics7125.robot.AprilTagLayout;
import io.github.tigerbotics7125.tigerlib.vision.SnakeEyes;
import org.photonvision.SimVisionSystem;
import org.photonvision.SimVisionTarget;

public class Vision extends SnakeEyes implements Subsystem {

    private final SimVisionSystem mSimVision =
            new SimVisionSystem(
                    kCameraName,
                    kCamDiagFOVDegrees,
                    kDefaultCamToRobot,
                    kMaxLEDRangeMeters,
                    kCamResWidth,
                    kCamResHeight,
                    kMinTargetArea);

    public Vision() {
        super(kCameraName, kDefaultCamToRobot.inverse());
        AprilTagLayout.mTags.forEach(this::addAprilTag);
        AprilTagLayout.mTags.forEach(
                (id, pose) -> mSimVision.addSimVisionTarget(new SimVisionTarget(pose, 8, 8, id)));
    }

    public void updateCameraPose(Pose2d robotPose) {
        mSimVision.processFrame(robotPose);
    }

    @Override
    public void periodic() {
        // cache latest result
        this.getLatestResult();
    }

    @Override
    public void simulationPeriodic() {}
}
