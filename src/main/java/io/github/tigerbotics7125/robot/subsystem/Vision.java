/*
 * Copyright (c) 2022 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.robot.subsystem;

import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision {

    PhotonCamera mCam = new PhotonCamera("tagCam");

    public Vision() {
        mCam.setDriverMode(false);
    }

    public List<PhotonTrackedTarget> getTargets() {
        return getResult().getTargets();
    }

    public PhotonPipelineResult getResult() {
        return mCam.getLatestResult();
    }
}
