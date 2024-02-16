package org.firstinspires.ftc.teamcode.pipeline;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.Globals;

@Config
public class PropFarPipeline extends PropBasePipeline {

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        super.init(width, height, calibration);

        blueLeftX = 120;
        blueLeftY = 10;

        blueCenterX = 415;
        blueCenterY = 0;

        redLeftX = 238;
        redLeftY = 28;

        redCenterX = 550;
        redCenterY = 0;

        blueThreshold = 1.5;
        redThreshold = 1.5;

        redDeltaThreshold = 1.0;
        blueDeltaThreshold = 1.25;

    }
}
