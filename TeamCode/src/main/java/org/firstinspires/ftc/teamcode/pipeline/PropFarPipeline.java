package org.firstinspires.ftc.teamcode.pipeline;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.utils.software.MovingArrayList;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.concurrent.atomic.AtomicReference;

@Config
public class PropFarPipeline extends PropBasePipeline {

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        super.init(width, height, calibration);

        blueLeftX = 375;
        blueLeftY = 80;

        blueCenterX = 950;
        blueCenterY = 0;

        redLeftX = 680;
        redLeftY = 80;

        redRightX = 1200;
        redRightY = 10;

        blueThreshold = 1.5;
        redThreshold = 1.75;

        redDeltaThreshold = 1.25;
        blueDeltaThreshold = 1.25;

    }

    public Side getLocation() {

        if(Globals.COLOR == AlliancePosition.RED) {
            if (super.getLocation() == Side.RIGHT) {
                return Side.CENTER;
            } else if (super.getLocation() == Side.CENTER) {
                return Side.RIGHT;
            } else {
                return Side.LEFT;
            }
        }
        else {
            return super.getLocation();
        }
    }

}
