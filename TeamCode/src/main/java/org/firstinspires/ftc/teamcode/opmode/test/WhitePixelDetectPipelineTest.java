package org.firstinspires.ftc.teamcode.opmode.test;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.pipeline.AlliancePosition;
import org.firstinspires.ftc.teamcode.pipeline.ContourDetectionPipeline;
import org.firstinspires.ftc.teamcode.pipeline.FieldPosition;
import org.firstinspires.ftc.teamcode.pipeline.PropBasePipeline;
import org.firstinspires.ftc.teamcode.pipeline.PropFarPipeline;
import org.firstinspires.ftc.teamcode.pipeline.PropNearPipeline;
import org.firstinspires.ftc.teamcode.pipeline.Side;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@Disabled
@TeleOp(group = "Test")
public class WhitePixelDetectPipelineTest extends LinearOpMode {
    private ContourDetectionPipeline pixelPipeline;
    private VisionPortal portal;
    public static int cameraWidth = 1920; // 640
    public static int cameraHeight = 1080; // 320

    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        pixelPipeline = new ContourDetectionPipeline();

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, Globals.FRONT_WEBCAM_NAME))
                .setCameraResolution(new Size(cameraWidth, cameraHeight))
                .addProcessor(pixelPipeline)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

        FtcDashboard.getInstance().startCameraStream(pixelPipeline,0);

        while (opModeInInit()) {

            telemetry.addData("pixel stack x", pixelPipeline.getRectX());
            telemetry.addData("pixel stack y", pixelPipeline.getRectY());
            telemetry.addData("pixel stack width", pixelPipeline.getRectWidth());
            telemetry.addData("# of contours", pixelPipeline.contourNumber);
            telemetry.addData("maxArea", pixelPipeline.maxArea);

            telemetry.update();
            idle();
        }

        portal.close();
        FtcDashboard.getInstance().onOpModePostStop(this);

    }
}