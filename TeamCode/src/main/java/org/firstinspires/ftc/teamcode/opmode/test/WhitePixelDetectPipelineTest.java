package org.firstinspires.ftc.teamcode.opmode.test;

import android.util.Log;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.pipeline.ContourDetectionPipeline2;
import org.firstinspires.ftc.teamcode.pipeline.PropBasePipeline;
import org.firstinspires.ftc.teamcode.pipeline.PropNearPipeline;
import org.firstinspires.ftc.teamcode.pipeline.Side;
import org.firstinspires.ftc.teamcode.utils.hardware.GamePadController;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@TeleOp(group = "Test")
public class WhitePixelDetectPipelineTest extends LinearOpMode {
    private ContourDetectionPipeline2 pixelPipeline;
    private PropBasePipeline propPipeline;
    private VisionPortal portal;
    public static int cameraWidth = 1920; // 640
    public static int cameraHeight = 1080; // 320

    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        pixelPipeline = new ContourDetectionPipeline2();
        propPipeline = new PropNearPipeline();

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, Globals.FRONT_WEBCAM_NAME))
                .setCameraResolution(new Size(cameraWidth, cameraHeight))
                .addProcessor(propPipeline)
                .addProcessor(pixelPipeline)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

        long startTime = System.currentTimeMillis();
        while(!portal.getCameraState().equals(VisionPortal.CameraState.STREAMING)) {
            telemetry.addLine(" Please wait, Webcam is streaming ... ");
            telemetry.update();
            idle();
        }

        Log.d("Pipeline_logger", "Webcam init time (ms): " + (System.currentTimeMillis() - startTime));

        portal.setProcessorEnabled(pixelPipeline, true);
        portal.setProcessorEnabled(propPipeline, false);

        Side side = Side.RIGHT;
        GamePadController g1 = new GamePadController(gamepad1);
        long start_time = System.currentTimeMillis();

        while (opModeInInit()) {
            if(portal.getProcessorEnabled(propPipeline)) {
                telemetry.addData(" color:", "Mean: %3.2f | Max: %3.2f ", propPipeline.meanCenterColor, propPipeline.maxCenterColor);
                telemetry.addData(" color:", "Mean: %3.2f | Max: %3.2f ", propPipeline.meanSideColor, propPipeline.maxSideColor);
                telemetry.addData("Spike Position", side.toString());
                telemetry.addData("Loop time Opmode (ms): ", System.currentTimeMillis() - start_time);
                telemetry.addLine("\n");
            }

            if(portal.getProcessorEnabled(pixelPipeline)) {
                telemetry.addData("pixel stack mid x", pixelPipeline.getRectX());
                telemetry.addData("pixel stack mid y", pixelPipeline.getRectY());
                telemetry.addData("pixel stack width", pixelPipeline.getRectWidth());
                telemetry.addData("pixel stack height", pixelPipeline.getRectHeight());
                telemetry.addData("# of contours", pixelPipeline.contourNumber);
                telemetry.addData("maxArea", pixelPipeline.getRectArea());
                telemetry.addData("pixel stack mean x", pixelPipeline.rect_x_list.getMean());
                telemetry.addData("pixel stack mean width", pixelPipeline.rect_width_list.getMean());
                telemetry.addData("Loop time Pipeline (ms)", pixelPipeline.loopTime);
                telemetry.addData("Loop time Opmode (ms): ", System.currentTimeMillis() - start_time);
            }

            if(g1.aOnce()) {
                portal.setProcessorEnabled(propPipeline, true);
                portal.setProcessorEnabled(pixelPipeline, false);
            }

            if(g1.bOnce()) {
                portal.setProcessorEnabled(propPipeline, false);
                portal.setProcessorEnabled(pixelPipeline, true);
            }

            telemetry.update();

            g1.update();

            start_time = System.currentTimeMillis();

            idle();
        }

        portal.setProcessorEnabled(propPipeline, false);
        portal.setProcessorEnabled(pixelPipeline, true);

        portal.close();
    }
}