package org.firstinspires.ftc.teamcode.opmode.test;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pipeline.PropPipeline;
import org.firstinspires.ftc.teamcode.pipeline.Side;
import org.firstinspires.ftc.teamcode.robot.Globals;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
//@Disabled
@TeleOp(group = "Test")
public class VisionPipelineTest extends LinearOpMode {

    private PropPipeline propPipeline;
    private VisionPortal portal;

    public static Side side = Side.RIGHT;
    public static int cameraWidth = 1920; // 640
    public static int cameraHeight = 1080; // 320

    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        propPipeline = new PropPipeline();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, Globals.FRONT_WEBCAM_NAME))
                .setCameraResolution(new Size(cameraWidth, cameraHeight))
//                .setCamera(BuiltinCameraDirection.BACK)
                .addProcessor(propPipeline)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

        while (opModeInInit()) {
            side = propPipeline.getLocation();

            int SPIKE = side.ordinal();

            telemetry.addData("Spike Position:", side.toString() + " | " + SPIKE);
            if(propPipeline.center != null) {
                telemetry.addData("Center Area:", propPipeline.centerZoneArea);
                telemetry.addData("Left Area:", propPipeline.leftZoneArea);
                telemetry.addData("left color:", "%3.2f",propPipeline.leftColor);
                telemetry.addData("Center color:", "%3.2f",propPipeline.centerColor);
            }

            telemetry.update();
            idle();
        }

        portal.close();

    }
}