package org.firstinspires.ftc.teamcode.opmode.test;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pipeline.AlliancePosition;
import org.firstinspires.ftc.teamcode.pipeline.FieldPosition;
import org.firstinspires.ftc.teamcode.pipeline.PropBasePipeline;
import org.firstinspires.ftc.teamcode.pipeline.PropFarPipeline;
import org.firstinspires.ftc.teamcode.pipeline.PropNearPipeline;
import org.firstinspires.ftc.teamcode.pipeline.Side;
import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@Disabled
@TeleOp(group = "Test")
public class VisionPipelineTest extends LinearOpMode {
    private PropBasePipeline propPipeline;
    private VisionPortal portal;

    public static Side side = Side.RIGHT;
    public static int cameraWidth = 1920; // 640
    public static int cameraHeight = 1080; // 320

    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        if(Globals.FIELD == FieldPosition.NEAR) {
            propPipeline = new PropNearPipeline();
        } else {
            propPipeline = new PropFarPipeline();
        }

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, Globals.FRONT_WEBCAM_NAME))
                .setCameraResolution(new Size(cameraWidth, cameraHeight))
                .addProcessor(propPipeline)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

        FtcDashboard.getInstance().startCameraStream(propPipeline,0);

        Side prevLocation = Side.CENTER;
        int counter = 0;
        int error = 0;
        while (opModeInInit()) {
            side = propPipeline.getLocation();

            int SPIKE = side.ordinal();

            telemetry.addData("Spike Position:", side.toString() + " | " + SPIKE);
            if(propPipeline.center != null) {
                telemetry.addData("Center Area:", propPipeline.centerZoneArea);
                telemetry.addData("Center color:", "%3.2f",propPipeline.centerColor);
                if(Globals.COLOR == AlliancePosition.RED) {
                    telemetry.addData("Right Area:", propPipeline.sideZoneArea);
                    telemetry.addData("Right color:", "%3.2f",propPipeline.sideColor);
                } else {
                    telemetry.addData("Left Area:", propPipeline.sideZoneArea);
                    telemetry.addData("left color:", "%3.2f",propPipeline.sideColor);
                }


                telemetry.addData("Side scalar:", propPipeline.side);
                telemetry.addData("Center scalar:", propPipeline.center);

                telemetry.addData("Mean side color:", "%3.2f", propPipeline.meanSideColor);
                telemetry.addData("Mean center color:", "%3.2f",propPipeline.meanCenterColor);

                telemetry.addData("Color delta:", "%3.2f", (propPipeline.meanCenterColor - propPipeline.meanSideColor));

                telemetry.addData("Array size:", propPipeline.arraySize);
            }

            if(propPipeline.centerColor > 2*propPipeline.meanCenterColor) {
                telemetry.addData("Outliers: ", true);
                telemetry.addData("Outliers color: ", "%3.2f", propPipeline.centerColor);
                telemetry.addData("Outliers mean color: ", "%3.2f", propPipeline.meanCenterColor);
                telemetry.addData("Outliers position: ", side);

                telemetry.addData("Previous location: ", prevLocation);

                telemetry.addData("Outliers count: ", counter++);
            }

            if(side != prevLocation) {
                prevLocation = side;
            }

            if(prevLocation == Side.CENTER && side != Side.CENTER) {
                telemetry.addData("**** Error count: ", error++);
            }

            telemetry.addData("Elapsed time for full array: ", propPipeline.elapsedTime);

            telemetry.update();
            idle();
        }

        portal.close();
        FtcDashboard.getInstance().onOpModePostStop(this);

    }
}