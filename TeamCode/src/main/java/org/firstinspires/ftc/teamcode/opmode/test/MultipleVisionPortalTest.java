/* Copyright (c) 2023 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.opmode.test;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.pipeline.AlliancePosition;
import org.firstinspires.ftc.teamcode.pipeline.FieldPosition;
import org.firstinspires.ftc.teamcode.pipeline.PreloadDetectionPipeline;
import org.firstinspires.ftc.teamcode.pipeline.PropBasePipeline;
import org.firstinspires.ftc.teamcode.pipeline.PropNearPipeline;
import org.firstinspires.ftc.teamcode.pipeline.Side;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionPortal.CameraState;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/*
 * This OpMode illustrates the basics of AprilTag recognition and pose estimation, using
 * two webcams.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@TeleOp(name = "2 Vision Portal Test", group = "Concept")
public class MultipleVisionPortalTest extends LinearOpMode {

    /*
     * Variables used for switching cameras.
     */
    private WebcamName webcam1, webcam2;
    private boolean oldLeftBumper;
    private boolean oldRightBumper;

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    private PropBasePipeline propPipeline;

    private PreloadDetectionPipeline preloadPipeline;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal frontVisionPortal;
    private VisionPortal rearVisionPortal;

    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    boolean firstDetection = false;

    double detectionTime = 0.0;

    @Override
    public void runOpMode() {

        Globals.COLOR = AlliancePosition.BLUE;
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();
        propPipeline = new PropNearPipeline();
        preloadPipeline = new PreloadDetectionPipeline(aprilTag);
        preloadPipeline.setTargetAprilTagID(Side.CENTER);

        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        webcam2 = hardwareMap.get(WebcamName.class, "Webcam 2");

        int[] visionPortalViewIDs = VisionPortal.makeMultiPortalView(2,
                VisionPortal.MultiPortalLayout.VERTICAL);

        // Create the vision portal by using a builder.
        frontVisionPortal = new VisionPortal.Builder()
                .setCamera(webcam1)
                .setCameraResolution(new Size(1920, 1080))
                .addProcessor(propPipeline)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setLiveViewContainerId(visionPortalViewIDs[0])
                .setAutoStopLiveView(false)
                .build();

        while (!isStopRequested() && frontVisionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)
        {
            telemetry.addLine("Waiting for portal: " + visionPortalViewIDs[0]
                    +  " (front camera) to come online");
            telemetry.update();
        }

        rearVisionPortal = new VisionPortal.Builder()
                .setCamera(webcam2)
                .setCameraResolution(new Size(1920, 1080))
                .setLiveViewContainerId(visionPortalViewIDs[1])
                .addProcessors(aprilTag,preloadPipeline)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setAutoStopLiveView(false)
                .build();

        while (!isStopRequested() && rearVisionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)
        {
            telemetry.addLine("Waiting for portal:"  +  visionPortalViewIDs[1] + " (back camera) to come online");
            telemetry.update();
        }

        rearVisionPortal.setProcessorEnabled(aprilTag, false);
        rearVisionPortal.setProcessorEnabled(preloadPipeline, false);

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        boolean frontClosed = false;
        boolean backClosed = false;

        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        while (opModeInInit() && !isStopRequested()) {

            telemetryPipeline();

            // Push telemetry to the Driver Station.
            telemetry.update();

            // Save CPU resources; can resume streaming when needed.
            if (gamepad1.dpad_down) {
                frontVisionPortal.close();
                frontClosed = true;
            } else if (gamepad1.dpad_up) {
                rearVisionPortal.close();
                backClosed = true;
            }

            telemetry.addLine("Front Portal: " + frontClosed + " | Back Portal: " + backClosed);
            telemetry.addLine("Front Portal State: " + frontVisionPortal.getCameraState() + " | Back Portal State: " + rearVisionPortal.getCameraState());
            telemetry.addLine("Loop time (ms): " + String.format("%2.3f", timer.milliseconds()));
            timer.reset();
            doCameraSwitching();

            // Share the CPU.
            idle();
        }

        // Save more CPU resources when camera is no longer needed.
        frontVisionPortal.close();
        rearVisionPortal.close();


    }   // end runOpMode()

    /**
     * Add telemetry about AprilTag detections.
     */
    private void telemetryPipeline() {

        if(frontVisionPortal.getProcessorEnabled(propPipeline)) {

            Side side = propPipeline.getLocation();

            String sideStr = "Left";
            String centerStr = "Center";

            if (Globals.COLOR == AlliancePosition.RED) {
                if (Globals.FIELD == FieldPosition.NEAR) {
                    centerStr = "Left";
                    sideStr = "Right";
                } else {
                    sideStr = "Left";
                }
            } else {
                if (Globals.FIELD == FieldPosition.FAR) {
                    sideStr = "Right";
                }
            }

            telemetry.addData(centerStr + " color:", "Mean: %3.2f | Max: %3.2f ", propPipeline.meanCenterColor, propPipeline.maxCenterColor);
            telemetry.addData(sideStr + " color:", "Mean: %3.2f | Max: %3.2f ", propPipeline.meanSideColor, propPipeline.maxSideColor);
            telemetry.addData("Spike Position", side.toString());
            telemetry.addLine("\n");
        }

        if(rearVisionPortal.getProcessorEnabled(aprilTag)) {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            telemetry.addData("# AprilTags Detected", currentDetections.size());

            // Step through the list of detections and display info for each one.
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                    telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                } else {
                    telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                    telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                }
            }   // end for() loop

            // Add "key" information to telemetry
            telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
            telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
            telemetry.addLine("RBE = Range, Bearing & Elevation");
        }

        if(rearVisionPortal.getProcessorEnabled(preloadPipeline)) {

            if(!firstDetection && preloadPipeline.leftZoneAverage > 50 && preloadPipeline.rightZoneAverage > 50) {
                firstDetection = true;
                detectionTime = timer.milliseconds();
            }

            telemetry.addLine( "\nElapsed time: " + detectionTime);
            telemetry.addLine( "\nPRELOAD LEFT AVG: " + preloadPipeline.leftZoneAverage);
            telemetry.addLine( "\nPRELOAD RIGHT AVG: " + preloadPipeline.rightZoneAverage);
            telemetry.addLine( "\nPRELOAD ZONE: " + preloadPipeline.getPreloadedZone());
        }

        telemetry.update();

    }   // end method telemetryAprilTag()

    /**
     * Set the active camera according to input from the gamepad.
     */
    private void doCameraSwitching() {
        if (frontVisionPortal.getCameraState() == CameraState.STREAMING
                && rearVisionPortal.getCameraState() == CameraState.STREAMING
        ) {
            // If the left bumper is pressed, use Webcam 1.
            // If the right bumper is pressed, use Webcam 2.
            boolean newLeftBumper = gamepad1.left_bumper;
            boolean newRightBumper = gamepad1.right_bumper;
            if (newLeftBumper && !oldLeftBumper) {
                frontVisionPortal.setProcessorEnabled(propPipeline, true);
                rearVisionPortal.setProcessorEnabled(aprilTag, false);
                rearVisionPortal.setProcessorEnabled(preloadPipeline, false);
            } else if (newRightBumper && !oldRightBumper) {
                frontVisionPortal.setProcessorEnabled(propPipeline, false);
                rearVisionPortal.setProcessorEnabled(aprilTag, true);
                rearVisionPortal.setProcessorEnabled(preloadPipeline, true);
                timer.reset();
            }
            oldLeftBumper = newLeftBumper;
            oldRightBumper = newRightBumper;
        }
    }   // end method doCameraSwitching()

}   // end class
