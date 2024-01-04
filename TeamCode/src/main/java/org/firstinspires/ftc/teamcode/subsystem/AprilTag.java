package org.firstinspires.ftc.teamcode.subsystem;

import android.util.Log;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class AprilTag {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    private Telemetry telemetry;
    private HardwareMap hardwareMap;

    public static Double yaw;

    public AprilTag(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
    }

    public void initialize() {

        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 2"), aprilTag);

    }

    public void close() {
        visionPortal.close();
    }

    public void update() {

        if(visionPortal != null && aprilTag != null) {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            telemetry.addData("# AprilTags Detected", currentDetections.size());

            // Step through the list of detections and display info for each one.
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                    telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));

                    this.yaw = detection.ftcPose.yaw;

                }

            }   // end for() loop
        }
    }

    public Action updatePosition() {
        return new UpdatePositionAction(this.visionPortal,this.aprilTag);
    }

    public static class UpdatePositionAction implements Action {
        VisionPortal portal;
        AprilTagProcessor aprilTag;

        public UpdatePositionAction(VisionPortal portal, AprilTagProcessor aprilTag) {
            this.portal = portal;
            this.aprilTag = aprilTag;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            try {
                if (this.portal != null && this.aprilTag != null) {
                    //this.portal.setProcessorEnabled(aprilTag, true);

                    List<AprilTagDetection> currentDetections = this.aprilTag.getDetections();
                    packet.addLine("# AprilTags Detected: " + currentDetections.size());

                    Log.d("AprilTag_Localization", "# AprilTags Detected: " + currentDetections.size());

                    Vector2d tagPosition = null;
                    Integer tagId = null;
                    AprilTag.yaw = null;
                    // Step through the list of detections and display info for each one.
                    for (AprilTagDetection detection : currentDetections) {
                        if (detection.metadata != null) {
                            tagId = detection.id;

                            //
                            //  check this: https://ftc-docs.firstinspires.org/en/latest/apriltag/understanding_apriltag_detection_values/understanding-apriltag-detection-values.html
                            //
                            //  https://ftc-docs.firstinspires.org/en/latest/_images/figure2.jpg
                            //
                            //  X is left and right, Y is the up and down
                            // so use X for the RR Y coordinate, use Y for RR coordinate
                            //
                            tagPosition = new Vector2d(detection.ftcPose.y, detection.ftcPose.x);

                            String msg1 = String.format("==== (ID %d) %s", detection.id, detection.metadata.name);
                            String msg2 = String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z);
                            String msg3 = String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw);

                            Log.d("AprilTag_Localization", msg1);
                            Log.d("AprilTag_Localization", msg2);
                            Log.d("AprilTag_Localization", msg3);

                            AprilTag.yaw = detection.ftcPose.yaw;
                            break;
                        }
                    }

                    //this.portal.setProcessorEnabled(aprilTag, false);
                }
            } catch(Exception e) {
                Log.e("AprilTag_Localization", e.getMessage());
            }

            return false;
        }
    }

}
