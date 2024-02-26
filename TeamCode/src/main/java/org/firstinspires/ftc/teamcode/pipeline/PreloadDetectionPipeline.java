package org.firstinspires.ftc.teamcode.pipeline;

import android.graphics.Canvas;
import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.subsystem.AprilTag;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.List;

@Config
public class PreloadDetectionPipeline implements VisionProcessor {

    private int targetAprilTagID = 0;

    private Side preloadedZone = Side.RIGHT;

    private AprilTagProcessor aprilTag;

    public int leftZoneAverage = 0;
    public int rightZoneAverage = 0;

    public int numOfDetections = 0;

    public int inclusion_y_offset = 215;
    public int exclusion_y_offset = 175;

    public Vector2d aprilTagPose = new Vector2d(0.0,0.0);

    public PreloadDetectionPipeline(AprilTagProcessor aprilTag) {
        this.aprilTag = aprilTag;
        if(Globals.COLOR == AlliancePosition.BLUE) {
            preloadedZone = Side.LEFT;
        }
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        numOfDetections = currentDetections.size();
        if (currentDetections != null) {
            for (AprilTagDetection detection : currentDetections) {
                    // we are doing this check in case for wahtever reason the apriltag got switched
                    // we only need to know the position LEFT/CENTER/RIGHT tp corresponding numbers
                    if (detection.id == targetAprilTagID || detection.id == (targetAprilTagID-3) || detection.id == (targetAprilTagID+3)) {
                        if (detection.metadata != null) {
                            Log.d("PreloadDetectionPipeline_logger", "Detected_id: " + detection.id + " | targetAprilId: " + targetAprilTagID + " | Alliance Color: " + Globals.COLOR);
                            Log.d("PreloadDetectionPipeline_logger",
                                    String.format("==== (ID %d) %s", detection.id, detection.metadata.name));
                            Log.d("PreloadDetectionPipeline_logger",
                                    String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                            Log.d("PreloadDetectionPipeline_logger",
                                    String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                            Log.d("PreloadDetectionPipeline_logger",
                                    String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));

                        aprilTagPose = new Vector2d(detection.ftcPose.x,detection.ftcPose.y);

                        int leftX = Integer.MAX_VALUE;
                        int rightX = Integer.MIN_VALUE;
                        int topY = Integer.MIN_VALUE;
                        int bottomY = Integer.MAX_VALUE;

                        for (Point point : detection.corners) {
                            if (point.x < leftX) leftX = (int) point.x;
                            if (point.x > rightX) rightX = (int) point.x;
                            if (point.y > topY) topY = (int) point.y;
                            if (point.y < bottomY) bottomY = (int) point.y;
                        }

                        int tagCenterX = (int) detection.center.x;
                        int tagCenterY = (int) detection.center.y;

                        int tagWidth = rightX - leftX;
                        int tagHeight = topY - bottomY;

                        int inclusionZoneWidth = (int) (tagWidth * 1.5);
                        int inclusionZoneHeight = (int) (tagHeight * 1.5);

                        int exclusionZoneWidth = (int) (tagWidth * 0.43);
                        int exclusionZoneHeight = (int) (tagHeight * 0.43);

                        Rect leftInclusionZone = new Rect(tagCenterX - inclusionZoneWidth, tagCenterY - inclusion_y_offset, inclusionZoneWidth, inclusionZoneHeight);
                        Rect rightInclusionZone = new Rect(tagCenterX, tagCenterY - inclusion_y_offset, inclusionZoneWidth, inclusionZoneHeight);

                        Rect leftExclusionZone = new Rect(tagCenterX - (int) (inclusionZoneWidth * 0.55), tagCenterY - exclusion_y_offset, exclusionZoneWidth, exclusionZoneHeight);
                        Rect rightExclusionZone = new Rect(tagCenterX + (int) (inclusionZoneWidth * 0.36), tagCenterY - exclusion_y_offset, exclusionZoneWidth, exclusionZoneHeight);

                        Imgproc.rectangle(frame, leftInclusionZone, new Scalar(0, 255, 0), 5);
                        Imgproc.rectangle(frame, rightInclusionZone, new Scalar(0, 255, 0), 5);

                        Imgproc.rectangle(frame, leftExclusionZone, new Scalar(255, 0, 0), 3);
                        Imgproc.rectangle(frame, rightExclusionZone, new Scalar(255, 0, 0), 3);

                        leftZoneAverage = meanColor(frame, leftInclusionZone, leftExclusionZone);
                        rightZoneAverage = meanColor(frame, rightInclusionZone, rightExclusionZone);

                        if(leftZoneAverage > 20 && rightZoneAverage > 20) {
                            preloadedZone = (leftZoneAverage > (rightZoneAverage + 30)) ? Side.LEFT : Side.RIGHT;
                            Globals.PRELOAD = preloadedZone;
                        }

                        Log.d("PreloadDetectionPipeline_logger", "Tag Id " + targetAprilTagID + " | leftZoneAverage: " + leftZoneAverage + " | rightZoneAverage: " + rightZoneAverage
                        + " | zone:" + preloadedZone + " | aprilTag Pose: " + String.format("%3.2f", aprilTagPose.x));

                        break;
                    }
                }
            }
        }

        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    public Side getPreloadedZone() {
        return this.preloadedZone;
    }

    public int getTargetAprilTagID() {
        return this.targetAprilTagID;
    }

    public void setTargetAprilTagID(Side preloadLocation) {
        targetAprilTagID = 0;
        switch (preloadLocation) {
            case LEFT:
                targetAprilTagID = 1;
                break;
            case CENTER:
                targetAprilTagID = 2;
                break;
            case RIGHT:
                targetAprilTagID = 3;
                break;
            default:
                break;
        }

        if (Globals.COLOR == AlliancePosition.RED) targetAprilTagID += 3;

        leftZoneAverage = 1;
        rightZoneAverage = 1;
        if(Globals.COLOR == AlliancePosition.BLUE) {
            preloadedZone = Side.LEFT;
        }

    }

    public int meanColor(Mat frame, Rect inclusionRect, Rect exclusionRect) {
        if (frame == null) {
            System.out.println("frame is bad");
            return -1;
        }

        int sum = 0;
        int count = 0;
        for (int y = inclusionRect.y; y < inclusionRect.y + inclusionRect.height; y++) {
            for (int x = inclusionRect.x; x < inclusionRect.x + inclusionRect.width; x++) {
                if (x < 0 || x >= frame.cols() || y < 0 || y >= frame.rows()) {
                    continue;
                }

                if (x >= exclusionRect.x && x < exclusionRect.x + exclusionRect.width && y >= exclusionRect.y && y < exclusionRect.y + exclusionRect.height) {
                    continue;
                }

                double[] data = frame.get(y, x);
                if (data != null && data.length > 0) {
                    sum += data[0];
                    count++;
                }
            }
        }

        Log.d("PreloadDetectionPipeline_logger", "# of data points: " + count);
        return count > 0 ? sum / count : -2;
    }


}