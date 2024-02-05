package org.firstinspires.ftc.teamcode.pipeline;

import android.graphics.Canvas;
import android.util.Log;

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

public class PreloadDetectionPipeline implements VisionProcessor {

    private int targetAprilTagID = 0;

    private Side preloadedZone = Side.RIGHT;

    private AprilTagProcessor aprilTag;

    public static int leftZoneAverage = 0;
    public static int rightZoneAverage = 0;

    public static int numOfDetections = 0;

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
                if (detection.metadata != null) {
                    if (detection.id == targetAprilTagID) {
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

                        Rect leftInclusionZone = new Rect(tagCenterX - inclusionZoneWidth, tagCenterY - 420, inclusionZoneWidth, inclusionZoneHeight);
                        Rect rightInclusionZone = new Rect(tagCenterX, tagCenterY - 420, inclusionZoneWidth, inclusionZoneHeight);

                        Rect leftExclusionZone = new Rect(tagCenterX - (int) (inclusionZoneWidth * 0.55), tagCenterY - 340, exclusionZoneWidth, exclusionZoneHeight);
                        Rect rightExclusionZone = new Rect(tagCenterX + (int) (inclusionZoneWidth * 0.36), tagCenterY - 340, exclusionZoneWidth, exclusionZoneHeight);

                        Imgproc.rectangle(frame, leftInclusionZone, new Scalar(0, 255, 0), 5);
                        Imgproc.rectangle(frame, rightInclusionZone, new Scalar(0, 255, 0), 5);

                        Imgproc.rectangle(frame, leftExclusionZone, new Scalar(255, 0, 0), 3);
                        Imgproc.rectangle(frame, rightExclusionZone, new Scalar(255, 0, 0), 3);

                        leftZoneAverage = meanColor(frame, leftInclusionZone, leftExclusionZone);
                        rightZoneAverage = meanColor(frame, rightInclusionZone, rightExclusionZone);

                        if(leftZoneAverage > 50 && rightZoneAverage > 50) {
                            preloadedZone = (leftZoneAverage > (rightZoneAverage + 50)) ? Side.LEFT : Side.RIGHT;
                            Globals.PRELOAD = preloadedZone;
                        }

//                        Log.d("PreloadDetectionPipeline_logger", "leftZoneAverage: " + leftZoneAverage + " | rightZoneAverage: " + rightZoneAverage
//                        + " | zone:" + preloadedZone);

                        break;
                    }
                }
            }
        }

        return frame;
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
    }

    public int meanColor(Mat frame, Rect inclusionRect, Rect exclusionRect) {
        if (frame == null) {
            System.out.println("frame is bad");
            return 0;
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

        return count > 0 ? sum / count : 0;
    }


}