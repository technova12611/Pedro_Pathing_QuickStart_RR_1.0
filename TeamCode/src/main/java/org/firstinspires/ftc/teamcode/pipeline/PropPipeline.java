package org.firstinspires.ftc.teamcode.pipeline;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.robot.Globals;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.concurrent.atomic.AtomicReference;

@Config
public class PropPipeline implements VisionProcessor, CameraStreamSource {
    private final AtomicReference<Bitmap> lastFrame = new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

    private Side location = Side.RIGHT;
    public MatOfKeyPoint keyPoints = new MatOfKeyPoint();

    public Rect leftZoneArea;
    public Rect centerZoneArea;

    private Mat finalMat = new Mat();

    public static int blueLeftX = 800;
    public static int blueLeftY = 550;

    public static int blueCenterX = 1175;
    public static int blueCenterY = 175;

    public static int redLeftX = 780;
    public static int redLeftY = 175;

    public static int redCenterX = 1250;
    public static int redCenterY = 100;

    public static int width = 200;
    public static int height = 200;

    public static double redLeftThreshold = 5.95;
    public static double redCenterThreshold = 5.15;
    public static double blueThreshold = 0.2;
    public static double threshold = 0;

    public double leftColor = 0.0;
    public double centerColor = 0.0;

    public Scalar left = new Scalar(0,0,0);
    public Scalar center = new Scalar(0,0,0);

    private final Scalar
            YELLOW  = new Scalar(255, 255, 0),
            CYAN    = new Scalar(0, 255, 255),
            MAGENTA = new Scalar(255, 0, 255),

            GREEN = new Scalar(0, 255, 0);

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        if (Globals.COLOR == Side.RED) {
            threshold = redCenterThreshold;
        } else {
            threshold = blueThreshold;
        }
    }

    @Override
    public Mat processFrame(Mat frame, long captureTimeNanos) {

        if (Globals.COLOR == Side.RED) {
            threshold = redCenterThreshold;
        } else {
            threshold = blueThreshold;
        }

        frame.copyTo(finalMat);
        Imgproc.GaussianBlur(finalMat, finalMat, new Size(5, 5), 0.0);

        leftZoneArea = new Rect(Globals.COLOR == Side.RED? redLeftX : blueLeftX, Globals.COLOR == Side.RED? redLeftY : blueLeftY, width, height);
        centerZoneArea = new Rect(Globals.COLOR == Side.RED?redCenterX:blueCenterX, Globals.COLOR == Side.RED?redCenterY:blueCenterY, width, height);

        Mat leftZone = finalMat.submat(leftZoneArea);
        Mat centerZone = finalMat.submat(centerZoneArea);

        left = Core.sumElems(leftZone);
        center = Core.sumElems(centerZone);

        leftColor = left.val[0] / 1000000.0;
        centerColor = center.val[0] / 1000000.0;

        if(Globals.COLOR == Side.BLUE){
            if (leftColor < threshold) {
                // left zone has it
                location = Side.LEFT;
                Imgproc.rectangle(frame, leftZoneArea, new Scalar(255, 255, 255));
            } else if (centerColor < threshold) {
                // center zone has it
                location = Side.CENTER;
                Imgproc.rectangle(frame, centerZoneArea, new Scalar(255, 255, 255));
            } else {
                // right zone has it
                location = Side.RIGHT;
                //Imgproc.rectangle(frame, leftZoneArea, new Scalar(255, 255, 255));
            }
        }else{
            if (centerColor > redCenterThreshold && leftColor < redLeftThreshold) {
                // center zone has it
                location = Side.CENTER;
                Imgproc.rectangle(frame, centerZoneArea, GREEN, 6);
            }
            else if (leftColor > redLeftThreshold && centerColor < redCenterThreshold) {
                // left zone has it
                location = Side.LEFT;
                Imgproc.rectangle(frame, leftZoneArea, GREEN,6);
            } else {
                // right zone has it
                location = Side.RIGHT;
            }
        }

        if(location.ordinal() != Side.LEFT.ordinal()) {
            Imgproc.rectangle(frame, leftZoneArea, YELLOW, 3);
        }

        if (location.ordinal() != Side.CENTER.ordinal()) {
            Imgproc.rectangle(frame, centerZoneArea, YELLOW, 3);
        }

        leftZone.release();
        centerZone.release();

        return frame;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    public Side getLocation() {
        return this.location;
    }

    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
    }
}
