package org.firstinspires.ftc.teamcode.pipeline;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.util.Log;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.utils.software.MovingArrayList;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

@Config
public class ContourDetectionPipeline2 implements VisionProcessor, CameraStreamSource {
    private final AtomicReference<Bitmap> lastFrame = new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

    public volatile boolean error = false;
    public volatile Exception debug;
    private final Mat mat = new Mat();
    private Rect maxRect = new Rect(500,1,1,1);
    public double maxArea = 0;

    private final Object sync = new Object();

    public int contourNumber = 0;

    public MovingArrayList rect_x_list;
    public MovingArrayList rect_width_list;
    public long loopTime = 0;
    public static int blueSubMatX = 900;
    public static int blueSubMatY = 100;
    public static int redSubMatX = 900;
    public static int redSubMatY = 100;
    public static int subMatWidth = 1000;
    public static int subMatHeight = 800;

    int rectX;
    int rectY;

    public ContourDetectionPipeline2() {
        rect_x_list = new MovingArrayList(10);
        rect_width_list = new MovingArrayList(10);
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Mat processFrame(Mat input, long captureTimeNanos) {
        long startTime = System.currentTimeMillis();
        Mat edges = new Mat();
        Mat subMat;
        try {
            rectX = Globals.COLOR == AlliancePosition.RED? redSubMatX : blueSubMatX;
            rectY = Globals.COLOR == AlliancePosition.RED? redSubMatY : blueSubMatY;
            input.copyTo(mat);
            Rect detectionRect = new Rect(rectX,rectY, subMatWidth, subMatHeight);
            subMat = mat.submat(detectionRect);

            // Process Image
            Imgproc.cvtColor(subMat, subMat, Imgproc.COLOR_BGR2GRAY);
            // GaussianBlur
            Imgproc.GaussianBlur(subMat, subMat, new Size(3.0, 3.0), 0.1);
            // Use Canny edge detection

            Imgproc.Canny(subMat, edges, 75, 150);

            // Find contours in the edge-detected image
            Mat hierarchy = new Mat();
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

            contourNumber = contours.size();

            // Lock this up to prevent errors when outside threads access the max rect property.
            synchronized (sync) {
                double tempMaxArea = 1000.0;
                Rect tempMaxRect = new Rect(100,1,10,10);
                // Loop Through Contours
                for (MatOfPoint contour : contours) {
                    Point[] contourArray = contour.toArray();
                    // Bound Rectangle if Contour is Large Enough
                    if (contourArray.length >= 15) {
                        MatOfPoint2f areaPoints = new MatOfPoint2f(contourArray);
                        Rect rect = Imgproc.boundingRect(contour);

                        if (  (rect.area() > tempMaxArea &&  rect.width > 100 && rect.width < 500 && rect.height < 500)
                                || contourNumber == 1)
                        {
                            tempMaxArea = rect.area();
                            tempMaxRect = rect;

                            rect_x_list.add(rect.x + rect.width/2.0);
                            rect_width_list.add(rect.width*1.0);
                        }

                        areaPoints.release();
                    }
                    contour.release();
                }

                maxArea = tempMaxArea;
                maxRect = tempMaxRect;

                if (contours.isEmpty()) {
                    maxRect = new Rect(500,50,10,10);
                }
            }
            // Draw Rectangles If Area Is At Least 500
            Rect newRect = new Rect(rectX + maxRect.x, rectY + maxRect.y, maxRect.width, maxRect.height);
            Imgproc.rectangle(input, newRect, new Scalar(255, 0, 0), 5);
            Imgproc.rectangle(input, detectionRect, new Scalar(0, 255, 0), 3);

            loopTime = System.currentTimeMillis() - startTime;
        }
        catch (Exception e) {
            debug = e;
            error = true;
            Log.e("ContourDetection_logger",debug.getLocalizedMessage());
        }
        return input;
    }
    /*
    Synchronize these operations as the user code could be incorrect otherwise, i.e a property is read
    while the same rectangle is being processed in the pipeline, leading to some values being not
    synced.
     */

    public int getRectHeight() {
        synchronized (sync) {
            return maxRect.height;
        }
    }
    public int getRectWidth() {
        synchronized (sync) {
            return maxRect.width;
        }
    }
    public int getRectX() {
        synchronized (sync) {
            return maxRect.x + rectX;
        }
    }
    public int getRectY() {
        synchronized (sync) {
            return maxRect.y + rectY;
        }
    }

    public int getMidRectX() {
        synchronized (sync) {
            return maxRect.x + rectX + ((int)maxRect.width/2);
        }
    }
    public int getMidRectY() {
        synchronized (sync) {
            return maxRect.y + rectY + ((int)maxRect.height/2);
        }
    }

    public double getRectArea() {
        synchronized (sync) {
            return maxRect.area();
        }
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
    }
}
