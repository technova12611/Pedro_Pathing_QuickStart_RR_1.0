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
    Scalar HOT_PINK = new Scalar(196, 23, 112);

//    public static Scalar scalarLowerHSV = new Scalar(0, 0, 200);
//    public static Scalar scalarUpperHSV = new Scalar(0, 0, 255);

    public static Scalar scalarLowerYCrCb = new Scalar(210.5, 126.0, 126.0);
    public static Scalar scalarUpperYCrCb = new Scalar(235.0, 128.0, 128.0);

    // Yellow, freight or ducks!
    //public static Scalar scalarLowerYCrCb = new Scalar(0.0, 100.0, 0.0);
    //public static Scalar scalarUpperYCrCb = new Scalar(255.0, 170.0, 120.0);

    // Green                                             Y      Cr     Cb
//     public static Scalar scalarLowerYCrCb = new Scalar(  0.0, 0.0, 0.0);
//     public static Scalar scalarUpperYCrCb = new Scalar(255.0, 120.0, 120.0);

    // Use this picture for you own color https://github.com/PinkToTheFuture/OpenCV_FreightFrenzy_2021-2022/blob/main/YCbCr.jpeg
    // Note that the Cr and Cb values range between 0-255. this means that the origin of the coordinate system is (128,128)

//    lower_red = np.array([0, 100, 100])
//    upper_red = np.array([10, 255, 255])
//
//            # Define the lower and upper boundaries of the blue color in HSV
//    lower_blue = np.array([110, 100, 100])
//    upper_blue = np.array([130, 255, 255])

    // Volatile because accessed by OpMode without sync
    public volatile boolean error = false;
    public volatile Exception debug;

    private final Mat mat = new Mat();
    private final Mat processed = new Mat();

    private Rect maxRect = new Rect(600,1,1,1);

    public double maxArea = 0;
    private boolean first = false;

    private final Object sync = new Object();

    public int contourNumber = 0;

    public MovingArrayList rect_x_list;
    public MovingArrayList rect_width_list;

    public long loopTime = 0;

    public static int blueSubMatX = 100;
    public static int blueSubMatY = 100;
    public static int redSubMatX = 800;
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
        try {
            rectX = Globals.COLOR == AlliancePosition.RED? redSubMatX : blueSubMatX;
            rectY = Globals.COLOR == AlliancePosition.RED? redSubMatY : blueSubMatY;
            input.copyTo(mat);
            Rect detectionRect = new Rect(rectX,rectY, subMatWidth, subMatHeight);
            Mat subMat = mat.submat(detectionRect);

            // Process Image
            Imgproc.cvtColor(subMat, subMat, Imgproc.COLOR_BGR2GRAY);
            // GaussianBlur
            Imgproc.GaussianBlur(subMat, subMat, new Size(5.0, 5.0), 0.00);
            // Use Canny edge detection
            Mat edges = new Mat();
            Imgproc.Canny(subMat, edges, 50, 150);

            // Find contours in the edge-detected image
            Mat hierarchy = new Mat();
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            contourNumber = contours.size();

            // Lock this up to prevent errors when outside threads access the max rect property.
            synchronized (sync) {
                double tempMaxArea = 0.0;
                Rect tempMaxRect = new Rect(100,100,10,10);
                // Loop Through Contours
                for (MatOfPoint contour : contours) {
                    Point[] contourArray = contour.toArray();
                    // Bound Rectangle if Contour is Large Enough
                    if (contourArray.length >= 15) {
                        MatOfPoint2f areaPoints = new MatOfPoint2f(contourArray);
                        Rect rect = Imgproc.boundingRect(contour);

                        if (  (rect.area() > tempMaxArea && rect.width < 500 && rect.height < 500)
                                || contourNumber == 1)
                        {
                            tempMaxArea = rect.area();
                            tempMaxRect = rect;
                            first = true;

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
                    maxRect = new Rect(100,100,10,10);
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
