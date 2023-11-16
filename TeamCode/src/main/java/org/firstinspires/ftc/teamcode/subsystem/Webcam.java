package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class Webcam {
//    SignalSleeveDetection sleeveDetection;
    OpenCvCamera camera;
    String webcamName = "Webcam 1";

    public Webcam(HardwareMap hardwareMap, Telemetry telemetry) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
 //       sleeveDetection = new SignalSleeveDetection(telemetry);
 //       camera.setPipeline(sleeveDetection);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                RobotLog.d("camera error with code - " + errorCode);
            }
        });
    }

//    public SignalSleeveDetection.ParkingPosition getPosition() {
//        return sleeveDetection.getPosition();
//    }

    public void stopStreaming() {
        try {
            camera.stopStreaming();
            camera.closeCameraDevice();
        } catch(Exception e) {
            // do nothing
            RobotLog.d("Stopping camera failed ...");
        }
    }
}
