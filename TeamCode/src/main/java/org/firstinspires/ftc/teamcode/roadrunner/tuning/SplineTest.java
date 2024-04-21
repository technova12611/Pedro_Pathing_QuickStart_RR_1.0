package org.firstinspires.ftc.teamcode.roadrunner.tuning;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.messages.PoseMessage;
import org.firstinspires.ftc.teamcode.utils.software.AutoActionScheduler;

import java.util.List;

public final class SplineTest extends LinearOpMode {

    public static double distance = 24.0;

    public static int mode = 1;

    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0), true);

        drive.startIMUThread(this);

        AutoActionScheduler scheduler = new AutoActionScheduler(this::update, hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        while (!isStarted() && !isStopRequested()) {

            List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
            for (LynxModule module : allHubs) {
                module.clearBulkCache();
            }

            telemetry.addLine("Ready to Start: SplineTest");

            drive.updatePoseEstimate();
//            Log.d("Spline_Logger: ", "Pose 0: " + new PoseMessage(drive.pose));

            telemetry.addData("Spline_Logger", "Loop: %3.3f", timer.milliseconds());
            timer.reset();

            telemetry.update();
        }

        int runCounter = 0;

        while (runCounter++ < 2) {
            scheduler.addAction(drive.actionBuilder(drive.pose)
                    .splineTo(new Vector2d(distance + 6, distance), 0)
//                       .splineTo(new Vector2d(60, 0), Math.PI)
                    .build());
            scheduler.run();

//        Actions.runBlocking(
//                drive.actionBuilderSlow(drive.pose)
//                        .splineTo(new Vector2d(distance+6, distance), 0)
////                       .splineTo(new Vector2d(60, 0), Math.PI)
//                        .build());


            if (mode != 0) {
                Thread.sleep(2000);
                drive.updatePoseEstimate();

                scheduler.addAction(drive.actionBuilder(drive.pose)
                        .setReversed(true)
                        .splineTo(new Vector2d(0, 0), Math.toRadians(180))
                        .build());

                scheduler.run();

                Thread.sleep(2000);
            }
        }

        while (!isStopRequested()) {
            drive.updatePoseEstimate();
            telemetry.addData("Pose estimate: ", new PoseMessage(drive.pose));
            //telemetry.addData("IMU heading: ", String.format("%3.2f",drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)));
            telemetry.update();
        }

    }

    public void update() {
    }
}
