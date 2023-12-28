package org.firstinspires.ftc.teamcode.roadrunner.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.messages.PoseMessage;

public final class SplineTest extends LinearOpMode {

    public static double distance = 24.0;

    public static int mode = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .splineTo(new Vector2d(distance, distance), 0)
//                       .splineTo(new Vector2d(60, 0), Math.PI)
                        .build());


        if(mode != 0) {
            Thread.sleep(2000);
            drive.updatePoseEstimate();

            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .setReversed(true)
                            .splineTo(new Vector2d(0, 0), Math.toRadians(180))
                            .build());
        }

        while (!isStopRequested()) {
            drive.updatePoseEstimate();
            telemetry.addData("Pose estimate: ", new PoseMessage(drive.pose));
            telemetry.addData("IMU heading: ", String.format("%3.2f",drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)));
            telemetry.update();
        }

    }
}
