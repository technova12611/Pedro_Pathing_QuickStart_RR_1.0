package org.firstinspires.ftc.teamcode.roadrunner.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.messages.PoseMessage;


public final class TurnTest extends LinearOpMode {
    public static double degree = 90.0;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0),false);

        waitForStart();

        Actions.runBlocking(
            drive.actionBuilder(drive.pose)
                    .turn(Math.toRadians(degree))
                    .build());

        telemetry.addData("end of turn pose estimate: ", new PoseMessage(drive.pose));
        telemetry.addData("end of turn IMU value: ", String.format("%3.2f",drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)));
        telemetry.update();

        while (!isStopRequested()) {
            drive.updatePoseEstimate();
            telemetry.addData("Current Pose estimate: ", new PoseMessage(drive.pose));
            telemetry.addData("Current IMU value: ", String.format("%3.2f",drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)));
            telemetry.update();
        }
    }
}
