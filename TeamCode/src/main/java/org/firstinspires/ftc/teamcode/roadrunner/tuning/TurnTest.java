package org.firstinspires.ftc.teamcode.roadrunner.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.TwoDeadWheelLocalizer;

@Config
public final class TurnTest extends LinearOpMode {
    public static double degree = 90.0;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

            waitForStart();

            Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .turn(Math.toRadians(degree))
                        .build());

            telemetry.addData("drive pose: ", "%.3f",Math.toDegrees(drive.pose.heading.log()));
            telemetry.update();

        } else {
            throw new AssertionError();
        }
    }
}
