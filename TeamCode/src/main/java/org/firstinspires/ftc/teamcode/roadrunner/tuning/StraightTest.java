package org.firstinspires.ftc.teamcode.roadrunner.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.TwoDeadWheelLocalizer;

@Config
public final class StraightTest extends LinearOpMode {
    public static double distance = 36.0;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

            telemetry.addData("par encoder begin value: ", ((TwoDeadWheelLocalizer)drive.localizer).par.getPositionAndVelocity().position);
            telemetry.update();
            waitForStart();

            Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .lineToX(distance)
                        .build());


            telemetry.addData("par encoder end value: ", ((TwoDeadWheelLocalizer)drive.localizer).par.getPositionAndVelocity().position);
            telemetry.update();

        } else {
            throw new AssertionError();
        }
    }
}
