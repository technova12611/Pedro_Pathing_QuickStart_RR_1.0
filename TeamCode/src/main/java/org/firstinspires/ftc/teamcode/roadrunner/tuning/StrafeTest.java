package org.firstinspires.ftc.teamcode.roadrunner.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.TwoDeadWheelLocalizer;

@Config
public final class StrafeTest extends LinearOpMode {

    public static double distance = 24.0;

    @Override
    public void runOpMode() throws InterruptedException {
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, -Math.PI/2));
            telemetry.addData("perp encoder begin value: ", ((TwoDeadWheelLocalizer)drive.localizer).perp.getPositionAndVelocity().position);
            telemetry.update();

            waitForStart();

            Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(new Vector2d(distance,0), -Math.PI/2)
                        .build());

            telemetry.addData("perp encoder end value: ", ((TwoDeadWheelLocalizer)drive.localizer).perp.getPositionAndVelocity().position);

            telemetry.update();
        } else {
            throw new AssertionError();
        }
    }
}
