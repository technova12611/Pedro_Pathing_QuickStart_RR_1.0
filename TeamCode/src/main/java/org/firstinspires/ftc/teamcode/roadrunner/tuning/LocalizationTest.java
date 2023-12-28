package org.firstinspires.ftc.teamcode.roadrunner.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.TwoDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.roadrunner.messages.PoseMessage;
import org.firstinspires.ftc.teamcode.roadrunner.ThreeDeadWheelLocalizer;

public class LocalizationTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        TwoDeadWheelLocalizer localizer = (TwoDeadWheelLocalizer) drive.localizer;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        int begin = localizer.par.getPositionAndVelocity().position;
        telemetry.addData("par0 encoder begin value: ", begin);
//        telemetry.addData("par1 encoder begin value: ", localizer.par1.getPositionAndVelocity().position);
        telemetry.addData("perp encoder begin value: ", localizer.perp.getPositionAndVelocity().position);

        waitForStart();

        while (opModeIsActive()) {
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));

            drive.updatePoseEstimate();

            telemetry.addData("x", new PoseMessage(drive.pose));

            int end = localizer.par.getPositionAndVelocity().position;
            telemetry.addData("par0 encoder end value: ", end);
            telemetry.addData("par0 encoder delta: ", (end-begin));
//            telemetry.addData("par1 encoder end value: ", localizer.par1.getPositionAndVelocity().position);
            telemetry.addData("perp encoder end value: ", localizer.perp.getPositionAndVelocity().position);
            telemetry.update();
        }

    }
}
