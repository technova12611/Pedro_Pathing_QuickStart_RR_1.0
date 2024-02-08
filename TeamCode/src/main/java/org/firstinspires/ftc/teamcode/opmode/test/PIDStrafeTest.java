package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.TwoDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.roadrunner.messages.PoseMessage;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;
import org.firstinspires.ftc.teamcode.utils.software.DriveWithPID;

@Config
@Disabled
@TeleOp(group = "Test")
public final class PIDStrafeTest extends LinearOpMode {
    public static double distance = 3.0;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.PI));
        Outtake outtake = new Outtake(hardwareMap, false);
        outtake.initialize();

        TwoDeadWheelLocalizer localizer = (TwoDeadWheelLocalizer)drive.localizer;

        int perp_begin = localizer.perp.getPositionAndVelocity().position;
        int par_begin = localizer.par.getPositionAndVelocity().position;
        telemetry.addData("perp encoder begin value: ", perp_begin);
        telemetry.addData("par0 encoder begin value: ", par_begin);
        telemetry.update();

        DriveWithPID pidDrive = new DriveWithPID(drive, null, DriveWithPID.DriveDirection.STRAFE);

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                    pidDrive.setTargetPositionActionBlocking((int)(distance/MecanumDrive.PARAMS.inPerTick)),
                    new SleepAction(2.0),
                        pidDrive.setTargetPositionActionBlocking((int)(-distance/MecanumDrive.PARAMS.inPerTick))
                ));


        drive.updatePoseEstimate();
        telemetry.addData("end of strafe pose estimate: ", new PoseMessage(drive.pose));
        telemetry.update();

        while (!isStopRequested()) {
            drive.updatePoseEstimate();
            telemetry.addData("Drive Pose estimate: ", new PoseMessage(drive.pose));
            int perp_end = localizer.perp.getPositionAndVelocity().position;
            int par_end = localizer.par.getPositionAndVelocity().position;
            telemetry.addData("perp encoder end value: ", perp_end);
            telemetry.addData("perp encoder delta value: ", perp_end-perp_begin);
            telemetry.addData("perp *position* : ", String.format("%3.2f",Math.abs(perp_end-perp_begin)*MecanumDrive.PARAMS.inPerTick));
            telemetry.addData("par encoder end value: ", par_end);
            telemetry.addData("par encoder delta value: ", par_end-par_begin);
            telemetry.addData("par *position* : ", String.format("%3.2f",Math.abs(par_end-par_begin)*MecanumDrive.PARAMS.inPerTick));

            telemetry.update();
        }

    }
}
