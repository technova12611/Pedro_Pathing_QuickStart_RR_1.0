package org.firstinspires.ftc.teamcode.roadrunner.tuning;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.roadrunner.TwoDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.roadrunner.messages.PoseMessage;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;
import org.firstinspires.ftc.teamcode.utils.software.MovingArrayList;

import java.util.List;

public class LocalizationTest extends LinearOpMode {
    protected Intake intake;
    protected Outtake outtake;
    @Override
    public void runOpMode() throws InterruptedException {
        //MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(14.5, 62.0, Math.toRadians(-90)), false);
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(0)), false);
        drive.startIMUThread(this);

        intake = new Intake(hardwareMap, false);
        outtake = new Outtake(hardwareMap, false);

        intake.initialize(false);
        outtake.initialize();

        ThreeDeadWheelLocalizer localizer = (ThreeDeadWheelLocalizer) drive.localizer;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        int begin = localizer.par0.getPositionAndVelocity().position;
        telemetry.addData("par0 encoder begin value: ", begin);
        telemetry.addData("par1 encoder begin value: ", localizer.par1.getPositionAndVelocity().position);
        telemetry.addData("perp encoder begin value: ", localizer.perp.getPositionAndVelocity().position);

        waitForStart();

        MovingArrayList avgLoopTime = new MovingArrayList(50);
        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        while (opModeIsActive()) {
            List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
            for (LynxModule module : allHubs) {
                module.clearBulkCache();
            }

            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));

            drive.updatePoseEstimate();

            telemetry.addData("x", new PoseMessage(drive.pose));

//            int end = localizer.par0.getPositionAndVelocity().position;
//            telemetry.addData("par0 encoder end value: ", end);
//            telemetry.addData("par0 encoder delta: ", (end-begin));
//            telemetry.addData("par1 encoder end value: ", localizer.par1.getPositionAndVelocity().position);
//            telemetry.addData("perp encoder end value: ", localizer.perp.getPositionAndVelocity().position);

//            telemetry.addData("left distance sensor: ", intake.getStackDistanceLeft());
//            telemetry.addData("right distance sensor: ", intake.getStackDistanceRight());

            if(gamepad1.a) {
                Log.d("Localization_logger", "Pose: " + new PoseMessage(drive.pose)
//                                + " | " + "left distance sensor: " + intake.getStackDistanceLeft()
//                                + " | " + "right distance sensor: " + intake.getStackDistanceRight()
                        );
            }
            avgLoopTime.add(timer.milliseconds());
            timer.reset();
            telemetry.addData("Loop Time: Mean:", String.format("%3.3f",avgLoopTime.getMean()) + " | Avg: " + String.format("%3.3f",avgLoopTime.getAvg()));
            telemetry.update();
        }
    }
}
