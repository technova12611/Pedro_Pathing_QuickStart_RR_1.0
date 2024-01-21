package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;
import org.firstinspires.ftc.teamcode.utils.software.ActionScheduler;
import org.firstinspires.ftc.teamcode.utils.software.AutoActionScheduler;

@Config
@TeleOp(group = "Test")
public class StackIntakeTest extends LinearOpMode {

    protected AutoActionScheduler sched;
    protected Intake intake;
    protected Outtake outtake;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
        intake.initialize(false);
        outtake.initialize();

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        sched = new AutoActionScheduler(this::update);

        sched.addAction(
                new SequentialAction(
                        intake.stackIntakeLinkageDown(),
                        intake.intakeOn(),
                        new SleepAction(1.0),
                        intake.intakeTwoStackedPixels()
                ));

//        sched.addAction(
//                new SleepAction(2.25)
//               );

        sched.addAction(
                new ParallelAction(
                    drive.actionBuilder(drive.pose)
                    .lineToX(-2.0)
                    .build(),

                   new SequentialAction(
                           new SleepAction(0.3),
                           intake.intakeTwoStackedPixels2(),
                           new SleepAction(2.0),
                           intake.prepareTeleOpsIntake(),
                           new SleepAction(0.5)
                        ))
        );

        intake.stackIntakeLinkageDownDirect();

        waitForStart();

        while (!isStopRequested()) {
            sched.run();
            telemetry.addLine(intake.getStackServoPositions());
            telemetry.update();
            idle();
        }
    }

    public void update() {
        intake.update();
        outtake.update();
    }
}
