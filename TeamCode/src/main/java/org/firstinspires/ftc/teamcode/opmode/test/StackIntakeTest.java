package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.utils.software.ActionScheduler;

@Config
@Disabled
@TeleOp(group = "Test")
public class StackIntakeTest extends LinearOpMode {

    protected ActionScheduler sched;
    protected Intake intake;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        intake = new Intake(hardwareMap);
        sched = new ActionScheduler();

        sched.queueAction(
                new SequentialAction(
                        intake.stackIntakeLinkageDown(),
                        new SleepAction(0.5),
                        intake.intakeTwoStackedPixels()
                ));

        waitForStart();

        while (!isStopRequested()) {
            sched.update();
            telemetry.addLine(intake.getStackServoPositions());
            telemetry.update();
        }
    }
}
