package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.pipeline.AlliancePosition;
import org.firstinspires.ftc.teamcode.pipeline.FieldPosition;

@Config
@Autonomous(name = "RED Right Cycle Auto", group = "RED Auto", preselectTeleOp = "Manual Drive")
public class RedRightCycleAuto extends AutoBase {
    public static Pose2d[] backdrop = {
            new Pose2d(49.2, -29, Math.toRadians(180)),
            new Pose2d(49.2, -36.5, Math.toRadians(180)),
            new Pose2d(49.2, -42, Math.toRadians(180))
    };
    public static Pose2d[] spike = {
            new Pose2d(14.5, -34.5, Math.toRadians(180)),
            new Pose2d(28.5, -24.5, Math.toRadians(180)),
            new Pose2d(36.0, -27.5, Math.toRadians(180))
    };

    public static Pose2d[] cycleStart = {
            new Pose2d(spike[0].position.x, -12.0, Math.toRadians(180)),
            new Pose2d(spike[1].position.x, -12.0, Math.toRadians(180)),
            new Pose2d(spike[2].position.x, -12.0, Math.toRadians(180))
    };

    public static Pose2d stackAlignment = new Pose2d(-50.0, -11.5, Math.toRadians(180));
    public static Pose2d stackIntake = new Pose2d(-54.5, -12.75, Math.toRadians(180));

//   public static Pose2d stackAlignment = new Pose2d(-1.0, -12.0, Math.toRadians(180));
//   public static Pose2d stackIntake = new Pose2d(-9.0, -12.0, Math.toRadians(180));

    public static Pose2d backdropAlignment = new Pose2d(45.0, -10.5, Math.toRadians(180));

    public static Pose2d cycleScore[] = {
            new Pose2d(48.5, -39.0, Math.toRadians(180)),
            new Pose2d(48.5, -32.0, Math.toRadians(180)),
            new Pose2d(48.5, -32.0, Math.toRadians(180))
    };

    public static Pose2d safeTrussPassStop = new Pose2d(-35.0, -10.5, Math.toRadians(180));

    // 0 = left, 1 = middle, 2 = right
    public static Pose2d start = new Pose2d(16.0, -62.0, Math.toRadians(90));
    public static Pose2d parking = new Pose2d(45.0, -20.0, Math.toRadians(180));

    protected AlliancePosition getAlliance() {
        return AlliancePosition.RED;
    }

    private int cycleCount = 0;

    @Override
    protected Pose2d getStartPose() {
        return start;
    }

    @Override
    protected void printDescription() {
        telemetry.addData("Description", "RED Right Cycle Auto");
    }

    @Override
    protected void onRun() {
        sched.addAction(
                new SequentialAction(
                        new MecanumDrive.DrivePoseLoggingAction(drive, "starting_position"),
                        // to score yellow pixel on the backdrop
                        new ParallelAction(
                                drive.actionBuilder(drive.pose)
                                        .setTangent(0)
                                        .splineTo(backdrop[SPIKE].position, Math.PI / 2)
                                        .build(),

                                outtake.prepareToSlide(),
                                new SequentialAction(
                                        new SleepAction(1.5),
                                        outtake.extendOuttakeLow()
                                )
                        ),

                        new MecanumDrive.DrivePoseLoggingAction(drive, "backdrop_position"),

                        outtake.prepareToScore(),
                        new SleepAction(0.20),
                        outtake.latchScore1(),
                        new SleepAction(0.60),
                        new ParallelAction(
                                outtake.retractOuttake(),
                                intake.stackIntakeLinkageDown(),

                                // to score the purple pixel on the spike
                                drive.actionBuilder(backdrop[SPIKE])
                                        .strafeTo(spike[SPIKE].position)
                                        .build()
                        ),

                        new MecanumDrive.DrivePoseLoggingAction(drive, "spike_position"),

                        intake.scorePurplePreload(),
                        new SleepAction(0.15)
                )
        );

        // do the first cycle from the spike position
        cyclePixelFromStack(spike[SPIKE]);

        // do the 2nd cycle from the cycle drop position
        cyclePixelFromStack(cycleScore[SPIKE]);

        sched.addAction(new ParallelAction(
                        new SequentialAction(
                                outtake.retractOuttake(),
                                new SleepAction(0.5),
                                new MecanumDrive.DrivePoseLoggingAction(drive, "slides_retracted_completed"),
                                intake.prepareTeleOpsIntake(),
                                outtake.prepareToTransfer()
                        ),

                        new SequentialAction(

                                new MecanumDrive.DrivePoseLoggingAction(drive, "start_of_parking"),
                                // to score the purple pixel on the spike
                                drive.actionBuilder(cycleScore[SPIKE])
                                        .strafeTo(parking.position)
                                        .build(),
                                new MecanumDrive.DrivePoseLoggingAction(drive, "end_of_parking")
                        )
                )
        );

        sched.addAction(new MecanumDrive.DrivePoseLoggingAction(drive, "auto_end_position", true));
    }

    private void cyclePixelFromStack(Pose2d startingPosition) {
        Action extendSlideAction;
        if(++cycleCount == 2) {
            extendSlideAction = outtake.extendOuttakeCycleTwo();
        } else {
            extendSlideAction = outtake.extendOuttakeCycleOne();
        }

        sched.addAction(
                new SequentialAction(
                        // to strafe to cycle start teleops
                        new ParallelAction(
                                outtake.retractOuttake(),
                                intake.prepareTeleOpsIntake(),
                                drive.actionBuilder(startingPosition) //spike[SPIKE]
                                        .strafeTo(cycleStart[SPIKE].position)
                                        .build()
                        ),

                        new MecanumDrive.DrivePoseLoggingAction(drive, "cycle_" + cycleCount + "_start"),

                        // move to stack alignment position
                        new ParallelAction(
                                outtake.prepareToTransfer(),
                                drive.actionBuilder(cycleStart[SPIKE])
                                        .strafeTo(stackAlignment.position,
                                                this.drive.highSpeedVelConstraint, this.drive.highSpeedAccelConstraint)
                                        .build(),
                                new SequentialAction(
                                        new SleepAction(1.0),
                                        intake.stackIntakeLinkageDown()
                                )
                        ),

                        new MecanumDrive.DrivePoseLoggingAction(drive, "stack_alignment"),

                        // drive to the stack
                        new ParallelAction(
                                drive.actionBuilder(stackAlignment)
                                        .strafeTo(stackIntake.position)
                                        .build(),
                                intake.intakeOn()
                        ),

                        new MecanumDrive.DrivePoseLoggingAction(drive, "stack_intake_start"),

                        // intake the pixels from the stack
                        intake.intakeTwoStackedPixels(),

                        new MecanumDrive.DrivePoseLoggingAction(drive, "stack_intake_end", true),

                        // move back to the backdrop
                        new ParallelAction(
                                new SequentialAction(
                                    drive.actionBuilder(stackIntake)
                                            .setReversed(true)
                                            .strafeTo(safeTrussPassStop.position)
                                            .build(),
                                        new MecanumDrive.DrivePoseLoggingAction(drive, "safe_pass_stop"),

                                        drive.actionBuilder(safeTrussPassStop)
                                                .setReversed(true)
                                                .strafeTo(backdropAlignment.position,
                                                        this.drive.highSpeedVelConstraint,
                                                        this.drive.highSpeedAccelConstraint)
                                                .build()
                                ),

                                new SequentialAction(
                                        new SleepAction(0.5),
                                        intake.stackIntakeLinkageUp(),
                                        new SleepAction(1.5),
                                        intake.prepareTeleOpsIntake()
                                )
                        ),

                        new MecanumDrive.DrivePoseLoggingAction(drive, "backdrop_alignment"),

                        // move to backdrop scoring position
                        new ParallelAction(
                                drive.actionBuilder(backdropAlignment)
                                        .setReversed(true)
                                        .strafeTo(cycleScore[SPIKE].position)
                                        .build(),

                                new SequentialAction(
                                        new SleepAction(0.2),
                                        outtake.prepareToSlide(),
                                        new SleepAction(0.5),
                                        extendSlideAction
                                )
                        ),

                        new MecanumDrive.DrivePoseLoggingAction(drive, "cycle_" + cycleCount + "_score"),

                        // score pixels
                        outtake.prepareToScoreCycle(),
                        new SleepAction(0.25),
                        new MecanumDrive.DrivePoseLoggingAction(drive, "cycle_score_" + cycleCount + "_open_latch"),
                        outtake.latchScore2(),
                        new SleepAction(0.75),
                        new MecanumDrive.DrivePoseLoggingAction(drive, "cycle_" + cycleCount + "_score_end")
                ));
    }

    @Override
    public FieldPosition getFieldPosition() {
        return FieldPosition.NEAR;
    }

}
