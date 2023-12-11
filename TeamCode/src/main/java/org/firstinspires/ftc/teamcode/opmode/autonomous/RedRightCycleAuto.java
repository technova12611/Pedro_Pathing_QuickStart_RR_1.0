package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.pipeline.AlliancePosition;
import org.firstinspires.ftc.teamcode.pipeline.FieldPosition;

@Config
@Autonomous(name = "RED Right Cycle Auto", group = "Auto Cycle", preselectTeleOp = "Manual Drive")
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

    public static Pose2d stackAlignment = new Pose2d(-48.0, -11.0, Math.toRadians(180));
    public static Pose2d stackIntake = new Pose2d(-55.0, -12.75, Math.toRadians(180));

//   public static Pose2d stackAlignment = new Pose2d(-1.0, -12.0, Math.toRadians(180));
//   public static Pose2d stackIntake = new Pose2d(-9.0, -12.0, Math.toRadians(180));

    public static Pose2d backdropAlignment = new Pose2d(45.0, -10.5, Math.toRadians(180));

    public static Pose2d cycleScore[] = {
            new Pose2d(49.7, -39.0, Math.toRadians(180)),
            new Pose2d(49.7, -32.0, Math.toRadians(180)),
            new Pose2d(49.7, -32.0, Math.toRadians(180))
    };


    // 0 = left, 1 = middle, 2 = right
    public static Pose2d start = new Pose2d(16.0, -62.0, Math.toRadians(90));
    public static Pose2d parking = new Pose2d(53.0, -60.0, Math.toRadians(180));

    protected AlliancePosition getAlliance() {
        return AlliancePosition.RED;
    }

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
                        new SleepAction(0.25),
                        outtake.latchScore1(),
                        new SleepAction(0.75),
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
                        new SleepAction(0.25)
                )
        );

        // do the first cycle from the spike position
        cyclePixelFromStack(spike[SPIKE]);

        // do the 2nd cycle from the cycle drop position
        cyclePixelFromStack(cycleScore[SPIKE]);

        // prepare for
        sched.addAction(intake.prepareTeleOpsIntake());
        sched.addAction(outtake.prepareToTransfer());
    }

    private void cyclePixelFromStack(Pose2d startingPosition) {

        sched.addAction(
                new SequentialAction(

                        // to strafe to cycle start teleops
                        new ParallelAction(
                                intake.prepareTeleOpsIntake(),
                                outtake.prepareToTransfer(),
                                drive.actionBuilder(startingPosition) //spike[SPIKE]
                                        .strafeTo(cycleStart[SPIKE].position)
                                        .build()
                        ),

                        new MecanumDrive.DrivePoseLoggingAction(drive, "cycle_start_position"),

                        // move to stack alignment position
                        new ParallelAction(
                                drive.actionBuilder(cycleStart[SPIKE])
                                        .strafeTo(stackAlignment.position)
                                        .build(),
                                new SequentialAction(
                                        new SleepAction(1.0),
                                        intake.stackIntakeLinkageDown()
                                )
                        ),


                        new MecanumDrive.DrivePoseLoggingAction(drive, "stack_alignment_position"),

                        //intake.prepareStackIntake(),
                        //new SleepAction(0.5),

                        // drive to the stack
                        new ParallelAction(
                                drive.actionBuilder(stackAlignment)
                                        .strafeTo(stackIntake.position)
                                        .build(),
                                intake.intakeOn()
                        ),

                        new MecanumDrive.DrivePoseLoggingAction(drive, "stack_intake_position"),

                        // intake the pixels from the stack
                        intake.intakeTwoStackedPixels(),

                        new ParallelAction(
                                drive.actionBuilder(stackIntake)
                                        .setReversed(true)
                                        .strafeTo(backdropAlignment.position)
                                        .build(),
                                new SequentialAction(
                                        new SleepAction(0.5),
                                        intake.prepareTeleOpsIntake()
                                )
                        ),

                        new MecanumDrive.DrivePoseLoggingAction(drive, "backdrop_alignment_position"),

                        new ParallelAction(
                                drive.actionBuilder(backdropAlignment)
                                        .setReversed(true)
                                        .strafeTo(cycleScore[SPIKE].position)
                                        .build(),

                                new SequentialAction(
                                        new SleepAction(0.2),
                                        outtake.prepareToSlide(),
                                        new SleepAction(0.5),
                                        outtake.extendOuttakeCycle()
                                )
                        ),

                        new MecanumDrive.DrivePoseLoggingAction(drive, "cycle_score_position"),

                        outtake.prepareToScoreCycle(),
                        new SleepAction(0.5),
                        outtake.latchScore2(),
                        new SleepAction(0.75),

                        new MecanumDrive.DrivePoseLoggingAction(drive, "cycle_score_end"),
                        outtake.retractOuttake(),
                        new SleepAction(0.25),

                        new MecanumDrive.DrivePoseLoggingAction(drive, "cycle_complete")
                ));
    }

    @Override
    public FieldPosition getFieldPosition() {
        return FieldPosition.NEAR;
    }

}
