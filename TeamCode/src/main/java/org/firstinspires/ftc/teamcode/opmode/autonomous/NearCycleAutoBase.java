package org.firstinspires.ftc.teamcode.opmode.autonomous;

import android.util.Log;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.pipeline.FieldPosition;
import org.firstinspires.ftc.teamcode.roadrunner.PoseMessage;

public abstract class NearCycleAutoBase extends AutoBase {
    // 0 = left, 1 = middle, 2 = right
    public Pose2d start;
    public Pose2d[] backdrop ;
    public Pose2d[] spike;
    public Pose2d[] cycleStart;
    public Pose2d stackAlignment;
    public Pose2d stackIntake1;
    public Pose2d stackIntake2;
    public Pose2d safeTrussPassStop;
    public Pose2d backdropAlignment;
    public Pose2d[] cycleScore;
    public Pose2d parking;
    private int cycleCount = 0;

    @Override
    protected void onRun() {

        Log.d("Drive_logger", "starting:" + new PoseMessage(drive.pose));
        Log.d("Drive_logger", "backdrop position:" + new PoseMessage(backdrop[SPIKE]));

        sched.addAction(
                new SequentialAction(
                        new MecanumDrive.DrivePoseLoggingAction(drive, "starting_position"),
                        // to score yellow pixel on the backdrop
                        new ParallelAction(
                                drive.actionBuilder(drive.pose)
                                        .strafeToLinearHeading(backdrop[SPIKE].position, backdrop[SPIKE].heading)
                                        .build(),

                                outtake.prepareToSlide(),

                                new SequentialAction(
                                        new SleepAction(1.0),
                                        outtake.extendOuttakeLow()
                                )
                        ),

                        new MecanumDrive.DrivePoseLoggingAction(drive, "backdrop_position"),

                        outtake.prepareToScore(),
                        new SleepAction(0.30),
                        outtake.latchScore1(),
                        new SleepAction(0.50),
                        intake.stackIntakeLinkageDown(),
                        new SleepAction(0.2),
                        new ParallelAction(
                            new SequentialAction(
                                new SleepAction(0.2),
                                outtake.retractOuttake()
                            ),

                            // to score the purple pixel on the spike
                            drive.actionBuilder(backdrop[SPIKE])
                                    .strafeTo(spike[SPIKE].position, this.drive.highSpeedVelConstraint, this.drive.highSpeedAccelConstraint)
                                    .build()
                        ),

                        new MecanumDrive.DrivePoseLoggingAction(drive, "spike_position"),

                        intake.scorePurplePreload(),
                        new SleepAction(0.2)
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
        Pose2d stackIntakePosition;
        Vector2d cycleScorePosition = cycleScore[SPIKE].position;
        if(++cycleCount == 2) {
            extendSlideAction = outtake.extendOuttakeCycleTwo();
            stackIntakePosition = stackIntake2;
            cycleScorePosition = new Vector2d(cycleScorePosition.x-0.5, cycleScorePosition.y);
        } else {
            extendSlideAction = outtake.extendOuttakeCycleOne();
            stackIntakePosition = stackIntake1;
        }

        sched.addAction(
                new SequentialAction(
                        // to strafe to cycle start teleops
                        new ParallelAction(
                                outtake.retractOuttake(),
                                intake.prepareTeleOpsIntake(),
                                drive.actionBuilder(startingPosition) //spike[SPIKE]
                                        .strafeTo(cycleStart[SPIKE].position, this.drive.highSpeedVelConstraint, this.drive.highSpeedAccelConstraint)
                                        .strafeTo(stackAlignment.position,
                                                this.drive.highSpeedVelConstraint, this.drive.highSpeedAccelConstraint)
                                        .build(),
                                new SequentialAction(
                                        new SleepAction(2.0),
                                        outtake.prepareToTransfer(),
                                        new SleepAction(1.0),
                                        intake.stackIntakeLinkageDown()
                                )
                        ),

                        new MecanumDrive.DrivePoseLoggingAction(drive, "stack_alignment"),

                        // drive to the stack
                        new ParallelAction(
                                drive.actionBuilder(stackAlignment)
                                        .strafeToLinearHeading(stackIntakePosition.position, stackIntakePosition.heading)
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
                                    drive.actionBuilder(stackIntakePosition)
                                            .setReversed(true)
                                            .strafeToLinearHeading(safeTrussPassStop.position, safeTrussPassStop.heading)
                                            .strafeToLinearHeading(backdropAlignment.position,backdropAlignment.heading,
                                                    this.drive.highSpeedVelConstraint,
                                                    this.drive.highSpeedAccelConstraint)
                                            .build(),
                                         new MecanumDrive.DrivePoseLoggingAction(drive, "backdrop_alignment_end")
                                ),

                                new SequentialAction(
                                        new SleepAction(0.5),
                                        intake.stackIntakeLinkageUp(),
                                        new SleepAction(2.0),
                                        intake.prepareTeleOpsIntake(),
                                        new MecanumDrive.DrivePoseLoggingAction(drive, "Intake_off")
                                )
                        ),

                        new MecanumDrive.DrivePoseLoggingAction(drive, "backdrop_move_to_score"),

                        // move to backdrop scoring position
                        new ParallelAction(
                                new SequentialAction(
                                    drive.actionBuilder(backdropAlignment)
                                            .setReversed(true)
                                            .strafeToLinearHeading(cycleScorePosition, cycleScore[SPIKE].heading)
                                            .build(),
                                        new MecanumDrive.DrivePoseLoggingAction(drive, "cycle_" + cycleCount + "_score_position")
                                ),

                                new SequentialAction(
                                        new SleepAction(0.1),
                                        outtake.prepareToSlide(),
                                        new SleepAction(0.4),
                                        new MecanumDrive.DrivePoseLoggingAction(drive, "cycle_" + cycleCount + "_prepare"),
                                        extendSlideAction,
                                        new MecanumDrive.DrivePoseLoggingAction(drive, "cycle_" + cycleCount + "_extend"),
                                        new SleepAction(0.4),
                                        outtake.prepareToScoreCycle(),
                                        new SleepAction(0.2)
                                )
                        ),

                        // score pixels
                        new MecanumDrive.DrivePoseLoggingAction(drive, "cycle_score_" + cycleCount + "_open_latch_start"),
                        new SleepAction(0.1),
                        outtake.latchScore1(),
                        new SleepAction(0.3),
                        outtake.latchScore2(),
                        new SleepAction(0.4),
                        outtake.afterScore(),
                        new SleepAction(0.2),
                        new MecanumDrive.DrivePoseLoggingAction(drive, "cycle_" + cycleCount + "_score_end")
                ));
    }

    @Override
    public FieldPosition getFieldPosition() {
        return FieldPosition.NEAR;
    }

    @Override
    protected Pose2d getStartPose() {
        return this.start;
    }

}
