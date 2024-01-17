package org.firstinspires.ftc.teamcode.opmode.autonomous;

import android.util.Log;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.NullAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.pipeline.FieldPosition;
import org.firstinspires.ftc.teamcode.roadrunner.messages.PoseMessage;

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

    protected void onInit() {
        // this is the default
        super.onInit();
        this.stackPosition = stackIntake1;
        this.sched.setStackAlignmentCallback(this);
    }

    @Override
    protected void onRun() {

        Log.d("Drive_logger", "starting:" + new PoseMessage(drive.pose));
        Log.d("Drive_logger", "backdrop position:" + new PoseMessage(backdrop[SPIKE]));

        sched.addAction(
                new SequentialAction(
                        new MecanumDrive.DrivePoseLoggingAction(drive, "starting_position"),
                        outtake.prepareToSlide(),
                        // to score yellow pixel on the backdrop
                        new ParallelAction(
                                drive.actionBuilder(drive.pose)
                                        .strafeToLinearHeading(backdrop[SPIKE].position, backdrop[SPIKE].heading)
                                        .build(),

                                new SequentialAction(

                                        new SleepAction(1.0),
                                        outtake.extendOuttakeLow()
                                )
                        ),

                        new MecanumDrive.DrivePoseLoggingAction(drive, "backdrop_position"),
                        //new MecanumDrive.UpdateDrivePoseAction(drive, this.visionPortal2, this.aprilTag),

                        outtake.prepareToScore(),
                        new SleepAction(0.30),
                        outtake.latchScore1(),
                        new SleepAction(0.60),
                        intake.stackIntakeLinkageDown(),
                        outtake.afterScore(),
                        new SleepAction(0.2),
                        new MecanumDrive.DrivePoseLoggingAction(drive, "end_of_scoring_position"),
                        new ParallelAction(
                                outtake.retractOuttake(),

                                // to score the purple pixel on the spike
                                drive.actionBuilder(backdrop[SPIKE])
                                        .strafeToLinearHeading(spike[SPIKE].position, spike[SPIKE].heading, this.drive.highSpeedVelConstraint, this.drive.highSpeedAccelConstraint)
                                        .build()
                        ),

                        new MecanumDrive.DrivePoseLoggingAction(drive, "spike_position"),

                        intake.scorePurplePreload(),
                        new SleepAction(0.2)
                )
        );

//        sched.addAction(
//                new SequentialAction(
//                new ParallelAction(
//                        outtake.retractOuttake(),
//                        intake.prepareTeleOpsIntake(),
//                        drive.actionBuilder(spike[SPIKE]) //spike[SPIKE]
//                                .strafeToLinearHeading(cycleStart[SPIKE].position, cycleStart[SPIKE].heading, this.drive.highSpeedVelConstraint, this.drive.highSpeedAccelConstraint)
//                                .build()
//                        ),
//
//                        new MecanumDrive.DrivePoseLoggingAction(drive, "start_cycle_position", true)
//                )
//        );

        // do the first cycle from the spike position
        cyclePixelFromStack(spike[SPIKE]);

        // do the 2nd cycle from the cycle drop position
        cyclePixelFromStack(cycleScore[SPIKE]);

        sched.addAction(new ParallelAction(
                        new SequentialAction(
                                outtake.retractOuttake(),
                                new SleepAction(0.5),
                                intake.prepareTeleOpsIntake(),
                                outtake.prepareToTransfer(),
                                new MecanumDrive.DrivePoseLoggingAction(drive, "slides_retracted_completed")
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
                                        .strafeToLinearHeading(cycleStart[SPIKE].position, cycleStart[SPIKE].heading, this.drive.highSpeedVelConstraint, this.drive.highSpeedAccelConstraint)
                                        .strafeToLinearHeading(stackAlignment.position,stackAlignment.heading,
                                                this.drive.highSpeedVelConstraint, this.drive.highSpeedAccelConstraint)
                                        .build(),
                                new SequentialAction(
                                        new SleepAction(1.8),
                                        outtake.prepareToTransfer(),
                                        new SleepAction(1.0),
                                        intake.stackIntakeLinkageDown()
                                )
                        ),

                        new MecanumDrive.DrivePoseLoggingAction(drive, "stack_alignment_position")
                ));

        sched.addAction(
                new SleepAction(0.15)
        );

        sched.addAction(
                new AutoBase.StackIntakePositionAction(drive,intake,portal,stackPipeline, stackIntakePosition)
        );

        sched.addAction(
                new SequentialAction(
//                        // drive to the stack
//                        new ParallelAction(
//                                drive.actionBuilder(stackAlignment)
//                                        .strafeToLinearHeading(stackIntakePosition.position, stackIntakePosition.heading,
//                                                this.drive.slowVelConstraint,
//                                                this.drive.slowAccelConstraint)
//                                        .build(),
//                                intake.intakeOn()
//                        ),

                        new MecanumDrive.DrivePoseLoggingAction(drive, "stack_intake_start_position"),

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
                                        new SleepAction(0.8),
                                        intake.stackIntakeLinkageUp(),
                                        new SleepAction(1.8),
                                        intake.prepareTeleOpsIntake(),
                                        new MecanumDrive.DrivePoseLoggingAction(drive, "Intake_off")
                                )
                        ),

                        new MecanumDrive.DrivePoseLoggingAction(drive, "Before_backdrop_score"),

                        new MecanumDrive.AutoPositionCheckAction(drive, backdropAlignment),

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
                                        outtake.prepareToSlide(),
                                        new SleepAction(0.3),
                                        new MecanumDrive.DrivePoseLoggingAction(drive, "cycle_" + cycleCount + "_prepare"),
                                        extendSlideAction,
                                        new MecanumDrive.DrivePoseLoggingAction(drive, "cycle_" + cycleCount + "_extend"),
                                        new SleepAction(0.3),
                                        outtake.prepareToScoreCycle(),
                                        new SleepAction(0.2)
                                )
                        ),

                        // score pixels
                        new MecanumDrive.DrivePoseLoggingAction(drive, "cycle_score_" + cycleCount + "_open_latch_start"),
                        outtake.latchScore1(),
                        new SleepAction(0.3),
                        outtake.latchScore2(),
                        new SleepAction(0.4),
//                        outtake.latchScore2(),
//                        new SleepAction(0.5),
//                        outtake.afterScore(),
//                        new SleepAction(0.2),
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

    @Override
    public Action driveToStack() {
        return new ParallelAction(
                new MecanumDrive.DrivePoseLoggingAction(drive, "cycle_" + cycleCount + "_auto_alignment_start"),
                drive.actionBuilder(stackAlignment)
                        .strafeToLinearHeading(getStackPosition().position, getStackPosition().heading)
                        .build(),
                intake.intakeOn()
        );
    }
}
