package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.NullAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.pipeline.AlliancePosition;
import org.firstinspires.ftc.teamcode.pipeline.FieldPosition;

@Config

public abstract class FarAutoBase extends AutoBase {
    // 0 = left, 1 = middle, 2 = right
    public Pose2d start, parking, moveUp1, stackIntakeAlignment2, stackAlignment, stackIntake1, safeTrussPassStop;
    public Pose2d[] spike, backdrop, cycleScore, backOffFromSpike, backdropAlignment,
            stackIntakeAlignment, stackIntake, crossFieldAlignment, cycleStart, backdropAlignmentCycle;

    @Override
    protected Pose2d getStartPose() {
        return start;
    }

    protected void onInit() {
        // this is the default
        super.onInit();
        this.stackPosition = stackIntake1;
        this.sched.setStackAlignmentCallback(this);
    }

    @Override
    protected void onRun() {

        // depends on the spike position, the paths are different
        //
        // For LEFT and RIGHT, intake from the FURTHEST stack
        // For CENTER, intake from the NEAREST stack
        //
        // For BLUE and RED, they share the same paths, only difference is the position
        //-------------------------------------------------------------------------------

        // RED SPIKE right or BLUE left
        //--------------------------------------------
        if ((SPIKE == 2 && getAlliance() == AlliancePosition.RED) ||
                (SPIKE == 0 && getAlliance() == AlliancePosition.BLUE)) {

            sched.addAction(
                    new SequentialAction(
                            new MecanumDrive.DrivePoseLoggingAction(drive, "auto_start"),
                            drive.actionBuilder(drive.pose)
                                    .strafeTo(moveUp1.position,drive.slowVelConstraint,drive.slowAccelConstraint)
                                    .build(),

                            new ParallelAction(
                                    new MecanumDrive.DrivePoseLoggingAction(drive, "to_spike"),
                                    drive.actionBuilder(moveUp1)
                                            .strafeToLinearHeading(spike[SPIKE].position, spike[SPIKE].heading,drive.slowVelConstraint,drive.slowAccelConstraint)
                                            .build(),
                                    new SequentialAction(
                                            intake.stackIntakeLinkageDown()
                                    )
                            ),

                            // drop the purple pixel
                            new MecanumDrive.DrivePoseLoggingAction(drive, "spike"),
                            intake.scorePurplePreload(),
                            new SleepAction(0.25),

                            intake.prepareTeleOpsIntake(),
                            new SleepAction(0.25),

                            new MecanumDrive.DrivePoseLoggingAction(drive, "purple_pixel"),
                            drive.actionBuilder(spike[SPIKE])
                                    .strafeToLinearHeading(backOffFromSpike[SPIKE].position, backOffFromSpike[SPIKE].heading,drive.slowVelConstraint,drive.slowAccelConstraint)
                                    .build(),

                            // move to middle stack on the left
                            new ParallelAction(
                                    new MecanumDrive.DrivePoseLoggingAction(drive, "backOff_from_spike"),
                                    drive.actionBuilder(backOffFromSpike[SPIKE])
                                            .strafeToLinearHeading(stackIntakeAlignment[SPIKE].position,
                                                    stackIntakeAlignment[SPIKE].heading,drive.slowVelConstraint,drive.slowAccelConstraint)
                                            .build(),
                                    intake.prepareStackIntake()
                            ),


                            // move to stack intake position
                            new MecanumDrive.DrivePoseLoggingAction(drive, "stack_intake_alignment"),
                            drive.actionBuilder(stackIntakeAlignment[SPIKE])
                                    .strafeTo(stackIntake[SPIKE].position,drive.slowVelConstraint,drive.slowAccelConstraint)
                                    .build(),

                            new MecanumDrive.DrivePoseLoggingAction(drive, "stack_intake_position"),
                            intake.intakeOneStackedPixels(),

                            new MecanumDrive.DrivePoseLoggingAction(drive, "intake_one_white"),

                            // move to cross field position
                            new ParallelAction(
                                    drive.actionBuilder(stackIntake[SPIKE])
                                            .strafeToLinearHeading(crossFieldAlignment[SPIKE].position, crossFieldAlignment[SPIKE].heading)
                                            .build(),

                                    new SequentialAction(
                                            new SleepAction(0.3),
                                            intake.intakeOneStackedPixels()
                                    )
                            ),

                            new MecanumDrive.DrivePoseLoggingAction(drive, "crossFieldAlignment", true)
                    )
            );

        }
        // SPIKE is center
        //-----------------------------------
        else if (SPIKE == 1) {

            sched.addAction(
                    new SequentialAction(
                            new MecanumDrive.DrivePoseLoggingAction(drive, "start"),
                            intake.stackIntakeLinkageDown(),
                            drive.actionBuilder(drive.pose)
                                    .strafeTo(spike[SPIKE].position,drive.slowVelConstraint,drive.slowAccelConstraint)
                                    .build(),
                            // drop the purple pixel
//                            intake.stackIntakeLinkageDown(),
//                            new SleepAction(0.75),
                            intake.scorePurplePreload(),
                            new SleepAction(0.25),

                            // backoff from spike
                            //
                            new MecanumDrive.DrivePoseLoggingAction(drive, "spike"),
                            drive.actionBuilder(spike[SPIKE])
                                    .setReversed(true)
                                    .strafeTo(backOffFromSpike[SPIKE].position,drive.slowVelConstraint,drive.slowAccelConstraint)
                                    .build(),

                            new MecanumDrive.DrivePoseLoggingAction(drive, "backOff_from_spike"),
                            new ParallelAction(
                                    drive.actionBuilder(backOffFromSpike[SPIKE])
                                            .strafeToLinearHeading(stackIntakeAlignment[SPIKE].position, stackIntakeAlignment[SPIKE].heading,drive.slowVelConstraint,drive.slowAccelConstraint)
                                            .build(),
                                    intake.prepareStackIntake()
                            ),

                            new MecanumDrive.DrivePoseLoggingAction(drive, "stackIntake_alignment"),
                            // move to stack intake position
                            drive.actionBuilder(stackIntakeAlignment[SPIKE])
                                    .strafeTo(stackIntake[SPIKE].position,drive.slowVelConstraint,drive.slowAccelConstraint)
                                    .build(),

                            new MecanumDrive.DrivePoseLoggingAction(drive, "stackIntake"),
                            intake.intakeOneStackedPixels(),

                            new MecanumDrive.DrivePoseLoggingAction(drive, "intake_one_white", true),
                            // move to cross field position
                            new ParallelAction(
                                    drive.actionBuilder(stackIntake[SPIKE])
                                            .strafeToLinearHeading(crossFieldAlignment[SPIKE].position, crossFieldAlignment[SPIKE].heading)
                                            .build(),

                                    new SequentialAction(
                                            new SleepAction(0.3),
                                            intake.intakeOneStackedPixels()
                                    )
                            ),

                            new MecanumDrive.DrivePoseLoggingAction(drive, "cross_field_alignment", true)
                    )
            );

        }
        // RED SPIKE left OR BLUE SPIKE right
        //------------------------------------------------
        else {
            sched.addAction(
                    new SequentialAction(
                            new ParallelAction(
                                    new MecanumDrive.DrivePoseLoggingAction(drive, "start"),
                                    drive.actionBuilder(drive.pose)
                                            .strafeTo(spike[SPIKE].position,drive.slowVelConstraint,drive.slowAccelConstraint)
                                            .build(),
                                    new SequentialAction(
                                            new SleepAction(0.2),
                                            intake.stackIntakeLinkageDown()
                                    )
                            ),

                            new MecanumDrive.DrivePoseLoggingAction(drive, "to_spike"),
                            // drop the purple pixel
                            intake.scorePurplePreload(),
                            new SleepAction(0.25),

                            intake.prepareTeleOpsIntake(),
                            outtake.prepareToTransfer(),

                            new MecanumDrive.DrivePoseLoggingAction(drive, "purple_pixel"),
                            // backoff from spike
                            //
                            drive.actionBuilder(spike[SPIKE])
                                            .setReversed(true)
                                            .strafeTo(backOffFromSpike[SPIKE].position,drive.slowVelConstraint,drive.slowAccelConstraint)
                                            .build(),

                            new MecanumDrive.DrivePoseLoggingAction(drive, "backOff_from_spike"),

                            // move to intake alignment
                            new ParallelAction(
                                    drive.actionBuilder(backOffFromSpike[SPIKE])
                                            .strafeToLinearHeading(stackIntakeAlignment[SPIKE].position, stackIntakeAlignment[SPIKE].heading,drive.slowVelConstraint,drive.slowAccelConstraint)
                                            .build(),

                                    new MecanumDrive.DrivePoseLoggingAction(drive, "stack_intake_alignment"),
                                    new SequentialAction(
                                            new SleepAction(1.0),
                                            intake.prepareStackIntake()
                                    )
                            ),

                            drive.actionBuilder(stackIntakeAlignment[SPIKE])
                                    .strafeToLinearHeading(stackIntakeAlignment2.position, stackIntakeAlignment2.heading)
                                    .build(),

                            new MecanumDrive.DrivePoseLoggingAction(drive, "stack_intake_alignment_2"),

                            // move to stack intake position
                            drive.actionBuilder(stackIntakeAlignment2)
                                    .strafeToLinearHeading(stackIntake[SPIKE].position, stackIntake[SPIKE].heading)
                                    .build(),

                            new MecanumDrive.DrivePoseLoggingAction(drive, "stack_intake_"),
                            intake.intakeOneStackedPixels(),

                            new MecanumDrive.DrivePoseLoggingAction(drive, "intake_one_white", true),
                            // move to stack intake position
                            new ParallelAction(
                                drive.actionBuilder(stackIntake[SPIKE])
                                    .strafeToLinearHeading(crossFieldAlignment[SPIKE].position, crossFieldAlignment[SPIKE].heading,
                                            drive.slowVelConstraint, drive.slowAccelConstraint)
                                    .build(),

                                new SequentialAction(
                                new SleepAction(0.3),
                                        intake.intakeOneStackedPixels()
                                        )
                            ),

                            new MecanumDrive.DrivePoseLoggingAction(drive, "cross_field_alignment", true)

                    )
            );
        }

        Action relocalizationAction = new BackdropRelocalizationAction(drive, outtake, backdrop[SPIKE]);

        if(SPIKE == 1) {
            relocalizationAction = new NullAction();
        }

        // Move to the backdrop side, and potentially add a configurable sleep time
        //-------------------------------------------------------------------------
        double waitTime = doCycle()?0.0:farSideAutoWaitTimeInSeconds;
        sched.addAction(
                new SequentialAction(
                        new ParallelAction(
                                drive.actionBuilder(crossFieldAlignment[SPIKE])
                                        .setReversed(true)
                                        .strafeToLinearHeading(backdropAlignment[SPIKE].position,
                                                backdropAlignment[SPIKE].heading,
                                                drive.highSpeedVelConstraint,
                                                drive.highSpeedAccelConstraint)
                                        .build(),
                                new MecanumDrive.DrivePoseLoggingAction(drive, "backdrop_alignment_end"),

                                new SequentialAction(
                                        new SleepAction(0.5),
                                        intake.stackIntakeLinkageUp(),
                                        new SleepAction(1.2),
                                        intake.prepareTeleOpsIntake(),
                                        new MecanumDrive.DrivePoseLoggingAction(drive, "Intake_off")
                                )
                        ),
                        new MecanumDrive.DrivePoseLoggingAction(drive, "backdrop_alignment_position"),

                        new MecanumDrive.AutoPositionCheckAction(drive, backdropAlignment[SPIKE]),

                        relocalizationAction,
                        new MecanumDrive.DrivePoseLoggingAction(drive, "after_localization"),

                        new SleepAction(waitTime),

                        new ParallelAction(
                                new SequentialAction(
                                        new SleepAction(0.5),
                                    drive.actionBuilder(backdropAlignment[SPIKE])
                                            .setReversed(true)
                                            .strafeToLinearHeading(backdrop[SPIKE].position,
                                                    backdrop[SPIKE].heading,
                                                    drive.slowVelConstraint,
                                                    drive.slowAccelConstraint)
                                            .build(),
                                            new MecanumDrive.DrivePoseLoggingAction(drive, "end_backdrop_position")
                                ),

                                new SequentialAction(
                                        outtake.prepareToSlide(),
                                        new SleepAction(0.5),
                                        outtake.extendOuttakeFarLow(),
                                        new SleepAction(0.5),
                                        outtake.prepareToScoreCycle(),
                                        new SleepAction(0.2)
                                )
                        ),

                        new MecanumDrive.DrivePoseLoggingAction(drive, "start_scoring"),

                        outtake.latchScore1(),
                        new SleepAction(0.65),
                        outtake.afterScore2(),
                        new SleepAction(0.3),
                        outtake.latchScore2(),
                        new SleepAction(0.40),
                        new MecanumDrive.DrivePoseLoggingAction(drive, "score_yellow_preload"),

                        outtake.prepareToSlide(),
                        new SleepAction(0.5),
                        new MecanumDrive.DrivePoseLoggingAction(drive, "backdrop_score_end")
                )
        );

        if(doCycle()) {
            cyclePixelFromStack(backdrop[SPIKE]);
        }

        // prepare for teleops and parking if time
        //-------------------------------------------------
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

    }

    private void cyclePixelFromStack(Pose2d startingPosition) {
        int cycleCount = 1;
        Action extendSlideAction;
        Pose2d stackIntakePosition;
        Vector2d cycleScorePosition = cycleScore[SPIKE].position;

        extendSlideAction = outtake.extendOuttakeCycleTwo();
        stackIntakePosition = stackIntake1;

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
                                        new SleepAction(0.8),
                                        intake.stackIntakeLinkageDown()
                                )
                        ),

                        new MecanumDrive.DrivePoseLoggingAction(drive, "stack_alignment_position")
                ));

        sched.addAction(
                new AutoBase.StackIntakePositionAction(drive,intake,stackIntakePosition)
        );

        sched.addAction(
                new SequentialAction(
                        // move back to the backdrop
                        new ParallelAction(
                                new SequentialAction(
                                        drive.actionBuilder(stackIntakePosition)
                                                .setReversed(true)
                                                .strafeToLinearHeading(backdropAlignmentCycle[SPIKE].position,backdropAlignmentCycle[SPIKE].heading,
                                                        this.drive.highSpeedVelConstraint,
                                                        this.drive.highSpeedAccelConstraint)
                                                .build(),
                                        new MecanumDrive.DrivePoseLoggingAction(drive, "backdrop_alignment_end")
                                ),

                                new SequentialAction(
                                        new SleepAction(1.2),
                                        intake.prepareTeleOpsIntake(),
                                        new MecanumDrive.DrivePoseLoggingAction(drive, "Intake_off")
                                )
                        ),

                        new MecanumDrive.DrivePoseLoggingAction(drive, "Before_backdrop_score"),
                        new MecanumDrive.AutoPositionCheckAction(drive, backdropAlignmentCycle[SPIKE]),

                        new BackdropRelocalizationAction(drive, outtake, cycleScore[SPIKE]),
                        new MecanumDrive.DrivePoseLoggingAction(drive, "after_localization"),

                        // move to backdrop scoring position
                        new ParallelAction(
                                new SequentialAction(
                                        drive.actionBuilder(backdropAlignmentCycle[SPIKE])
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
                        )
                ));
        sched.addAction(
                new SequentialAction(
                        // score pixels
                        new MecanumDrive.DrivePoseLoggingAction(drive, "cycle_score_" + cycleCount + "_open_latch_start"),
                        outtake.latchScore1(),
                        new SleepAction(0.30),
                        outtake.latchScore2(),
                        new SleepAction(0.25)
                )
        );

        sched.addAction(
                new SequentialAction(
                        outtake.afterScore2(),
                        new SleepAction(0.2)
                )
        );

        sched.addAction(
                new MecanumDrive.DrivePoseLoggingAction(drive, "cycle_" + cycleCount + "_score_end")
        );
    }

    @Override
    public FieldPosition getFieldPosition() {
        return FieldPosition.FAR;
    }

    protected boolean doCycle() {
        return false;
    }

}
