package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.pipeline.AlliancePosition;
import org.firstinspires.ftc.teamcode.pipeline.FieldPosition;

@Config

public abstract class FarAutoBase extends AutoBase {
    // 0 = left, 1 = middle, 2 = right
    public Pose2d start, parking, backdropAlignment, moveUp1, stackIntakeAlignment2;
    public Pose2d[] spike, backdrop, cycleScore, backOffFromSpike,
            stackIntakeAlignment, stackIntake, crossFieldAlignment;
    @Override
    protected Pose2d getStartPose() {
        return start;
    }

    @Override
    protected void onRun() {
        // RED SPIKE right or BLUE left
        if ((SPIKE == 2 && getAlliance() == AlliancePosition.RED) ||
                (SPIKE == 0 && getAlliance() == AlliancePosition.BLUE)) {

            sched.addAction(
                    new SequentialAction(
                            new MecanumDrive.DrivePoseLoggingAction(drive, "auto_start"),
                            drive.actionBuilder(drive.pose)
                                    .strafeTo(moveUp1.position)
                                    .build(),

                            new ParallelAction(
                                    new MecanumDrive.DrivePoseLoggingAction(drive, "to_spike"),
                                    drive.actionBuilder(moveUp1)
                                            .strafeToLinearHeading(spike[SPIKE].position, spike[SPIKE].heading)
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
                                    .strafeToLinearHeading(backOffFromSpike[SPIKE].position, backOffFromSpike[SPIKE].heading)
                                    .build(),

                            // move to middle stack on the left
                            new ParallelAction(
                                    new MecanumDrive.DrivePoseLoggingAction(drive, "backOff_from_spike"),
                                    drive.actionBuilder(backOffFromSpike[SPIKE])
                                            .strafeToLinearHeading(stackIntakeAlignment[SPIKE].position, stackIntakeAlignment[SPIKE].heading)
                                            .build(),
                                    intake.prepareStackIntake()
                            ),


                            // move to stack intake position
                            new MecanumDrive.DrivePoseLoggingAction(drive, "stack_intake_alignment"),
                            drive.actionBuilder(stackIntakeAlignment[SPIKE])
                                    .strafeTo(stackIntake[SPIKE].position)
                                    .build(),

                            new MecanumDrive.DrivePoseLoggingAction(drive, "stack_intake_position"),
                            intake.intakeOneStackedPixels(),

                            new MecanumDrive.DrivePoseLoggingAction(drive, "intake_one_white"),
                            // move to stack intake position
                            drive.actionBuilder(stackIntake[SPIKE])
                                    .strafeTo(crossFieldAlignment[SPIKE].position)
                                    .build(),

                            new MecanumDrive.DrivePoseLoggingAction(drive, "crossFieldAlignment", true),

                            intake.prepareTeleOpsIntake()
                    )
            );

        }
        // SPIKE is center
        else if (SPIKE == 1) {

            sched.addAction(
                    new SequentialAction(
                            new MecanumDrive.DrivePoseLoggingAction(drive, "start"),
                            drive.actionBuilder(drive.pose)
                                    .strafeTo(spike[SPIKE].position)
                                    .build(),
                            // drop the purple pixel
                            intake.stackIntakeLinkageDown(),
                            new SleepAction(0.75),
                            intake.scorePurplePreload(),
                            new SleepAction(0.25),

                            // backoff from spike
                            //
                            new MecanumDrive.DrivePoseLoggingAction(drive, "spike"),
                            drive.actionBuilder(spike[SPIKE])
                                    .setReversed(true)
                                    .strafeTo(backOffFromSpike[SPIKE].position)
                                    .build(),

                            new MecanumDrive.DrivePoseLoggingAction(drive, "backOff_from_spike"),
                            new ParallelAction(
                                    drive.actionBuilder(backOffFromSpike[SPIKE])
                                            .strafeToLinearHeading(stackIntakeAlignment[SPIKE].position, stackIntakeAlignment[SPIKE].heading)
                                            .build(),
                                    intake.prepareStackIntake()
                            ),

                            new MecanumDrive.DrivePoseLoggingAction(drive, "stackIntake_alignment"),
                            // move to stack intake position
                            drive.actionBuilder(stackIntakeAlignment[SPIKE])
                                    .strafeTo(stackIntake[SPIKE].position)
                                    .build(),

                            new MecanumDrive.DrivePoseLoggingAction(drive, "stackIntake"),
                            intake.intakeOneStackedPixels(),

                            new MecanumDrive.DrivePoseLoggingAction(drive, "intake_one_white", true),
                            // move to stack intake position
                            drive.actionBuilder(stackIntake[SPIKE])
                                    .strafeTo(crossFieldAlignment[SPIKE].position)
                                    .build(),
                            new MecanumDrive.DrivePoseLoggingAction(drive, "cross_field_alignment", true),

                            intake.prepareTeleOpsIntake()

                    )
            );

        }
        // RED SPIKE left OR BLUE SPIKE right
        else {
            sched.addAction(
                    new SequentialAction(
                            new ParallelAction(
                                    new MecanumDrive.DrivePoseLoggingAction(drive, "start"),
                                    drive.actionBuilder(drive.pose)
                                            .strafeTo(spike[SPIKE].position)
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
                                            .strafeTo(backOffFromSpike[SPIKE].position)
                                            .build(),

                            new MecanumDrive.DrivePoseLoggingAction(drive, "backOff_from_spike"),

                            // move to intake alignment
                            new ParallelAction(
                                    drive.actionBuilder(backOffFromSpike[SPIKE])
                                            .strafeToLinearHeading(stackIntakeAlignment[SPIKE].position, stackIntakeAlignment[SPIKE].heading)
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
                            drive.actionBuilder(stackIntake[SPIKE])
                                    .strafeToLinearHeading(crossFieldAlignment[SPIKE].position, crossFieldAlignment[SPIKE].heading)
                                    .build(),

                            new MecanumDrive.DrivePoseLoggingAction(drive, "cross_field_alignment", true),

                            intake.prepareTeleOpsIntake()

                    )
            );

        }

//        sched.addAction(
//                new SequentialAction(
//                        new MecanumDrive.DrivePoseLoggingAction(drive, "backdrop_alignment_position"),
//
//                        new ParallelAction(
//                                drive.actionBuilder(backdropAlignment)
//                                        .setReversed(true)
//                                        .strafeTo(backdrop[SPIKE].position)
//                                        .build(),
//
//                                new SequentialAction(
//                                        new SleepAction(0.2),
//                                        outtake.prepareToSlide(),
//                                        new SleepAction(0.5),
//                                        outtake.extendOuttakeCycle()
//                                )
//                        ),
//
//                        new MecanumDrive.DrivePoseLoggingAction(drive, "backdrop_score_position"),
//
//                        outtake.prepareToScoreCycle(),
//                        new SleepAction(0.5),
//                        outtake.latchScore1(),
//                        new SleepAction(0.5),
//
//                        drive.actionBuilder(backdrop[SPIKE])
//                                .setReversed(true)
//                                .strafeTo(cycleScore[SPIKE].position)
//                                .build(),
//
//                        outtake.latchScore2(),
//                        new SleepAction(0.5),
//
//                        new MecanumDrive.DrivePoseLoggingAction(drive, "backdrop_score_end"),
//                        outtake.retractOuttake(),
//                        new SleepAction(0.25),
//
//                        new MecanumDrive.DrivePoseLoggingAction(drive, "backdrop_complete")
//                )
//        );
    }

    @Override
    public FieldPosition getFieldPosition() {
        return FieldPosition.FAR;
    }

}
