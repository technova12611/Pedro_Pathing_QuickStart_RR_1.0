package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.pipeline.AlliancePosition;
import org.firstinspires.ftc.teamcode.pipeline.FieldPosition;

@Config
@Autonomous(name = "RED LEFT Auto (2+1)", group = "RED Auto", preselectTeleOp = "Manual Drive")
public class RedLeftAuto extends AutoBase {
    // 0 = left, 1 = middle, 2 = right
    public static Pose2d start = new Pose2d(-40.0, -62.0, Math.toRadians(90));

    public static Pose2d[] spike = {
            new Pose2d(-48.5, -45.0, Math.toRadians(90)),
            new Pose2d(-43.0, -35.5, Math.toRadians(90)),
            new Pose2d(-38.0, -35.5, Math.toRadians(25))
    };
    public static Pose2d[] backdrop = {
            new Pose2d(49.2, -29, Math.toRadians(180)),
            new Pose2d(49.2, -36, Math.toRadians(180)),
            new Pose2d(49.2, -42, Math.toRadians(180))
    };

    public static Pose2d parking = new Pose2d(45.0, -20.0, Math.toRadians(180));

    public static Pose2d cycleScore[] = {
            new Pose2d(49.0, -39.0, Math.toRadians(180)),
            new Pose2d(49.0, -32.0, Math.toRadians(180)),
            new Pose2d(49.0, -32.0, Math.toRadians(180))
    };

    protected AlliancePosition getAlliance() {
        return AlliancePosition.RED;
    }

    @Override
    protected Pose2d getStartPose() {
        return start;
    }

    @Override
    protected void printDescription() {
        telemetry.addData("Description", "RED Left Auto");
    }

    @Override
    protected void onRun() {

        Pose2d[] backOffFromSpike = {
                new Pose2d(-35, -48, Math.toRadians(90)),
                new Pose2d(-40, -42, Math.toRadians(90)),
                new Pose2d(-40, -45, Math.toRadians(90))
        };

        Pose2d[] stackIntakeAlignment = {
                new Pose2d(-36, -12.05, Math.toRadians(90)),
                new Pose2d(-50, -34.5, Math.toRadians(180)),
                new Pose2d(-50, -12.05, Math.toRadians(180))
        };

        Pose2d[] stackIntake = {
                new Pose2d(-58, -12.75, Math.toRadians(180)),
                new Pose2d(-56.5, -35, Math.toRadians(180)),
                new Pose2d(-58.5, -12.75, Math.toRadians(180))
        };
        Pose2d [] crossFieldAlignment = {
                new Pose2d(-48, -11, Math.toRadians(180)),
                new Pose2d(-40, -35, Math.toRadians(180)),
                new Pose2d(-48, -11, Math.toRadians(180))
        };
        Pose2d backdropAlignment = new Pose2d(45.0, -10.5, Math.toRadians(180));

        // SPIKE is right
        if (SPIKE == 2) {
            Pose2d moveUp1 =  new Pose2d(-40.0, -54.0, Math.toRadians(90));
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

                            // move to stack intake position
                            drive.actionBuilder(stackIntake[SPIKE])
                                    .strafeTo(crossFieldAlignment[SPIKE].position)
                                    .build()
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
                            new SleepAction(0.5),
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
                            new MecanumDrive.DrivePoseLoggingAction(drive, "cross_field_alignment"),

                            intake.prepareTeleOpsIntake()

                    )
            );

        }
        // SPIKE is lef
        else {
            Pose2d stackIntakeAlignment2 = new Pose2d(-50, -12.75, Math.toRadians(180));
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
