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
@Autonomous(name = "RED Left Auto", group = "RED Auto", preselectTeleOp = "Manual Drive")
public class RedLeftAuto extends AutoBase {
    // 0 = left, 1 = middle, 2 = right
    public static Pose2d start = new Pose2d(-40.0, -62.0, Math.toRadians(90));

    public static Pose2d[] spike = {
            new Pose2d(-48.5, -45.5, Math.toRadians(90)),
            new Pose2d(-34.0, -37.5, Math.toRadians(90)),
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
                new Pose2d(-40, -48, Math.toRadians(90)),
                new Pose2d(-36, -40, Math.toRadians(90)),
                new Pose2d(-40, -45, Math.toRadians(90))
        };

        Pose2d backOffFromSpike2 = new Pose2d(-36, -45, Math.toRadians(90));

        Pose2d[] stackIntakeAlignment = {
                new Pose2d(-50, -12.75, Math.toRadians(180)),
                new Pose2d(-50, -38, Math.toRadians(180)),
                new Pose2d(-50, -12.75, Math.toRadians(180))
        };

        Pose2d[] stackIntake = {
                new Pose2d(-54, -12.75, Math.toRadians(180)),
                new Pose2d(-54, -38, Math.toRadians(180)),
                new Pose2d(-58.5, -12.75, Math.toRadians(180))
        };
        Pose2d [] crossFieldAligment = {
                new Pose2d(-48, -12, Math.toRadians(180)),
                new Pose2d(-48, -12, Math.toRadians(180)),
                new Pose2d(-48, -12, Math.toRadians(180))
        };
        Pose2d backdropAlignment = new Pose2d(45.0, -10.5, Math.toRadians(180));

        // SPIKE is right
        if (SPIKE == 2) {
            Pose2d moveUp1 =  new Pose2d(-40.0, -54.0, Math.toRadians(90));
            sched.addAction(
                    new SequentialAction(
                            drive.actionBuilder(drive.pose)
                                    .strafeTo(moveUp1.position)
                                    .build(),

                            new ParallelAction(
                                    drive.actionBuilder(moveUp1)
                                            .strafeToLinearHeading(spike[SPIKE].position, spike[SPIKE].heading)
                                            .build(),
                                    new SequentialAction(
                                            intake.stackIntakeLinkageDown()
                                    )
                            ),

                            // drop the purple pixel
                            intake.scorePurplePreload(),
                            new SleepAction(0.25),

                            intake.prepareTeleOpsIntake(),
                            new SleepAction(0.25),

                            drive.actionBuilder(spike[SPIKE])
                                    .strafeToLinearHeading(backOffFromSpike[SPIKE].position, backOffFromSpike[SPIKE].heading)
                                    .build(),

                            // move to middle stack on the left
                            new ParallelAction(
                                    drive.actionBuilder(backOffFromSpike[SPIKE])
                                            .strafeToLinearHeading(stackIntakeAlignment[SPIKE].position, stackIntakeAlignment[SPIKE].heading)
                                            .build(),
                                    intake.prepareStackIntake()
                            ),


                            // move to stack intake position
                            drive.actionBuilder(stackIntakeAlignment[SPIKE])
                                    .strafeTo(stackIntake[SPIKE].position)
                                    .build(),

                            new MecanumDrive.DrivePoseLoggingAction(drive, "stack_intake_position"),
                            intake.intakeOneStackedPixels(),

                            // move to stack intake position
                            drive.actionBuilder(stackIntake[SPIKE])
                                    .strafeTo(crossFieldAligment[SPIKE].position)
                                    .build()
                    )
            );

        }
        // SPIKE is center
        else if (SPIKE == 1) {

            sched.addAction(
                    new SequentialAction(
                            new ParallelAction(
                                    drive.actionBuilder(drive.pose)
                                            .strafeTo(spike[SPIKE].position)
                                            .build(),
                                    new SequentialAction(
                                            new SleepAction(0.5),
                                            intake.stackIntakeLinkageDown()
                                    )
                            ),
                            // drop the purple pixel
                            intake.scorePurplePreload(),
                            new SleepAction(0.25),

                            // backoff from spike
                            //
                            new ParallelAction(
                                    drive.actionBuilder(spike[SPIKE])
                                            .setReversed(true)
                                            .splineToSplineHeading(backOffFromSpike[SPIKE], Math.toRadians(0))
                                            .build(),

                                    new SequentialAction(
                                            new SleepAction(0.25),
                                            intake.prepareTeleOpsIntake(),
                                            outtake.prepareToTransfer()
                                    )),

                            drive.actionBuilder(backOffFromSpike[SPIKE])
                                    .turnTo(Math.toRadians(180))
                                    .build(),

                            new ParallelAction(
                                    drive.actionBuilder(new Pose2d(backOffFromSpike[SPIKE].position, Math.toRadians(180)))
                                            .strafeTo(stackIntakeAlignment[SPIKE].position)
                                            .build(),
                                    intake.prepareStackIntake()
                            ),

                            // move to stack intake position
                            drive.actionBuilder(stackIntakeAlignment[SPIKE])
                                    .strafeTo(stackIntake[SPIKE].position)
                                    .build(),

                            intake.intakeOneStackedPixels(),

                            // move to stack intake position
                            drive.actionBuilder(stackIntake[SPIKE])
                                    .strafeTo(crossFieldAligment[SPIKE].position)
                                    .build()

                    )
            );

        }
        // SPIKE is lef
        else {
            sched.addAction(
                    new SequentialAction(
                            new ParallelAction(
                                    drive.actionBuilder(drive.pose)
                                            .strafeTo(spike[SPIKE].position)
                                            .build(),
                                    new SequentialAction(
                                            new SleepAction(0.5),
                                            intake.stackIntakeLinkageDown()
                                    )
                            ),
                            // drop the purple pixel
                            intake.scorePurplePreload(),
                            new SleepAction(0.25),

                            // backoff from spike
                            //
                            new ParallelAction(
                                    drive.actionBuilder(spike[SPIKE])
                                            .setReversed(true)
                                            .splineToSplineHeading(backOffFromSpike[SPIKE], Math.toRadians(0))
                                            .build(),

                                    new SequentialAction(
                                            new SleepAction(0.25),
                                            intake.prepareTeleOpsIntake(),
                                            outtake.prepareToTransfer()
                                    )),

                            drive.actionBuilder(backOffFromSpike[SPIKE])
                                    .strafeTo(backOffFromSpike2.position)
                                    .build(),

                            drive.actionBuilder(backOffFromSpike2)
                                    .turnTo(Math.toRadians(180))
                                    .build(),

                            new ParallelAction(
                                    drive.actionBuilder(new Pose2d(backOffFromSpike2.position, Math.toRadians(180)))
                                            .strafeTo(stackIntakeAlignment[SPIKE].position)
                                            .build(),

                                    intake.prepareStackIntake()
                            ),

                            // move to stack intake position
                            drive.actionBuilder(stackIntakeAlignment[SPIKE])
                                    .strafeTo(stackIntake[SPIKE].position)
                                    .build(),

                            intake.intakeOneStackedPixels(),

                            // move to stack intake position
                            drive.actionBuilder(stackIntake[SPIKE])
                                    .strafeTo(crossFieldAligment[SPIKE].position)
                                    .build()

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
