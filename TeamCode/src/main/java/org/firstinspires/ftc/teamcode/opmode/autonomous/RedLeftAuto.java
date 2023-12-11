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
@Autonomous(name = "RED Left Auto", group = "Auto", preselectTeleOp = "Manual Drive")
public class RedLeftAuto extends AutoBase {
    public static Pose2d[] spike = {
            new Pose2d(-48.5, -45.5, Math.toRadians(180)),
            new Pose2d(-34.0, -37.5, Math.toRadians(180)),
            new Pose2d(-36.0, -36.5, Math.toRadians(30))
    };
    public static Pose2d[] backdrop = {
            new Pose2d(49.2, -29, Math.toRadians(180)),
            new Pose2d(49.2, -36, Math.toRadians(180)),
            new Pose2d(49.2, -42, Math.toRadians(180))
    };
    // 0 = left, 1 = middle, 2 = right
    public static Pose2d start = new Pose2d(-40.0, -62.0, Math.toRadians(90));
    public static Pose2d parking = new Pose2d(45.0, -20.0, Math.toRadians(180));

    public static Pose2d cycleScore[] = {
            new Pose2d(49.7, -39.0, Math.toRadians(180)),
            new Pose2d(49.7, -32.0, Math.toRadians(180)),
            new Pose2d(49.7, -32.0, Math.toRadians(180))
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

        Pose2d backOffFromSpike = new Pose2d(-40, -48, Math.toRadians(180));
        Pose2d stackIntakeAlignment = new Pose2d(-50, -25, Math.toRadians(180));
        Pose2d stackIntake = new Pose2d(-53, -25, Math.toRadians(180));
        Pose2d crossFieldAligment = new Pose2d(-48, -12, Math.toRadians(180));
        Pose2d backdropAlignment = new Pose2d(45.0, -10.5, Math.toRadians(180));

        // SPIKE is right
        if (SPIKE == 2) {
            sched.addAction(
                    new SequentialAction(
                            new ParallelAction(
                                    drive.actionBuilder(drive.pose)
                                            .setTangent(0)
                                            .splineTo(spike[SPIKE].position, spike[SPIKE].heading)
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
                                            .splineToSplineHeading(backOffFromSpike, Math.toRadians(0))
                                            .build(),

                                    new SequentialAction(
                                            new SleepAction(0.25),
                                            intake.prepareTeleOpsIntake(),
                                            outtake.prepareToTransfer()
                                    ),

                                    // move to middle stack on the left
                                    drive.actionBuilder(backOffFromSpike)
                                            .strafeTo(stackIntakeAlignment.position)
                                            .build(),

                                    intake.intakeOneStackedPixels(),

                                    // move to stack intake position
                                    drive.actionBuilder(stackIntakeAlignment)
                                            .strafeTo(stackIntake.position)
                                            .build(),

                                    // move to stack intake position
                                    drive.actionBuilder(stackIntake)
                                            .strafeTo(crossFieldAligment.position)
                                            .build()
                            )
                    )
            );

        }
        // SPIKE is center
        else if (SPIKE == 1) {

        }
        // SPIKE is lef
        else {

        }

        sched.addAction(
                new SequentialAction(
                        new MecanumDrive.DrivePoseLoggingAction(drive, "backdrop_alignment_position"),

                        new ParallelAction(
                                drive.actionBuilder(backdropAlignment)
                                        .setReversed(true)
                                        .strafeTo(backdrop[SPIKE].position)
                                        .build(),

                                new SequentialAction(
                                        new SleepAction(0.2),
                                        outtake.prepareToSlide(),
                                        new SleepAction(0.5),
                                        outtake.extendOuttakeCycle()
                                )
                        ),

                        new MecanumDrive.DrivePoseLoggingAction(drive, "backdrop_score_position"),

                        outtake.prepareToScoreCycle(),
                        new SleepAction(0.5),
                        outtake.latchScore1(),
                        new SleepAction(0.5),

                        drive.actionBuilder(backdrop[SPIKE])
                                .setReversed(true)
                                .strafeTo(cycleScore[SPIKE].position)
                                .build(),

                        outtake.latchScore2(),
                        new SleepAction(0.5),

                        new MecanumDrive.DrivePoseLoggingAction(drive, "backdrop_score_end"),
                        outtake.retractOuttake(),
                        new SleepAction(0.25),

                        new MecanumDrive.DrivePoseLoggingAction(drive, "backdrop_complete")
                )
        );
    }

    @Override
    public FieldPosition getFieldPosition() {
        return FieldPosition.FAR;
    }

}
