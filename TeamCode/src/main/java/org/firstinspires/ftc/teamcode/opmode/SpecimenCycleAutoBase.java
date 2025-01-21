package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point.CARTESIAN;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.NullAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.FollowPathAction;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathCallback;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.DrivePoseLoggingAction;
import org.firstinspires.ftc.teamcode.utils.software.ActionUtil;

public abstract class SpecimenCycleAutoBase extends AutoBase {
    //increased number moves towards center
    //
    protected Pose preloadPose = new Pose(8.5, -31.0, Math.toRadians(90));
    protected Pose preloadAfterScorePose = new Pose(preloadPose.getX(),
                                               preloadPose.getY() - 5.0,
                                                     preloadPose.getHeading());
    protected Pose specimenPickupPose = new Pose(35, -52.0, Math.toRadians(90));
    protected Pose[] specimenPickupPose2 = {
            new Pose(50, -63, Math.toRadians(90)),
            new Pose(35, -63.8, Math.toRadians(90)),
            new Pose(35, -63.8, Math.toRadians(90)),
            new Pose(35, -63.8, Math.toRadians(90))
    };

    protected Pose parkingPose = new Pose(50, -57.0, Math.toRadians(90));
    protected Pose sampleScorePose = new Pose(-57.5,-57.5, Math.toRadians(45));

    protected Pose[] sampleDropPoses = {
            new Pose(55.0, -56.0, Math.toRadians(90)),
            new Pose(55.0, -56.0, Math.toRadians(90)),
            new Pose(50.0, -56.0, Math.toRadians(90))
    };
    protected Pose[] sampleIntakePoses = {
            new Pose(48.0, -37.5, Math.toRadians(90)),
            new Pose(55.0, -35.5, Math.toRadians(80)),
            new Pose(57.0, -31.0, Math.toRadians(20))};
    protected double[] sampleLinkagePosition = {
//            Intake.LINKAGE_EXTEND_COLOR_SAMPLE_1,
//            Intake.LINKAGE_EXTEND_COLOR_SAMPLE_2,
//            Intake.LINKAGE_EXTEND_COLOR_SAMPLE_3
        };
    protected Pose[] specimenScorePoses = {
            new Pose(6.0, -32.0, Math.toRadians(90)),
            new Pose(4.0, -32.0, Math.toRadians(90)),
            new Pose(2.0, -31.5, Math.toRadians(90)),
            new Pose(1.0, -31.5, Math.toRadians(90))
    };

    protected double INTAKE_POWER_FOR_AUTO = 0.75;
    private static final double FULL_SPEED = 0.88;
    private static final double FULL_SPEED_PLUS = 0.95;
    private static final double MEDIUM_SPEED = 0.70;
    private static final double SLOW_SPEED = 0.5;

    Follower follower;

    @Override
    protected void onRun() {
        //follower = robot.driveTrain.follower;
        assert follower != null;

        // score the color preload
        sched.addAction(scorePreloadSpecimen());
        sched.run();


        // move the 3 color samples to the HP zone
        int sampleCounter = 3;
        for (int count=0; count < sampleCounter; count++) {
            sched.addAction(driveToIntake(count));
            sched.run();
            sched.addAction(intakeAndDropSamples(count));
            sched.run();
        }

        sched.addAction(
            new SequentialAction(
                new SleepAction(0.1)
//                robot.intake.resetIntakeInAuto(),
//                new ActionUtil.RunnableAction(() -> {
//                    robot.outtake.moveSlides(Outtake.VERT_SLIDE_WALL);
//                    return false;
//                })
            )
        );
        sched.run();

        sched.addAction(driveToPickupSpecimen());
        sched.run();

        int specimenCounter = 4;
        for (int count=0; count <specimenCounter; count++) {
            if(getRuntime() < 28.0) {
                sched.addAction(scoreSpecimen(count));
                sched.run();
            }

            if(count < 3) {
                sched.addAction(driveToPickupSpecimenCurve(count + 1));
                sched.run();
            }
        }

        if (getRuntime() < 25.5) {
            sched.addAction(driveToPickupSpecimenCurve(0));
            sched.run();
            sched.addAction(scoreSample());
            sched.run();
        } else {
            sched.addAction(driveToPark());
            sched.run();
        }

        setEndingPose();
        sched.addAction(new SleepAction(20.0));
        sched.run();
    }

    /**
     * Score the preload sample action
     *
     * @return Action - move the robot to score the preload sample
     */
    private Action scorePreloadSpecimen() {
        follower.setMaxPower(FULL_SPEED_PLUS);

        Path preloadPath = new Path(
            new BezierLine(new Point(getStartPose()), new Point(preloadPose)));
        preloadPath.setConstantHeadingInterpolation(Math.toRadians(90));
        preloadPath.setPathEndTValueConstraint(0.96);

        PathChain preloadPathChain = new PathChain(preloadPath);
        preloadPathChain.setCallbacks(
                new PathCallback(0.8, () -> {follower.setMaxPower(FULL_SPEED);}, PathCallback.PARAMETRIC, 0)
        );

        return new SequentialAction(

            new DrivePoseLoggingAction(follower, "preload_score_begin", true),
//            robot.outtake.prepareToScorePreloadSpecimen(),

            new DrivePoseLoggingAction(follower, "preload_path_begin"),
            new FollowPathAction(follower, preloadPathChain),
            new DrivePoseLoggingAction(follower, "preload_path_end")
        );
    }

    /**
     * Drive to the intake sample location
     * 3 samples, 3 cycles.
     *
     * First sample has a completely different path, it moves from the preload pose to the
     * intake pose
     *
     * @param cycle - which cycle
     * @return Action
     */
    private Action driveToIntake(int cycle) {

        follower.setMaxPower(FULL_SPEED_PLUS);
        Pose intakePose = sampleIntakePoses[cycle];

        Pose startPose = follower.getPose();
        if(cycle == 0) {
            startPose = preloadAfterScorePose;
        }
        Path sampleIntakePath = new Path(
                new BezierLine(new Point(startPose),new Point(intakePose)));
        sampleIntakePath.setLinearHeadingInterpolation(startPose.getHeading(),
                intakePose.getHeading());
        // sampleIntakePath.setPathEndTValueConstraint(0.985);

        if(cycle == 0) {
            Path preloadAfterScorePath = new Path(
                    new BezierLine(new Point(preloadPose), new Point(preloadAfterScorePose)));

            preloadAfterScorePath.setConstantHeadingInterpolation(preloadAfterScorePose.getHeading());
            PathChain preloadAfterScoreChain = new PathChain(preloadAfterScorePath, sampleIntakePath);

            // set up intake on the second path when the robot is strafing to the right
            //
            preloadAfterScoreChain.setCallbacks(
                    new PathCallback(0.1, () -> {
                        follower.setMaxPower(MEDIUM_SPEED);
                    }, PathCallback.PARAMETRIC, 0),
                    new PathCallback(0.0, () -> {
                        follower.setMaxPower(FULL_SPEED_PLUS);
                    }, PathCallback.PARAMETRIC, 1)
            );

            preloadAfterScorePath.setPathEndTValueConstraint(0.8);
            sampleIntakePath.setPathEndTValueConstraint(0.98);

            return new ParallelAction(
                //robot.outtake.openClawAction(),
                new SequentialAction(
                        new SleepAction(0.2),
                        new DrivePoseLoggingAction(follower, "intake_cycle_0_begin"),
                        new FollowPathAction(follower, preloadAfterScoreChain),
                        new DrivePoseLoggingAction(follower, "intake_cycle_0_end")),

                new SequentialAction(
                    new SleepAction(0.5)
                    //robot.outtake.pickupSpecimenFromWall(true)
            ));
        }

        return new ParallelAction(
            new SequentialAction(
                    new ActionUtil.RunnableAction(() -> {
                        //robot.intake.bucketDown();
                        return false;
                    })
            ),
            new SequentialAction(
                new DrivePoseLoggingAction(follower, "intake_sample_path_begin_"+cycle),
                new FollowPathAction(follower, sampleIntakePath),
                new DrivePoseLoggingAction(follower, "intake_sample_path_end_"+cycle)
            )
        );
    }

    /**
     * Action to move the robot to align with the sample
     *
     * @param cycle - which sample to pick up
     * @return Action - drive to sample
     */
    private Action intakeAndDropSamples(int cycle) {
        Pose startPose = follower.getPose();
        follower.setMaxPower(FULL_SPEED_PLUS);

        double linkageExtendPosition = sampleLinkagePosition[cycle];

        Path sampleDropPath = new Path(
                new BezierLine(new Point(startPose),new Point(sampleDropPoses[cycle])));
        sampleDropPath.setLinearHeadingInterpolation(startPose.getHeading(),
                sampleDropPoses[cycle].getHeading());
        PathChain sampleDropPathChain = new PathChain(sampleDropPath);

        double temp = 0.1;
        if (cycle == 2){
            temp = 0.4;
        }
        sampleDropPathChain.setCallbacks(
                new PathCallback(temp, () -> {
                    //robot.intake.bucketDump();
                }, PathCallback.PARAMETRIC, 0)
        );

        Action delayAction = new  SleepAction(0.1);

        if(cycle == 0) {
            delayAction = new SleepAction(0.2);
        }
        return new SequentialAction(

                new DrivePoseLoggingAction(follower, "intake_samples_begin_" + cycle),
                new ActionUtil.RunnableAction(() -> {
                    //robot.intake.spin(INTAKE_POWER_FOR_AUTO);
                    return false;
                }),

                // h-slide extends out
                //robot.intake.prepareToIntakeSampleInAutoForSpecimen(linkageExtendPosition),

                // wait until the sample is in the bucket
                // check color sensor
                //new WaitIntakeSampleAction(1000.0),

                new DrivePoseLoggingAction(follower, "intake_sample_end_"+cycle),

                new ParallelAction(
                    new SequentialAction(
                        //robot.intake.moveLinkageAction(Intake.LINKAGE_RETRACT_COLOR_SAMPLE_DROP)
                    ),

                    new SequentialAction(
                        delayAction,
                        new DrivePoseLoggingAction(follower, "drop_sample_path_begin_"+cycle),
                        new FollowPathAction(follower, sampleDropPathChain),
                        new DrivePoseLoggingAction(follower, "drop_sample_path_end_"+cycle)
                    )
                ),

                new DrivePoseLoggingAction(follower, "drop_sample_end_"+cycle)
        );
    }

    /**
     *  this is used for the first pick up after the intake samples
     *
     * @return Action -
     */
    private Action driveToPickupSpecimen() {
        follower.setMaxPower(SLOW_SPEED);
        Path specimenPickupPath = new Path(
                new BezierLine(new Point(follower.getPose()),new Point(specimenPickupPose2[0])));
        specimenPickupPath.setLinearHeadingInterpolation(follower.getPose().getHeading(),
                specimenPickupPose2[0].getHeading());

        return new SequentialAction(
                new DrivePoseLoggingAction(follower, "pickup_specimen_path_begin_0"),
                new FollowPathAction(follower, specimenPickupPath),
                new DrivePoseLoggingAction(follower, "pickup_specimen_path_end_0")
        );
    }

    private Action driveToPark() {
        follower.setMaxPower(FULL_SPEED_PLUS);
        Path parkPath = new Path(
                new BezierLine(new Point(follower.getPose()),new Point(parkingPose)));
        parkPath.setLinearHeadingInterpolation(follower.getPose().getHeading(),
                parkingPose.getHeading());

        Action pathAction = new FollowPathAction(follower, parkPath);

        if(follower.getPose().getY() < parkingPose.getY()) {
            pathAction = new NullAction();
        }

        // chain the two paths together
        // full speed for the first path
        // slowdown for the second path

        return new ParallelAction(
                new SequentialAction(
                        new SleepAction(0.4)
                        // reset outtake
                        //robot.outtake.resetOuttakeAfterSpecimenAuto(),
                        // move the h-slide back, in case it's not
                        //robot.intake.retractAction()
                ),

                new SequentialAction(
                        new DrivePoseLoggingAction(follower, "parking_path_begin"),
                        pathAction,
                        new DrivePoseLoggingAction(follower, "parking_path_end"),
                        // move the intake bucket in the init position
                        //robot.intake.bucketHalfAction(),
                        new DrivePoseLoggingAction(follower, "parking_end")
                )
        );
    }

    /**
     *  Drive to score the specimen
     *
     * @param cycle - cycle count
     * @return Action
     */
    private Action scoreSpecimen(int cycle) {
        follower.setMaxPower(FULL_SPEED_PLUS);
        Pose stepEnd = specimenScorePoses[cycle];
        Pose step1 = new Pose(stepEnd.getX()+3, stepEnd.getY()-17, stepEnd.getHeading());
        Path specimenScorePath = new Path(
                new BezierLine(new Point(follower.getPose()),
                        //new Point(1.5, -64,Point.CARTESIAN),
                        //new Point(11.5, -67.5,Point.CARTESIAN),
                        new Point(step1)));
        specimenScorePath.setConstantHeadingInterpolation(stepEnd.getHeading());
        specimenScorePath.setPathEndTValueConstraint(0.98);

        Path specimenScorePath2 = new Path(
                new BezierLine(new Point(step1), new Point(stepEnd)));
        specimenScorePath2.setConstantHeadingInterpolation(stepEnd.getHeading());
        specimenScorePath2.setPathEndTValueConstraint(0.97);
        specimenScorePath2.setPathEndTimeoutConstraint(300);

        PathChain specimenScoreChain = new PathChain(specimenScorePath, specimenScorePath2);

        // slow down for the second path when approaching the chamber
        //------------------------
        specimenScoreChain.setCallbacks(
                new PathCallback(0.85, () -> {follower.setMaxPower(FULL_SPEED);}, PathCallback.PARAMETRIC, 1));

        return new SequentialAction(
                new DrivePoseLoggingAction(follower, "scoreSpecimen_begin_"+cycle),
                new ParallelAction(

                    // prepare to score Specimen while robot moves
                    //robot.outtake.prepareToScoreSpecimenAuto(),

                    new SequentialAction(
                        new SleepAction(0.1),
                        new DrivePoseLoggingAction(follower, "specimen_score_path_begin_"+cycle),
                        new FollowPathAction(follower, specimenScoreChain),
                        new DrivePoseLoggingAction(follower, "specimen_score_path_end_"+cycle)

                        //robot.outtake.openClawAction()
                    ),

                    new DrivePoseLoggingAction(follower, "scoreSpecimen_end_"+cycle)
                )
        );
    }

    /**
     * Drive to the specimen pickup position from the chamber
     *
     * This path uses the Bezier curve, and use t-value to control the velocity of the drive
     * when the robot is approaching the specimen on the wall, it slows down
     *
     * @param cycle - cycle count
     * @return Action
     */
    private Action driveToPickupSpecimenCurve(int cycle) {
        follower.setMaxPower(FULL_SPEED_PLUS);

        // change the control point 1, if the claw is hitting the submersible
        // reduce x and increase y
        Path specimenPickupPath = new Path(
                new BezierCurve(new Point(follower.getPose()),
                        new Point(11, -54, CARTESIAN),
                        new Point(32, -43, CARTESIAN),
                        new Point(specimenPickupPose2[cycle])));
        specimenPickupPath.setConstantHeadingInterpolation(specimenPickupPose2[cycle].getHeading());
        PathChain specimenPickupChain = new PathChain(specimenPickupPath);

        specimenPickupChain.setCallbacks(
                new PathCallback(0.70, () -> {follower.setMaxPower(MEDIUM_SPEED);}, PathCallback.PARAMETRIC, 0),
                new PathCallback(0.85, () -> {follower.setMaxPower(SLOW_SPEED);}, PathCallback.PARAMETRIC, 0));

        return new ParallelAction(

                // tune the delay, if the claw is hitting the submersible frame
                new SequentialAction(
                        new SleepAction(0.3),
                        new DrivePoseLoggingAction(follower, "pickup_specimen_begin_"+cycle),
                        //robot.outtake.pickupSpecimenFromWall(false),
                        new DrivePoseLoggingAction(follower, "pickup_specimen_end_"+cycle)
                ),

                new SequentialAction(
                    new DrivePoseLoggingAction(follower, "specimen_pickup_path_begin_"+cycle),
                    new FollowPathAction(follower, specimenPickupChain),
                    new DrivePoseLoggingAction(follower, "specimen_pickup_path_end_"+cycle)
                )
        );
    }
    /**
     *  Drive to score the sample
     *
     * @return Action
     */
    private Action scoreSample() {
        follower.setMaxPower(FULL_SPEED_PLUS);
        Path sampleScorePath = new Path(
                new BezierCurve(new Point(follower.getPose()),
                        new Point(0.5, -29,Point.CARTESIAN),
                        new Point(sampleScorePose)));
        sampleScorePath.setLinearHeadingInterpolation(follower.getPose().getHeading(),sampleScorePose.getHeading());
        sampleScorePath.setPathEndTValueConstraint(0.97);
        sampleScorePath.setPathEndTimeoutConstraint(300);

        return new SequentialAction(
                new DrivePoseLoggingAction(follower, "scoreSample_begin"),
                new ParallelAction(
//                        robot.outtake.closeClawAction(),
//                        // prepare to score Specimen while robot moves
//                        new SequentialAction(
//                            new SleepAction(2.0),
//                            robot.outtake.prepareToScoreSample()
//                        ),
//                        robot.outtake.prepareToScoreSpecimenAuto(),

                        new FollowPathAction(follower, sampleScorePath)

                ),
                new SleepAction(0.1),
                //robot.outtake.afterSampleScored(),
                new DrivePoseLoggingAction(follower, "scoreSample_end")
        );
    }
}
