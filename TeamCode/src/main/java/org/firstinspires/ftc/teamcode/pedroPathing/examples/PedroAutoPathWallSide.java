package org.firstinspires.ftc.teamcode.pedroPathing.examples;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.FollowPathAction;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.AutoActionScheduler;
import org.firstinspires.ftc.teamcode.pedroPathing.util.DrivePoseLoggingAction;
import org.firstinspires.ftc.teamcode.pedroPathing.util.PoseMessage;


@Config
@Autonomous(name = "Blue Pedro Wall side Test",group = "Test")
public final class PedroAutoPathWallSide extends LinearOpMode {

    protected AutoActionScheduler sched;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        sched = new AutoActionScheduler(this::update, hardwareMap);

        while(!isStarted() && !isStopRequested()) {
            telemetry.addLine("Ready to start!! Blue Auto Pedro Pathing Test ...");
            telemetry.update();
        }

        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        Follower follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose2d(14.5, 62, Math.toRadians(-90)));

        Point backdrop = new Point(48.0,32.0, Point.CARTESIAN);
        Point cycle = new Point(48.0,40.0, Point.CARTESIAN);

        Path purplePath = new Path(
                new BezierLine(new Point(14.5, 62.0, Point.CARTESIAN),
                        new Point(32.0, 23.0, Point.CARTESIAN)));

        purplePath.setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(180));
        purplePath.setZeroPowerAccelerationMultiplier(2.25);

        sched.addAction(
                new SequentialAction(
                        new DrivePoseLoggingAction(follower, "purple_path_begin", true),
                        new FollowPathAction(follower, purplePath, false),
                        new DrivePoseLoggingAction(follower, "purple_path_end"),
                        new SleepAction(0.5)
                ));
        sched.run();

        Pose2d currentPose = follower.getPose();
        //follower.setStartingPose(currentPose);

        Path yellowPath = new Path(new BezierLine(new Point(currentPose), backdrop));

        // drop yellow
//        yellowPath.setLinearHeadingInterpolation(currentPose.heading.toDouble(), Math.toRadians(180));
        yellowPath.setConstantHeadingInterpolation(Math.toRadians(180));
        yellowPath.setZeroPowerAccelerationMultiplier(2.75);
        //yellowPath.setReversed(true);

        follower.update();
        sched.addAction(
                new SequentialAction(
                        new DrivePoseLoggingAction(follower, "yellow_path_begin"),
                        new FollowPathAction(follower, yellowPath),
                        new DrivePoseLoggingAction(follower, "yellow_path_end"),
                        new SleepAction(1.25)
                ));
        sched.run();

        int cycleCount = 0;

        double y_position0 = 58.5;
        double y_position = 43.0;

        while(cycleCount++ < 2) {

            // cycle
            Pose2d backdrop0 = follower.getPose();

            if(cycleCount == 2) {
                y_position0 = 59.0;
                //y_position = 47.5;
                follower.setPose(new Pose2d(backdrop0.position.x, backdrop0.position.y-0.5,backdrop0.heading.toDouble()));
            }

            Point stagePoint = new Point(backdrop0);

            Path toStack0 = new Path(new BezierLine(stagePoint,
                    new Point(30, y_position0, Point.CARTESIAN))
                    );
            toStack0.setConstantHeadingInterpolation(Math.PI);
            toStack0.setZeroPowerAccelerationMultiplier(3.75);

            sched.addAction(
                    new SequentialAction(
                            new DrivePoseLoggingAction(follower, "stack0_path_begin"),
                            new FollowPathAction(follower, toStack0),
                            new DrivePoseLoggingAction(follower, "stack0_path_end")
                    ));

            sched.run();

            Path toStack1 = new Path(new BezierCurve(
                    new Point(follower.getPose()),
                    new Point(0, y_position0, Point.CARTESIAN),
                    new Point(-16, y_position0, Point.CARTESIAN)));

            toStack1.setZeroPowerAccelerationMultiplier(3.5);
            toStack1.setConstantHeadingInterpolation(Math.PI);

            Path toStack2 = new Path(new BezierCurve(
                    new Point(-16, y_position0, Point.CARTESIAN),
                    new Point(-38, y_position0, Point.CARTESIAN),
                    new Point(-45, y_position0, Point.CARTESIAN),
                    new Point(-52, y_position, Point.CARTESIAN)));

            toStack2.setZeroPowerAccelerationMultiplier(4.25);
            toStack2.setLinearHeadingInterpolation(Math.PI, Math.toRadians(205));

            PathBuilder builder = new PathBuilder();
            builder.addPath(toStack0).addPath(toStack1).addPath(toStack2);

            sched.addAction(
                    new SequentialAction(
                            new DrivePoseLoggingAction(follower, "stack_path_begin"),
                            new FollowPathAction(follower,  builder.build()),
                            new DrivePoseLoggingAction(follower, "stack_path_end"),
                            new SleepAction(1.75)
                    ));
            sched.run();

            follower.update();
            Pose2d stack0 = follower.getPose();
            Path toStage0 = new Path(new BezierLine( new Point(stack0),
                    new Point(-40, y_position0, Point.CARTESIAN)
                    ));

            toStage0.setReversed(true);
            toStage0.setZeroPowerAccelerationMultiplier(2.75);
            toStage0.setLinearHeadingInterpolation(stack0.heading.toDouble(), Math.PI);

            sched.addAction(
                    new SequentialAction(
                            new DrivePoseLoggingAction(follower, "stage0_path_begin"),
                            new FollowPathAction(follower, toStage0),
                            new DrivePoseLoggingAction(follower, "stage0_path_end")
                    ));
            sched.run();

            follower.update();
            Path toStage1 = new Path(new BezierCurve(new Point(follower.getPose()),
                    new Point(8, y_position0, Point.CARTESIAN),
                    new Point(32,y_position0, Point.CARTESIAN),
                    cycle));

            toStage1.setReversed(true);
            toStage1.setConstantHeadingInterpolation(Math.toRadians(180));
            toStage1.setZeroPowerAccelerationMultiplier(2.25);

            // drop yellow
            sched.addAction(
                    new SequentialAction(
                            new DrivePoseLoggingAction(follower, "stage1_path_begin"),
                            new FollowPathAction(follower, toStage1),
                            new DrivePoseLoggingAction(follower, "stage1_path_end"),
                            new SleepAction(1.75),
                            new DrivePoseLoggingAction(follower, "cycle_end")
                    ));
            sched.run();
        }

        boolean firstTime = true;
        Pose2d endPose = follower.getPose();
        while(!isStopRequested() ) {
            if(sched.isEmpty()) {
                if(firstTime) {
                    follower.update();
                    endPose = follower.getPose();
                    firstTime = false;
                    Log.d("Drive_logger", "End drive pose: " + new PoseMessage(endPose));
                    Log.d("Drive_logger","Auto elapsed time (ms): " + String.format("%3.3f",timer.milliseconds()));
                }

                telemetry.addData("Auto elapsed time (ms): ", String.format("%3.3f",timer.milliseconds()));
                telemetry.addData("End Pose: ", new PoseMessage(endPose).toString());
                telemetry.update();
            }
            idle();
        }
    }

    final public void update() {
        telemetry.addData("Time left", 30 - getRuntime());
    }
}
