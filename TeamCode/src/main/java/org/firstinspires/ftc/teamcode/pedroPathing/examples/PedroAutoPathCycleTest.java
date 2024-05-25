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
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.AutoActionScheduler;
import org.firstinspires.ftc.teamcode.pedroPathing.util.DrivePoseLoggingAction;
import org.firstinspires.ftc.teamcode.pedroPathing.util.PoseMessage;


@Config
@Autonomous(name = "Blue Pedro Cycle Test",group = "Test")
public final class PedroAutoPathCycleTest extends LinearOpMode {
    public static Pose2d starting = new Pose2d(14.5, -62.0, Math.PI/2);
    public static Pose2d backdrop = new Pose2d(48.5, -36.0, Math.PI);
    public static Pose2d spike = new Pose2d(28.5, -24.5, Math.PI);
    public static Pose2d parking = new Pose2d(52.0, -60.0, Math.PI);

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

        Point backdrop = new Point(48.0,36.0, Point.CARTESIAN);
        Point cycle = new Point(48.0,30.0, Point.CARTESIAN);

        Path purplePath = new Path(
                new BezierLine(new Point(14.5, 62.0),
                        new Point(32.0, 23.0)));

        purplePath.setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(180));
        purplePath.setZeroPowerAccelerationMultiplier(3.75);

        sched.addAction(
                new SequentialAction(
                        new DrivePoseLoggingAction(follower, "purple_path_begin", true),
                        new FollowPathAction(follower, purplePath, false),
                        new DrivePoseLoggingAction(follower, "purple_path_end"),
                        new SleepAction(0.5)
                ));
        sched.run();

        Pose2d currentPose = follower.getPose();

        Path yellowPath = new Path(new BezierLine(new Point(currentPose), backdrop));

        // drop yellow
        yellowPath.setConstantHeadingInterpolation(Math.toRadians(180));
        yellowPath.setZeroPowerAccelerationMultiplier(2.5);
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

        double y_position0 = 11.0;
        double y_position = 12.0;
        while(cycleCount++ < 3) {

            // cycle
            Pose2d backdrop0 = follower.getPose();

            if(cycleCount == 2) {
                y_position0 = 11.5;
                y_position = 12.5;
                follower.setPose(new Pose2d(backdrop0.position.x, backdrop0.position.y-0.75,backdrop0.heading.toDouble()));
            }
            if(cycleCount == 3) {
                y_position0 = 12.0;
                y_position = 13.0;
                follower.setPose(new Pose2d(backdrop0.position.x, backdrop0.position.y-1.5,backdrop0.heading.toDouble()));
            }

            Point stagePoint = new Point(backdrop0);

            Path toStack = new Path(new BezierCurve(stagePoint,
                    new Point(45, y_position0, Point.CARTESIAN),
                    new Point(24, y_position, Point.CARTESIAN),
//                    new Point(-0, y_position, Point.CARTESIAN)));
                    new Point(-24, y_position, Point.CARTESIAN),
                    new Point(-54, y_position, Point.CARTESIAN))
            );

            toStack.setZeroPowerAccelerationMultiplier(4.5);
            toStack.setConstantHeadingInterpolation(Math.PI);

            sched.addAction(
                    new SequentialAction(
                            new DrivePoseLoggingAction(follower, "stack_path_begin"),
                            new FollowPathAction(follower, toStack),
                            new DrivePoseLoggingAction(follower, "stack_path_end"),
                            new SleepAction(1.5)
                    ));
            sched.run();

            Pose2d stack0 = follower.getPose();
            Point stackPoint = new Point(stack0.position.x, stack0.position.y, Point.CARTESIAN);
            Path toStage = new Path(new BezierCurve(stackPoint,
                    new Point(8, 11.0, Point.CARTESIAN),
//                    new Point(20, 12.0, Point.CARTESIAN),
                    new Point(28, 11.0, Point.CARTESIAN),
                    cycle));

            toStage.setReversed(true);
            toStage.setConstantHeadingInterpolation(Math.toRadians(180));
            toStage.setZeroPowerAccelerationMultiplier(2.20);

            // drop yellow
            sched.addAction(
                    new SequentialAction(
                            new DrivePoseLoggingAction(follower, "stage_path_begin"),
                            new FollowPathAction(follower, toStage),
                            new DrivePoseLoggingAction(follower, "stage_path_end"),
                            new SleepAction(1.25),
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
