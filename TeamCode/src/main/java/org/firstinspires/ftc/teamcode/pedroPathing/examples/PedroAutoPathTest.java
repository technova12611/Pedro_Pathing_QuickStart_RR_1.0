package org.firstinspires.ftc.teamcode.pedroPathing.examples;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.FollowPathAction;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.AutoActionScheduler;
import org.firstinspires.ftc.teamcode.pedroPathing.util.DrivePoseLoggingAction;
import org.firstinspires.ftc.teamcode.pedroPathing.util.PoseMessage;

@Config
@Autonomous(name = "Test Blue Pedro",group = "Test")
public final class PedroAutoPathTest extends LinearOpMode {
    public static Pose2d starting = new Pose2d(0, 0, 0);
    protected AutoActionScheduler sched;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        sched = new AutoActionScheduler(this::update, hardwareMap);

        Follower follower = new Follower(hardwareMap);

        Path forwards = new Path(new BezierCurve(new Point(0,0, Point.CARTESIAN),
                new Point(20,24, Point.CARTESIAN),
                new Point(53,24, Point.CARTESIAN)));
        Path backwards = new Path(new BezierCurve(new Point(53,24, Point.CARTESIAN),
                new Point(22,24, Point.CARTESIAN),
                new Point(0,0, Point.CARTESIAN)));

        forwards.setZeroPowerAccelerationMultiplier(4.75);

        backwards.setZeroPowerAccelerationMultiplier(2);
        backwards.setReversed(true);

        PathBuilder builder = new PathBuilder();

        while(!isStarted() && !isStopRequested()) {
            telemetry.addLine("Ready to start!! Blue Pedro Auto Test");
            telemetry.update();
        }

        sched.addAction(
                new SequentialAction(
                        new DrivePoseLoggingAction(follower, "forward_path_begin", true),
                        new FollowPathAction(follower, forwards, false),
                        new DrivePoseLoggingAction(follower, "forward_path_end"),
                        new SleepAction(2.0),
                        new DrivePoseLoggingAction(follower, "backward_path_begin"),
                        new FollowPathAction(follower,
                                builder.addPath(backwards).setLinearHeadingInterpolation(0.0,0.0).build(),
                                false),
                        new DrivePoseLoggingAction(follower, "backward_path_end")
                ));
        sched.run();

        while(!isStopRequested()) {
            if(sched.isEmpty()) {
                follower.update();
                telemetry.addData("Auto elapsed time: ", sched.autoRunElapsedTime);
                telemetry.addData("End Pose: ", new PoseMessage(follower.getPose()));
                telemetry.update();
            }
            idle();
        }
    }

    final public void update() {
        telemetry.addData("Time left", 30 - getRuntime());
    }
}
