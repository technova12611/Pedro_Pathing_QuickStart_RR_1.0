package org.firstinspires.ftc.teamcode.pedroPathing.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathCallback;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.PoseMessage;

/**
 * This is the StraightBackAndForth autonomous OpMode. It runs the robot in a specified distance
 * straight forward. On reaching the end of the forward Path, the robot runs the backward Path the
 * same distance back to the start. Rinse and repeat! This is good for testing a variety of Vectors,
 * like the drive Vector, the translational Vector, and the heading Vector. Remember to test your
 * tunings on CurvedBackAndForth as well, since tunings that work well for straight lines might
 * have issues going in curves.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/12/2024
 */
@Config
@Autonomous (name = "Straight Back And Forth", group = "Autonomous Pathing Tuning")
public class StraightBackAndForth extends OpMode {
    private Telemetry telemetryA;

    public static double DISTANCE = 30;

    private boolean forward = true;

    private Follower follower;

    private Path forwards;

    PathChain forwardChain;
    private Path backwards;
    private PathChain backwardsChain;

    /**
     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
     * initializes the FTC Dashboard telemetry.
     */
    @Override
    public void init() {
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(8,-63, Math.toRadians(90.0)));

        forwards = new Path(new BezierLine(new Point(8,-63, Point.CARTESIAN), new Point(8,-63+DISTANCE, Point.CARTESIAN)));
        //forwards.setLinearHeadingInterpolation(0, Math.toRadians(80));
        forwards.setConstantHeadingInterpolation(Math.toRadians(90.0));

        forwards.setZeroPowerAccelerationMultiplier(3.5);

        backwards = new Path(new BezierLine(new Point(8,-63+DISTANCE, Point.CARTESIAN), new Point(8,-63, Point.CARTESIAN)));
        backwards.setConstantHeadingInterpolation(Math.toRadians(90.0));

        backwardsChain = new PathChain(backwards);

        backwardsChain.setCallbacks(new PathCallback(0.7, () -> {
            follower.setMaxPower(0.5);
        }, PathCallback.PARAMETRIC, 0));

        backwards.setZeroPowerAccelerationMultiplier(3);

        forwardChain = new PathChain(forwards);

//        forwardChain.setCallbacks();
//        forwardChain.setCallbacks(new PathCallback(0.5, () -> {
//            follower.setMaxPower(0.6);
//        }, PathCallback.PARAMETRIC, 0));

        follower.followPath(forwardChain);

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("This will run the robot in a straight line going " + DISTANCE
                            + " inches forward. The robot will go forward and backward continuously"
                            + " along the path. Make sure you have enough room.");
        telemetryA.update();
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the FTC Dashboard.
     */
    @Override
    public void loop() {
        follower.update();
        if (!follower.isBusy()) {
            if (forward) {
                forward = false;
                //follower.followPath(backwardsChain);
            } else {
                forward = true;
                //follower.followPath(forwardChain);
            }
        }

        telemetryA.addData("going forward", forward);
        telemetryA.addData("Pose: ", new PoseMessage(follower.getPose()));
        follower.telemetryDebug(telemetryA);
    }
}
