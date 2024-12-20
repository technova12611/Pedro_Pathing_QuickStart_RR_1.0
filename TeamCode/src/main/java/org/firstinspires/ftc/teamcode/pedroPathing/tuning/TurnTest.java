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


@Config
@Disabled
@Autonomous (name = "Turn Test", group = "Autonomous Pathing Tuning")
public class TurnTest extends OpMode {
    private Telemetry telemetryA;

    public static double HEADING = 30;

    private Follower follower;

    private Path turnPath;

    /**
     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
     * initializes the FTC Dashboard telemetry.
     */
    @Override
    public void init() {
        follower = new Follower(hardwareMap);

        Point startingPoint = new Point(0,0, Point.CARTESIAN);
        Point endingPoint = new Point(10,0, Point.CARTESIAN);
        //follower.setStartingPose(new Pose(0,0, Math.toRadians(0.0)));

        turnPath = new Path(new BezierLine(startingPoint, endingPoint));
        turnPath.setLinearHeadingInterpolation(0, Math.toRadians(HEADING));
        turnPath.setZeroPowerAccelerationMultiplier(3.5);

        follower.followPath(turnPath);

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("This will turn the robot going " + HEADING
                            + " degree counter clock wise. Make sure you have enough room.");
        telemetryA.update();
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the FTC Dashboard.
     */
    @Override
    public void loop() {
        follower.update();
        follower.telemetryDebug(telemetryA);
    }
}
