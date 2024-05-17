package org.firstinspires.ftc.teamcode.pedroPathing.tuning;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.PoseMessage;

@Autonomous (name = "Curved Path Test", group = "Autonomous Pathing Tuning")
public class CurvedPathTest extends OpMode {
    private Telemetry telemetryA;

    private Follower follower;

    private Path forwards;

    /**
     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
     * initializes the FTC Dashboard telemetry.
     */
    @Override
    public void init() {
        follower = new Follower(hardwareMap);

        follower.setStartingPose(new Pose2d(0, 0, Math.PI));
        forwards = new Path(new BezierCurve(
                new Point(0,0, Point.CARTESIAN),
                new Point(-20,0, Point.CARTESIAN),
                new Point(-40,0, Point.CARTESIAN),
                new Point(-55, -16, Point.CARTESIAN)));

        forwards.setZeroPowerAccelerationMultiplier(4.75);
        forwards.setLinearHeadingInterpolation(Math.PI, Math.toRadians(210));
        follower.followPath(forwards);

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("This will run the robot in a curve path");
        telemetryA.update();
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the FTC Dashboard.
     */
    @Override
    public void loop() {
        if(follower.isBusy()) {
            follower.update();
        }
        if (!follower.isBusy() && !isDone) {
            try {
                Thread.sleep(2000);
                follower.update();
                Log.d("Petro_logger", "Robot pose: " + new PoseMessage(follower.getPose()));
                isDone = true;
            } catch(Exception e) {
                //
            }
        }
    }

    private boolean isDone = false;
}
