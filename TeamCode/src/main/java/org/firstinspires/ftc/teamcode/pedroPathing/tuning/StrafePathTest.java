package org.firstinspires.ftc.teamcode.pedroPathing.tuning;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.PoseMessage;

@Autonomous (name = "Strafe Path Test", group = "Autonomous Pathing Tuning")
public class StrafePathTest extends OpMode {
    private Telemetry telemetryA;

    private Follower follower;

    private Path strafe;

    long startTime = 0;

    /**
     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
     * initializes the FTC Dashboard telemetry.
     */
    @Override
    public void init() {
        follower = new Follower(hardwareMap);

        follower.setStartingPose(new Pose2d(48, 40, Math.PI));
        strafe = new Path(new BezierCurve(
                new Point(48,40, Point.CARTESIAN),
                new Point(8,58, Point.CARTESIAN)));

        strafe.setZeroPowerAccelerationMultiplier(4.75);
        strafe.setConstantHeadingInterpolation(Math.PI);
        follower.followPath(strafe);

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("This will run the robot in a diagonal path");
        telemetryA.update();
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the FTC Dashboard.
     */
    @Override
    public void loop() {
        if(startTime ==0) {
            startTime = System.currentTimeMillis();
        }
        if(follower.isBusy()) {
            follower.update();
        }
        if (!follower.isBusy() && !isDone) {
            try {
                Log.d("Petro_logger", "Elapsed time (ms): " + (System.currentTimeMillis()-startTime));
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
