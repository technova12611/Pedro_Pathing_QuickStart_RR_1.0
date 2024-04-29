package org.firstinspires.ftc.teamcode.pathing.purepursuit;

import static org.firstinspires.ftc.teamcode.pathing.purepursuit.PurePursuitConfig.MAX_ROTATIONAL_SPEED;
import static org.firstinspires.ftc.teamcode.pathing.purepursuit.PurePursuitConfig.MAX_TRANSLATIONAL_SPEED;
import static org.firstinspires.ftc.teamcode.pathing.purepursuit.PurePursuitConfig.X_GAIN;
import static org.firstinspires.ftc.teamcode.pathing.purepursuit.PurePursuitConfig.hD;
import static org.firstinspires.ftc.teamcode.pathing.purepursuit.PurePursuitConfig.hP;
import static org.firstinspires.ftc.teamcode.pathing.purepursuit.PurePursuitConfig.xD;
import static org.firstinspires.ftc.teamcode.pathing.purepursuit.PurePursuitConfig.xP;
import static org.firstinspires.ftc.teamcode.pathing.purepursuit.PurePursuitConfig.yD;
import static org.firstinspires.ftc.teamcode.pathing.purepursuit.PurePursuitConfig.yP;

import android.util.Log;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.pathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pathing.purepursuit.PurePursuitConfig;
import org.firstinspires.ftc.teamcode.pathing.purepursuit.PurePursuitPath;
import org.firstinspires.ftc.teamcode.roadrunner.messages.PoseMessage;

public class PurePursuitAction implements Action {
    private final MecanumDrive drivetrain;
    private final PurePursuitPath purePursuitPath;
    private final Pose endPose;

    private boolean PID = false;
    private boolean finished = false;

    public static com.arcrobotics.ftclib.controller.PIDFController xController = new com.arcrobotics.ftclib.controller.PIDFController(xP, 0.0, xD, 0);
    public static com.arcrobotics.ftclib.controller.PIDFController yController = new com.arcrobotics.ftclib.controller.PIDFController(yP, 0.0, yD, 0);
    public static com.arcrobotics.ftclib.controller.PIDFController hController = new PIDFController(hP, 0.0, hD, 0);

    private ElapsedTime accelLimit;
    private final double ACCEL_LIMIT = 0.5;

    private ElapsedTime timer;
    private ElapsedTime loopTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public PurePursuitAction(MecanumDrive drivetrain, PurePursuitPath purePursuitPath) {
        this.drivetrain = drivetrain;
        this.purePursuitPath = purePursuitPath;
        this.endPose = purePursuitPath.getEndPose();
    }

    @Override
    public boolean run(TelemetryPacket packet) {
        if (accelLimit == null) accelLimit = new ElapsedTime();
        if (purePursuitPath.isFinished()) PID = true;
        if (PID && timer == null) {
            timer = new ElapsedTime();

        }

        Log.d("PP_Drive_logger", "Prev Loop time (ms): " + String.format("%3.3f",loopTimer.milliseconds()));
        loopTimer.reset();

        drivetrain.updatePoseEstimate();

        Log.d("PP_Drive_logger", "Localizer elapsed time (ms): " + String.format("%3.3f",loopTimer.milliseconds()));


        Pose2d pose = drivetrain.pose;
        Pose robotPose =  new Pose(-pose.position.y, pose.position.x, pose.heading.toDouble());
        Pose targetPose = purePursuitPath.update(robotPose);

        Log.d("PP_Drive_logger", "drive pose=" + robotPose.toString() + " | target pose=" + targetPose.toString());

        if (PID && targetPose.subt(robotPose).toVec2D().magnitude() < PurePursuitConfig.ALLOWED_TRANSLATIONAL_ERROR
                && Math.abs(targetPose.subt(robotPose).heading) < PurePursuitConfig.ALLOWED_HEADING_ERROR) finished = true;

        if (targetPose.heading - robotPose.heading > Math.PI) targetPose.heading -= 2 * Math.PI;
        if (targetPose.heading - robotPose.heading < -Math.PI) targetPose.heading += 2 * Math.PI;

        double xPower = xController.calculate(robotPose.x, targetPose.x);
        double yPower = yController.calculate(robotPose.y, targetPose.y);
        double hPower = -hController.calculate(robotPose.heading, targetPose.heading);

        double x_rotated = xPower * Math.cos(-robotPose.heading) - yPower * Math.sin(-robotPose.heading);
        double y_rotated = xPower * Math.sin(-robotPose.heading) + yPower * Math.cos(-robotPose.heading);

        Log.d("PP_Drive_logger", "motor power: " + String.format("(x=%3.3f,y=%3.2f,h=%3.2f)", x_rotated, y_rotated, hPower));

        hPower = Range.clip(hPower, -MAX_ROTATIONAL_SPEED, MAX_ROTATIONAL_SPEED);
        x_rotated = Range.clip(x_rotated, -MAX_TRANSLATIONAL_SPEED / X_GAIN, MAX_TRANSLATIONAL_SPEED / X_GAIN);
        y_rotated = Range.clip(y_rotated, -MAX_TRANSLATIONAL_SPEED, MAX_TRANSLATIONAL_SPEED);

        drivetrain.set(
        new Pose(x_rotated * X_GAIN, y_rotated, hPower));//.scale(Math.min(accelLimit.seconds() / ACCEL_LIMIT, 1)));

        if(isFinished()) {
            end(false);
            return false;
        }

        return true;
    }

    public boolean isFinished() {
        return PID && finished || (timer != null && timer.milliseconds() > 2500);
    }

    public void end(boolean interrupted) {
        drivetrain.setPower(new Pose());
    }

}


