package org.firstinspires.ftc.teamcode.utils.software;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.pathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pathing.purepursuit.PurePursuitConfig;
import org.firstinspires.ftc.teamcode.pathing.purepursuit.PurePursuitPath;
import org.firstinspires.ftc.teamcode.roadrunner.TwoDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.utils.control.PIDCoefficients;
import org.firstinspires.ftc.teamcode.utils.control.PIDFController;

public class PurePursuitAction implements Action {

    private final MecanumDrive drivetrain;
    private final TwoDeadWheelLocalizer localizer;
    private final PurePursuitPath purePursuitPath;
    private final Pose endPose;

    private boolean PID = false;
    private boolean finished = false;

    public static double xP = 0.0335;
    public static double xD = 0.006;

    public static double yP = 0.0335;
    public static double yD = 0.006;

    public static double hP = 1;
    public static double hD = 0.03;

    public static double kStatic = 0.05;

    public static PIDFController xController = new PIDFController(new PIDCoefficients(xP,0.0, xD),0, 0);
    public static PIDFController yController = new PIDFController(new PIDCoefficients(yP,0.0, yD),0, 0);
    public static PIDFController hController = new PIDFController(new PIDCoefficients(hP,0.0, hD),0, 0);

    private ElapsedTime timer;

    public PurePursuitAction(MecanumDrive drivetrain, TwoDeadWheelLocalizer localizer, PurePursuitPath purePursuitPath) {
        this.drivetrain = drivetrain;
        this.localizer = localizer;
        this.purePursuitPath = purePursuitPath;
        this.endPose = purePursuitPath.getEndPose();
    }

    @Override
    public boolean run(TelemetryPacket packet) {
        if (purePursuitPath.isFinished()) PID = true;

        drivetrain.updatePoseEstimate();
        Pose robotPose = new Pose(drivetrain.pose.position.x,  drivetrain.pose.position.y, drivetrain.pose.heading.toDouble());

        Pose targetPose = purePursuitPath.update(robotPose);

        if(PID && timer == null){
            timer = new ElapsedTime();
        }

        if (PID && targetPose.subt(robotPose).toVec2D().magnitude() < PurePursuitConfig.ALLOWED_TRANSLATIONAL_ERROR
                && Math.abs(targetPose.subt(robotPose).heading) < PurePursuitConfig.ALLOWED_HEADING_ERROR) {
            finished = true;
        }

        if (PID) {
            Pose delta = targetPose.subtract(robotPose);

            double xPower = xController.update(robotPose.x, targetPose.x);
            double yPower = yController.update(robotPose.y, targetPose.y);
            double hPower = -hController.update(0, delta.heading);

            double x_rotated = xPower * Math.cos(robotPose.heading) - yPower * Math.sin(robotPose.heading);
            double y_rotated = xPower * Math.sin(robotPose.heading) + yPower * Math.cos(robotPose.heading);

            if (Math.abs(x_rotated) < 0.01) x_rotated = 0;
            else x_rotated += kStatic * Math.signum(x_rotated);
            if (Math.abs(y_rotated) < 0.01) y_rotated = 0;
            else y_rotated += kStatic * Math.signum(y_rotated);
            if (Math.abs(hPower) < 0.01) hPower = 0;

            drivetrain.setPower(new Pose((y_rotated / drivetrain.getVoltage() * 12.5) *1.1, x_rotated / drivetrain.getVoltage() * 12.5, hPower / drivetrain.getVoltage() * 12.5));
        } else {
            Pose delta = targetPose.subtract(robotPose);
            double y_rotated = delta.x * Math.cos(robotPose.heading) - delta.y * Math.sin(robotPose.heading);
            double x_rotated = delta.x * Math.sin(robotPose.heading) + delta.y * Math.cos(robotPose.heading);

            double xPercentage = x_rotated / purePursuitPath.getRadius() * PurePursuitConfig.FOLLOW_SPEED;
            double yPercentage = y_rotated / purePursuitPath.getRadius() * PurePursuitConfig.FOLLOW_SPEED;
            double hPower = -hController.update(0, delta.heading);

            drivetrain.setPower(new Pose(xPercentage *1.1, yPercentage, hPower));
        }

        if(finished) {
            return false;
        }

        return true;
    }

    public boolean isFinished() {
        return PID && finished || (timer != null && timer.milliseconds() > 2000);
    }

    public void end(boolean interrupted) {
        drivetrain.setPower(new Pose());
    }

}


