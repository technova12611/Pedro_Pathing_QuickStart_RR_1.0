package org.firstinspires.ftc.teamcode.opmode.test;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.pathing.geometry.Point;
import org.firstinspires.ftc.teamcode.pathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pathing.purepursuit.PurePursuitAction;
import org.firstinspires.ftc.teamcode.pathing.purepursuit.PurePursuitPath;
import org.firstinspires.ftc.teamcode.pathing.purepursuit.Waypoint;
import org.firstinspires.ftc.teamcode.roadrunner.messages.PoseMessage;
import org.firstinspires.ftc.teamcode.subsystem.Drone;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;
import org.firstinspires.ftc.teamcode.utils.software.AutoActionScheduler;

@Config
@Autonomous(name = "Blue PP Auto Path Test",group = "Test")
public final class PPAutoPathTest extends LinearOpMode {
    public static Pose2d starting = new Pose2d(0, 0, 0);

    protected AutoActionScheduler sched;
    protected Intake intake;
    protected Outtake outtake;
    protected MecanumDrive drive;
    protected Drone drone;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new MecanumDrive(hardwareMap, starting,true);
        intake = new Intake(hardwareMap, false);
        outtake = new Outtake(hardwareMap, false);
        this.drone = new Drone(hardwareMap);
        this.sched = new AutoActionScheduler(this::update,hardwareMap);

        intake.initialize(true);
        outtake.initialize();
        drone.initialize();

        while(!isStarted() && !isStopRequested()) {
            telemetry.addLine("Ready to start!! Blue Auto Test");
            telemetry.update();
        }

        drive.pose = new Pose2d(0.0, 0.0, Math.toRadians(0.0));

        Pose start = new Pose(0, 0.0, 0);
        Pose target = new Pose(-24, 53.0, Math.toRadians(0.0));

        PurePursuitPath test_path = new PurePursuitPath(
                new Waypoint(start, 15),
                new Waypoint(new Point(-10, 12.0), 15),
                new Waypoint(new Point(-22, 24.0), 15),
                new Waypoint(target, 15)
        );

        ElapsedTime timer1 = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        sched.addAction(
                new SequentialAction(
                        new MecanumDrive.DrivePoseLoggingAction(drive, "path_begin"),
                        new PurePursuitAction(drive, test_path)));
        sched.run();

        double elapsedTime1 = timer1.milliseconds();

        Thread.sleep(2000);

        drive.updatePoseEstimate();
        Pose2d path1_pose = drive.pose;

        timer1.reset();
        PurePursuitPath test_path2 = new PurePursuitPath(
                new Waypoint(target, 16),
                new Waypoint(new Point(-22, 24.0), 16),
                new Waypoint(new Point(-10, 12.0), 16),
                new Waypoint(start, 16)
        );
        sched.addAction(
                new SequentialAction(
                        new MecanumDrive.DrivePoseLoggingAction(drive, "path_begin"),
                        new PurePursuitAction(drive, test_path2)));
        sched.run();

        double elapsedTime2 = timer1.milliseconds();
        Thread.sleep(500);

        Log.d("Drive_logger", "--- Elapsed Time (ms): " + String.format("%3.1f,%3.1f", elapsedTime1, elapsedTime2));
        Log.d("Drive_logger", "--- End of run pose. Path1=" + new PoseMessage(path1_pose) + "| Path2=" + new PoseMessage(drive.pose));


        boolean firstTime = true;
        Pose2d endPose = drive.pose;
        while(!isStopRequested()) {
            if(sched.isEmpty()) {
                drive.updatePoseEstimate();
                telemetry.addData("Auto elapsed time: ", sched.autoRunElapsedTime);
                telemetry.addData("End Pose: ", new PoseMessage(endPose));
                telemetry.update();
            }
            idle();
        }
    }

    final public void update() {
        telemetry.addData("Time left", 30 - getRuntime());
        outtake.update();
        intake.update();
    }
}
