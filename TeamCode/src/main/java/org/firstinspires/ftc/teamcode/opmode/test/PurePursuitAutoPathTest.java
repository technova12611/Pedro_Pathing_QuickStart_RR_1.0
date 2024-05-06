package org.firstinspires.ftc.teamcode.opmode.test;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.pathing.purepursuit.PurePursuitAction;
import org.firstinspires.ftc.teamcode.pathing.purepursuit.PurePursuitPath;
import org.firstinspires.ftc.teamcode.pathing.purepursuit.Waypoint;
import org.firstinspires.ftc.teamcode.roadrunner.messages.PoseMessage;
import org.firstinspires.ftc.teamcode.subsystem.Drone;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;
import org.firstinspires.ftc.teamcode.utils.software.AutoActionScheduler;

@Config
@Autonomous(name = "Blue AutoPath PP Test",group = "Test")
public final class PurePursuitAutoPathTest extends LinearOpMode {
    public static Pose2d starting = new Pose2d(14.5, -62.0, Math.PI/2);
    public static Pose2d cycleStart = new Pose2d(28.5, -11.0, Math.PI);
    public static Pose2d backdropAlignment = new Pose2d(38.0, -11.0, Math.PI);
    public static Pose2d backdrop = new Pose2d(48.5, -36.0, Math.PI);
    public static Pose2d spike = new Pose2d(28.5, -24.5, Math.PI);
    public static Pose2d parking = new Pose2d(52.0, -60.0, Math.PI);

    public static Pose2d stackAlignment = new Pose2d(-48.0, -11.0, Math.PI);
    public static Pose2d stackIntake = new Pose2d(-56.0, -12.0, Math.PI);
    public Pose2d safeTrussPassStop = new Pose2d(-49.0, -11.0, Math.toRadians(180));

    protected AutoActionScheduler sched;
    protected Intake intake;
    protected Outtake outtake;
    protected MecanumDrive drive;
    protected Drone drone;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new MecanumDrive(hardwareMap, backdropAlignment,true);
        intake = new Intake(hardwareMap, false);
        outtake = new Outtake(hardwareMap, false);
        this.drone = new Drone(hardwareMap);
        this.sched = new AutoActionScheduler(this::update,hardwareMap);

        intake.initialize(true);
        outtake.initialize();
        drone.initialize();

        drive.startIMUThread(this);

        // 1. drive to the backstage
        // 2. raise the slide and prepare the yellow pixel drop
        // 3. score the yellow pixel onto the backdrop
        // 4. retract slide
        // 5. drive to the spike position
        // 6. drop the purple pixel
        // 7. drive to parking
        while(!isStarted() && !isStopRequested()) {
            telemetry.addLine("Ready to start!! Blue Auto Test");
            telemetry.update();
        }

        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        drive.pose = new Pose2d(14.5, 62, Math.toRadians(-90));
        Pose2d backdrop = new Pose2d(48.0,36.0, Math.toRadians(180.00));
        Pose2d cycle = new Pose2d(47.0,32.0, Math.toRadians(180.00));

        // drop purple pixel
        sched.addAction(
                new SequentialAction(
                        new MecanumDrive.DrivePoseLoggingAction(drive, "path_begin"),
                        intake.intakeOff(),
                        new PurePursuitAction(drive,
                                new PurePursuitPath(
                                        new Waypoint(drive.pose, 15),
                                        new Waypoint(new Pose2d(32, 24.0, Math.toRadians(180)), 15))
                        ),
                        new SleepAction(0.5),
                        new MecanumDrive.DrivePoseLoggingAction(drive, "Dropped_purple")));
        sched.run();

        // drop yellow
        sched.addAction(
                new SequentialAction(
                        intake.intakeOff(),
                        new PurePursuitAction(drive,
                                new PurePursuitPath(
                                        new Waypoint(drive.pose, 15),
                                        new Waypoint(backdrop, 15))
                        ),
                        new SleepAction(1.0),
                        new MecanumDrive.DrivePoseLoggingAction(drive, "Dropped_yellow")));
        sched.run();

        Pose2d stack = new Pose2d(-56.0, 12.5, Math.toRadians(180));

        int i = 0;
        double position_y = 12.0;
        while(i++ < 3) {
            if(i == 2) {
                position_y = 12.5;
                Pose2d currentPose = drive.pose;
                drive.pose = new Pose2d(currentPose.position.x, currentPose.position.y-0.5, currentPose.heading.toDouble());
            }
            if(i==3) {
                position_y = 13.0;
                Pose2d currentPose = drive.pose;
                drive.pose = new Pose2d(currentPose.position.x, currentPose.position.y-1.0, currentPose.heading.toDouble());
            }
            drive.updatePoseEstimate();
            sched.addAction(
                    new SequentialAction(
                            new MecanumDrive.DrivePoseLoggingAction(drive, "cycle_stack_begin_"+i),
                            new PurePursuitAction(drive, new PurePursuitPath(
                                new Waypoint(drive.pose, 18),
                                new Waypoint(new Vector2d(30, position_y), 18),
                                new Waypoint(new Vector2d(-24, position_y), 18),
                            new Waypoint(stack, 18))),
                            new MecanumDrive.DrivePoseLoggingAction(drive, "cycle_statck_end_"+i),
                            new SleepAction(1.5)
                    )
            );

            sched.run();

            sched.addAction(
                    new SequentialAction(
                            new MecanumDrive.DrivePoseLoggingAction(drive, "cycle_backdrop_begin_"+i),
                            new PurePursuitAction(drive, new PurePursuitPath(
                                    new Waypoint(drive.pose, 18),
//                                    new Waypoint(new Pose2d(-24, position_y, Math.toRadians(180.0)), 15),
                                    new Waypoint(new Vector2d(30, position_y), 18),
                                    new Waypoint(cycle, 18))),
                            new MecanumDrive.DrivePoseLoggingAction(drive, "cycle_backdrop_end_"+i),
                            new SleepAction(1.0)
                            ));
            sched.run();
        }

        boolean firstTime = true;
        Pose2d endPose = drive.pose;
        while(!isStopRequested()) {
            if(sched.isEmpty()) {
                drive.updatePoseEstimate();
                if(firstTime) {
                    endPose = drive.pose;
                    firstTime = false;
                    Log.d("Drive_logger", "End drive pose: " + new PoseMessage(drive.pose));
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
        outtake.update();
        intake.update();
    }
}
