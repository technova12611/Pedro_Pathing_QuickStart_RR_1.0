package org.firstinspires.ftc.teamcode.opmode.test;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.messages.PoseMessage;
import org.firstinspires.ftc.teamcode.subsystem.Drone;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;
import org.firstinspires.ftc.teamcode.utils.software.AutoActionScheduler;

@Config
@Disabled
@Autonomous(group = "Test")
public final class AutoPathTest extends LinearOpMode {
    public static Pose2d starting = new Pose2d(14.5, -62.0, Math.PI/2);
    public static Pose2d cycleStart = new Pose2d(28.5, -11.0, Math.PI);
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

        drive = new MecanumDrive(hardwareMap, starting);
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
        this.drone = new Drone(hardwareMap);
        this.sched = new AutoActionScheduler(this::update);

        intake.initialize(true);
        outtake.initialize();
        drone.initialize();

        waitForStart();

        // 1. drive to the backstage
        // 2. raise the slide and prepare the yellow pixel drop
        // 3. score the yellow pixel onto the backdrop
        // 4. retract slide
        // 5. drive to the spike position
        // 6. drop the purple pixel
        // 7. drive to parking
        sched.addAction(
                new SequentialAction(
                        new MecanumDrive.DrivePoseLoggingAction(drive, "path_begin"),
                        drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(backdrop.position, backdrop.heading)
                                .build(),

                        new MecanumDrive.DrivePoseLoggingAction(drive, "backdrop"),
                        drive.actionBuilder(backdrop)
                                .strafeToLinearHeading(spike.position, spike.heading)
                                .build(),

                        new MecanumDrive.DrivePoseLoggingAction(drive, "spike"),
                        drive.actionBuilder(spike)
                                .strafeToLinearHeading(cycleStart.position, cycleStart.heading)
                                .build(),

                        new MecanumDrive.DrivePoseLoggingAction(drive, "cycle"),
                        new ParallelAction(
                            drive.actionBuilderFast(cycleStart)
                                    .strafeTo(stackAlignment.position)
                                    .build(),
                                new SequentialAction(
                                new SleepAction(1.0),
                                    intake.prepareStackIntake()
                            )
                        ),
                        new MecanumDrive.DrivePoseLoggingAction(drive, "stackIntake"),
                        drive.actionBuilder(stackAlignment)
                                .strafeTo(stackIntake.position)
                                .build(),

                        intake.intakeTwoStackedPixels(),
                        new MecanumDrive.DrivePoseLoggingAction(drive, "stackIntake_end")

//                        new ParallelAction(
//                                new SequentialAction(
//                                        drive.actionBuilder(stackIntake)
//                                                .setReversed(true)
//                                                .strafeTo(safeTrussPassStop.position)
//                                                .build(),
//                                        new MecanumDrive.DrivePoseLoggingAction(drive, "safe_pass_stop")
//                                ),
//
//                                new SequentialAction(
//                                        new SleepAction(0.5),
//                                        intake.stackIntakeLinkageUp(),
//                                        new SleepAction(1.25),
//                                        intake.prepareTeleOpsIntake(),
//                                        new MecanumDrive.DrivePoseLoggingAction(drive, "Intake_off")
//                                )
//                        )
                )
        );

        sched.run();

        boolean firstTime = true;
        Pose2d endPose = drive.pose;
        while(!isStopRequested()) {
            if(sched.isEmpty()) {
                drive.updatePoseEstimate();
                if(firstTime) {
                    endPose = drive.pose;
                    firstTime = false;
                }

                telemetry.addData("Auto elapsed time: ", sched.autoRunElapsedTime);
                telemetry.addData("End Pose: ", new PoseMessage(endPose).toString());
                telemetry.update();
            }
            idle();
        }

        Log.d("Drive_logger", "End drive pose: " + new PoseMessage(drive.pose));
    }

    final public void update() {
        telemetry.addData("Time left", 30 - getRuntime());
        outtake.update();
        intake.update();
    }
}
