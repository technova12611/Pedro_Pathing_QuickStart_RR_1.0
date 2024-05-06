package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;
import org.firstinspires.ftc.teamcode.utils.software.AutoActionScheduler;

@Config
@Disabled
@Autonomous(group = "Test")
public final class AutoPathTestForDonuts extends LinearOpMode {

    public static Pose2d starting = new Pose2d(16, -62, Math.PI/2);
 //   public static Pose2d starting = new Pose2d(48.5, -35.5, Math.PI);

    //   public static Pose2d starting = new Pose2d(16.0, -63.0, Math.PI/2);

    public static Pose2d backdrop = new Pose2d(49.0, -35.5, Math.PI);
    public static Pose2d spike = new Pose2d(28.5, -25.5, Math.PI);
    protected AutoActionScheduler sched;
    protected Intake intake;
    protected Outtake outtake;
    protected MecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new MecanumDrive(hardwareMap, starting, true);

        intake = new Intake(hardwareMap, false);
        outtake = new Outtake(hardwareMap, false);

        this.sched = new AutoActionScheduler(this::update,hardwareMap);

        intake.initialize(true);
        outtake.initialize();

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
                    new MecanumDrive.DrivePoseLoggingAction(drive, "program_start"),
                    drive.actionBuilder(drive.pose)
                            .setTangent(0)
                            .strafeToLinearHeading(backdrop.position, backdrop.heading)
                            .build(),
                    new MecanumDrive.DrivePoseLoggingAction(drive, "backdrop_end"),
                    new SleepAction(0.5),
                    drive.actionBuilder(backdrop)
                            //.setTangent(0)
                            .strafeTo(spike.position)
                            .build(),
                    new MecanumDrive.DrivePoseLoggingAction(drive, "spike_end")
            )
        );

        sched.run();

        while(!isStopRequested()) {
            telemetry.addData("Game timer: ", getRuntime());
            idle();
        }

    }

    final public void update() {
        telemetry.addData("Time left", 30 - getRuntime());
        outtake.update();
        intake.update();
    }
}
