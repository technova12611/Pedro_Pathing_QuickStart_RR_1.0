package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.PoseMessage;
import org.firstinspires.ftc.teamcode.subsystem.Drone;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;
import org.firstinspires.ftc.teamcode.utils.software.AutoActionScheduler;

@Config
@Autonomous(group = "Test")
public final class AutoPathTest extends LinearOpMode {

    public static Pose2d starting = new Pose2d(16.0, -62.5, Math.PI/2);
    public static Pose2d backdrop = new Pose2d(48.5, -36.0, Math.PI);

    public static Pose2d spike = new Pose2d(28.5, -24.5, Math.PI);

    public static Pose2d parking = new Pose2d(52.0, -60.0, Math.PI);

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
                        // to score yellow pixel on the backdrop
                        drive.actionBuilder(drive.pose)
                                .setTangent(0)
                                .splineTo(backdrop.position, Math.PI/2)
                                .build(),
                        outtake.extendOuttakeLow(),
                        outtake.prepareToScore(),
                        outtake.latchScore1(),
                        new SleepAction(0.75),
                        new ParallelAction(
                            outtake.retractOuttake(),
                            intake.stackIntakeLinkageDown(),

                            // to score the purple pixel on the spike
                            drive.actionBuilder(backdrop)
                                    .strafeTo(spike.position)
                                    .build()
                        ),

                        intake.scorePurplePreload(),
                        new SleepAction(0.5),

                        // to park and prepare for teleops
                        intake.prepareTeleOpsIntake(),
                        //outtake.prepareToTransfer(),

                        drive.actionBuilder(spike)
                                .setReversed(true)
                                .strafeTo(parking.position)
                                .build()
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
    }

    final public void update() {
        telemetry.addData("Time left", 30 - getRuntime());
        outtake.update();
        intake.update();
    }
}
