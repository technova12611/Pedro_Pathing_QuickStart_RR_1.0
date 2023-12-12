package org.firstinspires.ftc.teamcode.opmode.autonomous;

import android.util.Log;
import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pipeline.AlliancePosition;
import org.firstinspires.ftc.teamcode.pipeline.FieldPosition;
import org.firstinspires.ftc.teamcode.pipeline.PropBasePipeline;
import org.firstinspires.ftc.teamcode.pipeline.PropFarPipeline;
import org.firstinspires.ftc.teamcode.pipeline.PropNearPipeline;
import org.firstinspires.ftc.teamcode.pipeline.Side;
import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.PoseMessage;
import org.firstinspires.ftc.teamcode.subsystem.Hang;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Memory;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;
import org.firstinspires.ftc.teamcode.subsystem.Drone;
import org.firstinspires.ftc.teamcode.utils.software.AutoActionScheduler;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
public abstract class AutoBase extends LinearOpMode {
    protected MecanumDrive drive;
    protected Outtake outtake;
    protected Intake intake;
//    protected Vision vision;
    protected Drone drone;
    protected Hang hang;
    protected AutoActionScheduler sched;

    private PropBasePipeline propPipeline;
    private VisionPortal portal;

    public static Side side = Side.RIGHT;
    public static int SPIKE = 2;

    final public void update() {
        telemetry.addData("Time left", 30 - getRuntime());
        outtake.update();
        intake.update();
    }

    final public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing... Please wait");
        telemetry.update();

        Memory.LAST_POSE = getStartPose();
        Memory.RAN_AUTO = true;
        Globals.RUN_AUTO = true;

        // Init subsystems
        this.drive = new MecanumDrive(hardwareMap, Memory.LAST_POSE);
        this.intake = new Intake(hardwareMap);
        this.outtake = new Outtake(hardwareMap);
//        this.vision = new Vision(hardwareMap);
        this.drone = new Drone(hardwareMap);
        this.hang = new Hang(hardwareMap);

        this.sched = new AutoActionScheduler(this::update);

        outtake.initialize();
        intake.initialize(true);
        drone.initialize();
        hang.initialize();

        Globals.COLOR = getAlliance();

        if(getFieldPosition() == FieldPosition.NEAR) {
            propPipeline = new PropNearPipeline();
        } else {
            propPipeline = new PropFarPipeline();
        }

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, Globals.FRONT_WEBCAM_NAME))
                .setCameraResolution(new Size(1920, 1080))
                .addProcessor(propPipeline)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

        onInit();

        while (opModeInInit()) {
            side = propPipeline.getLocation();

            SPIKE = side.ordinal();
            printDescription();

            if(getAlliance() == AlliancePosition.RED) {
                telemetry.addData("Mean Right color:", "%3.2f", propPipeline.meanSideColor);
                telemetry.addData("Mean Left color:", "%3.2f", propPipeline.meanCenterColor);
            } else {
                telemetry.addData("Mean Left color:", "%3.2f", propPipeline.meanSideColor);
                telemetry.addData("Mean Center color:", "%3.2f",propPipeline.meanCenterColor);
            }
            telemetry.addData("Spike Position", side.toString());
            telemetry.update();
            idle();
        }

        portal.close();

        // Auto start
        resetRuntime(); // reset runtime timer
        Memory.saveStringToFile(String.valueOf(System.currentTimeMillis()), Memory.SAVED_TIME_FILE_NAME); // save auto time for persistence

        if (isStopRequested()) return; // exit if stopped

        onRun();
        drive.pose = getStartPose();
        drive.imu.resetYaw();

        MecanumDrive.previousLogTimestamp = System.currentTimeMillis();
        sched.run();
        Memory.LAST_POSE = drive.pose;
        Globals.drivePose = drive.pose;

        Log.d("Auto", "Auto path ended at " + getRuntime());
        Log.d("Auto", "End path drive EstimatedPose " + new PoseMessage(drive.pose));

        while(opModeIsActive()) {
            Globals.drivePose = drive.pose;
            idle();
        }

        Log.d("Auto", "Auto program ended at " + getRuntime());
        Log.d("Auto", "End program drive EstimatedPose " + new PoseMessage(drive.pose));
    }

    protected abstract Pose2d getStartPose();
    protected abstract void printDescription();
    protected void onInit() {}
    protected abstract void onRun();

    protected abstract AlliancePosition getAlliance();
    protected abstract FieldPosition getFieldPosition();
}

