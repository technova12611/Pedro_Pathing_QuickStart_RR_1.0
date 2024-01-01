package org.firstinspires.ftc.teamcode.opmode.autonomous;

import android.util.Log;
import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pipeline.AlliancePosition;
import org.firstinspires.ftc.teamcode.pipeline.FieldPosition;
import org.firstinspires.ftc.teamcode.pipeline.PropBasePipeline;
import org.firstinspires.ftc.teamcode.pipeline.PropFarPipeline;
import org.firstinspires.ftc.teamcode.pipeline.PropNearPipeline;
import org.firstinspires.ftc.teamcode.pipeline.Side;
import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.messages.PoseMessage;
import org.firstinspires.ftc.teamcode.subsystem.Hang;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Memory;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;
import org.firstinspires.ftc.teamcode.subsystem.Drone;
import org.firstinspires.ftc.teamcode.utils.hardware.GamePadController;
import org.firstinspires.ftc.teamcode.utils.software.AutoActionScheduler;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

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
    private GamePadController g1, g2;
    // configure a wait time to allow partner time to finish the backdrop
    //----------------------------------------------------------------
    public int farSideAutoWaitTimeInSeconds = 0;
    private int[] waitTimeOptions = {0,5,8};
    private int selectionIdx = 0;

    protected AprilTagProcessor aprilTag;
    protected VisionPortal visionPortal2;

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
        g1 = new GamePadController(gamepad1);
        g2 = new GamePadController(gamepad2);

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

        // init 2nd webcam to detect Backdrop apriltags
//        initAprilTag();

        onInit();

        while (opModeInInit()) {
            side = propPipeline.getLocation();

            SPIKE = side.ordinal();
            printDescription();
            telemetry.addLine("   ");
            telemetry.addLine(" <----- Team Prop Vision Detection -----> ");
            telemetry.addLine(" Wait a few seconds to capture the Maximum color value ");
            telemetry.addLine(" before placing the team prop on the field ");

            String sideStr = "Left";
            String centerStr = "Center";

            if(getAlliance() == AlliancePosition.RED) {
                if(getFieldPosition() == FieldPosition.NEAR) {
                    centerStr = "Left";
                    sideStr = "Right";
                } else {
                    sideStr = "Left";
                }
            } else {
                if(getFieldPosition() == FieldPosition.FAR) {
                    sideStr = "Right";
                }
            }

            telemetry.addData(centerStr + " color:", "Mean: %3.2f | Max: %3.2f ", propPipeline.meanCenterColor, propPipeline.maxCenterColor);
            telemetry.addData(sideStr + " color:", "Mean: %3.2f | Max: %3.2f ", propPipeline.meanSideColor, propPipeline.maxSideColor);
            telemetry.addData("Spike Position", side.toString());
            telemetry.addLine("\n");

            telemetry.addData("Outtake Servo Positions", outtake.getServoPositions());

            if(getFieldPosition() == FieldPosition.FAR) {
                // use dpad to select wait time
                if (g1.dpadUpOnce()) {
                    selectionIdx++;
                } else if (g1.dpadDownOnce()) {
                    selectionIdx--;
                }

                // cycle the idx
                if (selectionIdx < 0) {
                    selectionIdx = waitTimeOptions.length;
                } else if (selectionIdx > waitTimeOptions.length) {
                    selectionIdx = 0;
                }

                farSideAutoWaitTimeInSeconds = waitTimeOptions[selectionIdx];

                telemetry.addLine("   ");
                telemetry.addLine("<------- FAR side Wait Time Selection ------->");
                telemetry.addLine("  Use DPAD Up/Down button to select wait time ");
                telemetry.addData(   "    Wait Time to Score Yellow: ", farSideAutoWaitTimeInSeconds + " (seconds)");
            }

            telemetry.update();
            idle();
        }

        // Auto start
        resetRuntime(); // reset runtime timer
        MecanumDrive.previousLogTimestamp = System.currentTimeMillis();
        MecanumDrive.autoStartTimestamp = System.currentTimeMillis();

        try {
            portal.close();
        }catch(Exception e) {
            // ignore
        }

        if (isStopRequested()) return; // exit if stopped

        Log.d("Auto_logger", String.format("Auto program started at %.3f", getRuntime()));

        drive.pose = getStartPose();
        // reset IMU
        // ----------------------------
        try {
            drive.imu.resetYaw();
        } catch (Exception e) {
            //
        }

        // prepare for the run, build the auto path
        //-------------------------------------------
        onRun();

        Log.d("Auto_logger", String.format("Auto program after onRun %.3f", getRuntime()));

        // run the auto path, all the actions are queued
        //-------------------------------
        sched.run();

        Log.d("Auto_logger", String.format("Auto program ended at %.3f", getRuntime()));

        drive.updatePoseEstimate();

//        try {
//            if (visionPortal2 != null) {
//                visionPortal2.close();
//            }
//        } catch(Exception e) {
//            //
//        }

        // end of the auto run
        // keep position and settings in memory for TeleOps
        //--------------------------------------------------
        Memory.LAST_POSE = drive.pose;
        Globals.drivePose = drive.pose;

        Log.d("Auto_logger", "End path drive Estimated Pose: " + new PoseMessage(drive.pose) + "| " +
                "Heading: " + String.format("%3.2f", drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)));

        double logStartTime = getRuntime();
        while(opModeIsActive()) {
            drive.updatePoseEstimate();
            Globals.drivePose = drive.pose;
            if(getRuntime() - logStartTime > 5.0) {
                Log.d("Auto_logger","End program drive Estimated Pose: " + new PoseMessage(drive.pose) + "| " +
                        "Heading: " + String.format("%3.2f", drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)));
                logStartTime = getRuntime();
            }
            idle();
        }
    }
    private void initAprilTag() {
        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        visionPortal2 = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 2"), aprilTag);
    }

    // the following needs to be implemented by the real auto program
    // mainly the path and other scoring actions

    /**
     * define the different starting pose for each locations
     * RED Right, RED Left, BLUE Right, BLUE Left. All have different Starting Posees.
     *
     * @return
     */
    protected abstract Pose2d getStartPose();

    /**
     * print the user friendly message to alert driver the program is running
     */
    protected abstract void printDescription();

    /**
     * Initialize all the components, if anything is required beyond the base auto
     */
    protected void onInit() {}

    /**
     * Build the auto path and queue up actions to execute
     */
    protected abstract void onRun();

    // these are needed to know where the robot located
    protected abstract AlliancePosition getAlliance();
    protected abstract FieldPosition getFieldPosition();
}

