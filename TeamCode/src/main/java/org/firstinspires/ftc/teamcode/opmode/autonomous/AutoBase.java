package org.firstinspires.ftc.teamcode.opmode.autonomous;

import android.util.Log;
import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.NullAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pipeline.AlliancePosition;
import org.firstinspires.ftc.teamcode.pipeline.FieldPosition;
import org.firstinspires.ftc.teamcode.pipeline.PreloadDetectionPipeline;
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
import org.firstinspires.ftc.teamcode.utils.software.ActionUtil;
import org.firstinspires.ftc.teamcode.utils.software.AutoActionScheduler;
import org.firstinspires.ftc.teamcode.utils.software.DriveWithPID;
import org.firstinspires.ftc.teamcode.utils.software.MovingArrayList;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Config
public abstract class AutoBase extends LinearOpMode implements StackPositionCallback, BackdropPositionCallback {
    protected MecanumDrive drive;
    protected Outtake outtake;
    protected Intake intake;
    //    protected Vision vision;
    protected Drone drone;
    protected Hang hang;
    protected AutoActionScheduler sched;
    protected PropBasePipeline teamProPipeline;
    protected AprilTagProcessor aprilTag;
    protected PreloadDetectionPipeline preloadPipeline;

    protected VisionPortal frontVisionPortal;
    protected static VisionPortal backVisionPortal;

    public Side teamPropPosition = Side.CENTER;

    private Side prev_teamPropPosition = null;

    public int SPIKE = 1;
    private GamePadController g1, g2;
    // configure a wait time to allow partner time to finish the backdrop
    //----------------------------------------------------------------
    public int farSideAutoWaitTimeInSeconds = 0;
    private int[] waitTimeOptions = {0, 4, 7, 10};
    private int selectionIdx = 0;

    public static boolean displayDistanceSensor = true;

    protected static Pose2d stackPosition;
    protected static Side preloadPosition = Side.RIGHT;

    protected DriveWithPID pidDriveStraight;
    ElapsedTime loopTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    final public void update() {
        telemetry.addData("Time left", 30 - getRuntime());
        outtake.update();
        intake.update();
        pidDriveUpdate();
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
        this.drone = new Drone(hardwareMap);
        this.hang = new Hang(hardwareMap);

        this.sched = new AutoActionScheduler(this::update);

        outtake.initialize();
        intake.initialize(true);
        drone.initialize();
        hang.initialize();

        pidDriveStraight = new DriveWithPID(drive, null, DriveWithPID.DriveDirection.STRAIGHT);

        if (getFieldPosition() == FieldPosition.NEAR) {
            outtake.setupForSlidingInAuto();
        }

        Globals.COLOR = getAlliance();

        if (getFieldPosition() == FieldPosition.NEAR) {
            teamProPipeline = new PropNearPipeline();
        } else {
            teamProPipeline = new PropFarPipeline();
        }

        teamProPipeline.allianceColor = getAlliance();

        // initialize front and back vision portals
        loopTimer.reset();
        initVisionPortal();
        double webCamReadyTime = loopTimer.milliseconds();

        // initialize roadrunner positions
        onInit();

        double maxLoopTime = 0.0;
        double minLoopTime = 100.0;
        loopTimer.reset();

        while (opModeInInit()) {
            double loopTimeBegin = loopTimer.milliseconds();
            g1.update();

            teamPropPosition = teamProPipeline.getLocation();

            SPIKE = teamPropPosition.ordinal();
            printDescription();
            telemetry.addLine("   ");
            telemetry.addLine(" <----- Team Prop Vision Detection " + "(" + Globals.COLOR + ", " + getFieldPosition() + ")" + " -----> ");
            telemetry.addLine(" Check if the right alliance program is running ... ");
            telemetry.addLine(" Wait a few seconds to capture the Maximum color value ");
            telemetry.addLine(" before placing the team prop on the field ");

            double loopTimeForProp = loopTimer.milliseconds();

            String sideStr = "Left";
            String centerStr = "Center";

            if (getAlliance() == AlliancePosition.RED) {
                preloadPosition = Side.RIGHT;
                if (getFieldPosition() == FieldPosition.NEAR) {
                    centerStr = "Left";
                    sideStr = "Right";
                } else {
                    sideStr = "Left";
                }
            } else {
                preloadPosition = Side.LEFT;
                if (getFieldPosition() == FieldPosition.FAR) {
                    sideStr = "Right";
                }
            }

            telemetry.addData(centerStr + " color:", "Mean: %3.2f | Max: %3.2f ", teamProPipeline.meanCenterColor, teamProPipeline.maxCenterColor);
            telemetry.addData(sideStr + " color:", "Mean: %3.2f | Max: %3.2f ", teamProPipeline.meanSideColor, teamProPipeline.maxSideColor);
            telemetry.addData("Spike Position", teamPropPosition.toString() + " | SPIKE: " + SPIKE);
            telemetry.addLine("\n");

            if (getFieldPosition() == FieldPosition.FAR) {
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
                telemetry.addLine("   ");
                telemetry.addLine("    Wait Time to Score Yellow: " + farSideAutoWaitTimeInSeconds + " (seconds)");
                telemetry.addLine("   ");
            }

            double loopTimeForAfterProp = loopTimer.milliseconds();

            if (displayDistanceSensor) {
                telemetry.addLine(" Stack Distance sensor Left (in): " + String.format("%3.2f", intake.getStackDistanceLeft()));
                telemetry.addLine(" Stack Distance sensor Right (in): " + String.format("%3.2f", intake.getStackDistanceRight()));
            }

            double loopTimeForDistance = loopTimer.milliseconds();

            double loopTimeInMili = loopTimer.milliseconds();

            if (loopTimeInMili < minLoopTime) {
                minLoopTime = loopTimeInMili;
            } else if (loopTimeInMili > maxLoopTime) {
                maxLoopTime = loopTimeInMili;
            }

            telemetry.addLine(" Loop time: " + String.format("%.1f", loopTimeInMili)
                    + " | Min Loop: " + String.format("%.1f", minLoopTime) + " | Max Loop: " + String.format("%.1f", maxLoopTime));

            telemetry.addLine(" Elapsed time, loop begin: " + String.format("%.1f", loopTimeBegin) + " | For Prop: " + String.format("%.1f", loopTimeForProp)
                    + " | After Prop: " + String.format("%.1f", loopTimeForAfterProp) + " | For Distance: " + String.format("%.1f", loopTimeForDistance));

            telemetry.addLine("Outtake Servo Positions: " + outtake.getServoPositions());
            telemetry.addLine("Webcam ready time (ms):" + webCamReadyTime);

            loopTimer.reset();
            telemetry.update();

            drive.pose = getStartPose();
            if(prev_teamPropPosition == null || prev_teamPropPosition != teamPropPosition) {
                long start_onrun = System.currentTimeMillis();
                sched.reset();

                Log.d("Auto_logger", " onRun() starts for " + getAlliance() + "-" + getFieldPosition() + "-" + teamPropPosition + " | actions: " + sched.size() + " | Prev_Prop_Position: " + prev_teamPropPosition);
                onRun();

                prev_teamPropPosition = teamPropPosition;

                Log.d("Auto_logger", " onRun() finished, elapsed time:  "
                        + (System.currentTimeMillis() - start_onrun) + " | actions: " + sched.size());
            }

            idle();
        }

        // Auto start
        resetRuntime(); // reset runtime timer
        MecanumDrive.previousLogTimestamp = System.currentTimeMillis();
        MecanumDrive.autoStartTimestamp = System.currentTimeMillis();

        Log.d("Auto_logger", String.format("Auto program started at %.3f", getRuntime()));

        try {
            frontVisionPortal.close();
            teamProPipeline = null;
            frontVisionPortal = null;
        } catch (Exception e) {
            // ignore
        }
        Log.d("Auto_logger", String.format("Front Vision Portal closed at %.3f", getRuntime()));

        if (isStopRequested()) return; // exit if stopped

        // reset IMU
        // ----------------------------
        try {
            drive.imu.resetYaw();
        } catch (Exception e) {
            //
        }

        // prepare for the run, build the auto path
        //-------------------------------------------
        Log.d("Auto_logger", String.format("onRun() started at %.3f", getRuntime()) + " | actions: " + sched.size());
        if(prev_teamPropPosition != teamPropPosition || sched.isEmpty()) {
            sched.reset();
            onRun();
        }

        Log.d("Auto_logger", String.format("!!! onRun() finished at %.3f", getRuntime()));

        // run the auto path, all the actions are queued
        //-------------------------------
        sched.run();

        Log.d("Auto_logger", String.format("!!! Auto run() finished at %.3f", getRuntime()));

        try {
            if(backVisionPortal != null) backVisionPortal.close();

            aprilTag = null;
            preloadPipeline = null;
            backVisionPortal = null;
        } catch (Exception e) {
            // ignore
        }

        Log.d("Auto_logger", String.format("!!! Auto program ended at %.3f", getRuntime()));

        drive.updatePoseEstimate();

        // end of the auto run
        // keep position and settings in memory for TeleOps
        //--------------------------------------------------
        Memory.LAST_POSE = drive.pose;
        Globals.drivePose = drive.pose;

        Log.d("Auto_logger", "End path drive Estimated Pose: " + new PoseMessage(drive.pose));

        double logStartTime = getRuntime();
        while (opModeIsActive()) {
            drive.updatePoseEstimate();
            Globals.drivePose = drive.pose;
            if (getRuntime() - logStartTime > 5.0) {
                Log.d("Auto_logger", "End program drive Estimated Pose: "
                        + new PoseMessage(drive.pose) + " at "
                        + String.format("%3.3f",getRuntime()));
                logStartTime = getRuntime();
            }
            idle();
        }
    }

    protected void initVisionPortal() {
        frontVisionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, Globals.FRONT_WEBCAM_NAME))
                .setCameraResolution(new Size(1920, 1080))
                .addProcessor(teamProPipeline)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(false)
 //               .setAutoStopLiveView(true)
                .build();

        while (!frontVisionPortal.getCameraState().equals(VisionPortal.CameraState.STREAMING)) {
            telemetry.addLine(" Please wait, Webcam 1 is streaming ... ");
            telemetry.update();
            idle();
        }

        frontVisionPortal.setProcessorEnabled(teamProPipeline, true);
    }

    // the following needs to be implemented by the real auto program
    // mainly the path and other scoring actions

    /**
     * define the different starting pose for each locations
     * RED Right, RED Left, BLUE Right, BLUE Left. All have different Starting Posees.
     *
     */
    protected abstract Pose2d getStartPose();

    /**
     * print the user friendly message to alert driver the program is running
     */
    protected abstract void printDescription();

    /**
     * Initialize all the components, if anything is required beyond the base auto
     */
    protected void onInit() {
    }

    /**
     * Build the auto path and queue up actions to execute
     */
    protected abstract void onRun();

    // these are needed to know where the robot located
    protected abstract AlliancePosition getAlliance();

    protected abstract FieldPosition getFieldPosition();

    protected static double x_adjustment = 0.0;
    protected static double y_adjustment = 0.0;

    public static class StackIntakePositionAction implements Action {
        MecanumDrive drive;
        Intake intake;
        Pose2d stackPose;
        Boolean firstTime = null;

        int counter = 0;
        long startTime = 0;

        MovingArrayList sensorDistancesLeftList = new MovingArrayList(10);
        MovingArrayList sensorDistancesRightList = new MovingArrayList(10);

        public StackIntakePositionAction(MecanumDrive drive, Intake intake, Pose2d stackPose) {
            this.drive = drive;
            this.intake = intake;
            this.stackPose = stackPose;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            if (counter++ == 0) {
                startTime = System.currentTimeMillis();
                AutoBase.x_adjustment = 0.0;
            }

            double sensorElapsedTime = System.currentTimeMillis();
            double stackDistanceLeft = intake.getStackDistanceLeft();
            double stackDistanceRight = intake.getStackDistanceRight();

            if (System.currentTimeMillis() - sensorElapsedTime > 200) {
                AutoBase.setStackPositionStatic(stackPose);
                Log.d("StackIntakePosition_Logger", "Current Drive Pose: " + new PoseMessage(drive.pose) +
                        "sensor failed !!!" + "Elapsed time: " + (System.currentTimeMillis() - startTime)
                        + "Left sensor:" + String.format("%3.2f", stackDistanceLeft) + ","
                        + "Right sensor: " + String.format("%3.2f", stackDistanceRight)
                );

                return false;
            }
            if (counter > 5) {
                drive.updatePoseEstimate();
                double delta;
                if (drive.pose.heading.toDouble() > 0) {
                    delta = 5.0 * Math.tan(drive.pose.heading.toDouble() - Math.toRadians(180));
                } else {
                    delta = 5.0 * Math.tan(Math.toRadians(180) + drive.pose.heading.toDouble());
                }

                double avg_y_adj_left = sensorDistancesLeftList.getAvg() * 1.0;
                double avg_y_adj_right = sensorDistancesRightList.getAvg() * 1.0 + delta / 2.0;

                try {
                    Log.d("StackIntakePosition_Logger", "Current Drive Pose: " + new PoseMessage(drive.pose)
                            + " | Stack Target Pose: " + new PoseMessage(stackPose) + " | count: " + counter
                            + " | avg_stack_position (adj_y_left, adj_y_right, delta): "
                            + String.format("%3.2f", avg_y_adj_left) + ","
                            + String.format("%3.2f", avg_y_adj_right) + ","
                            + String.format("%3.2f", (avg_y_adj_left - avg_y_adj_right))
                            + " | angle adjustment: " + String.format("%3.2f", delta)
                    );
                } catch (Exception e) {
                    Log.d("StackIntakePosition_Logger", "Exception 2:  " + e.getMessage());
                }

                double adjustment_y = 0.0;
                double adjustment_x = 0.0;

                double delta_stack_position = avg_y_adj_left - avg_y_adj_right;

                double y_offset = 1.25;
                if (Math.abs(delta_stack_position) <= 0.6 && avg_y_adj_left > 5.0 && avg_y_adj_right > 5.0) {
                    adjustment_x = (avg_y_adj_left + avg_y_adj_right) / 2 - 0.5;
                } else if (delta_stack_position < -1.8) {
                    adjustment_y = -1.75;
                    adjustment_x = avg_y_adj_right - y_offset;
                } else if (delta_stack_position < -1.2) {
                    adjustment_y = -1.05;
                    adjustment_x = avg_y_adj_right - y_offset;
                } else if (delta_stack_position < -0.6) {
                    adjustment_y = -0.5;
                    adjustment_x = avg_y_adj_right - y_offset;
                } else if (delta_stack_position > 1.8) {
                    adjustment_y = 1.75;
                    adjustment_x = avg_y_adj_left - y_offset;
                } else if (delta_stack_position > 1.2) {
                    adjustment_y = 1.05;
                    adjustment_x = avg_y_adj_left - y_offset;
                } else if (delta_stack_position > 0.6) {
                    adjustment_y = 0.5;
                    adjustment_x = avg_y_adj_left - y_offset;
                }

                if (avg_y_adj_left < 5.0) {
                    adjustment_y = -1.0;
                }

                Log.d("StackIntakePosition_Logger", "calculated adjustment (x,y): " +
                        String.format("(%3.2f,%3.2f)", adjustment_x, adjustment_y) +
                        " | calculated pose: (" + String.format("%3.2f", (drive.pose.position.x - adjustment_x)) + "," +
                        String.format("%3.2f", (drive.pose.position.y + adjustment_y)) + "," + adjustment_y + ")"
                );

                double x_position = Range.clip(drive.pose.position.x - adjustment_x, -58.5, -56.5);

                if (180 - Math.abs(Math.toDegrees(drive.pose.heading.toDouble())) > 3.0) {
                    adjustment_y = 0.0;
                }

                double y_position = drive.pose.position.y + adjustment_y;
                if (Globals.COLOR == AlliancePosition.RED) {
                    y_position = Range.clip(y_position, -15.0, -9.0);
                } else {
                    y_position = Range.clip(y_position, 9.0, 15.0);
                }

                Pose2d proposedPose = new Pose2d(x_position, y_position, stackPose.heading.toDouble());

                Log.d("StackIntakePosition_Logger", "Final adjustment (adjustment_x, adjustment_y: " +
                        String.format("(%3.2f,%3.2f)", adjustment_x, adjustment_y));
                Log.d("StackIntakePosition_Logger", "Proposed Stack Pose: " + new PoseMessage(proposedPose)
                        + " | target stack pose: " + new PoseMessage(stackPose));

                AutoBase.setStackPositionStatic(proposedPose);

                y_adjustment = adjustment_y / 2.0;

                return false;
            }

            if (counter >= 1 && stackDistanceLeft > 6.0 && stackDistanceLeft < 16.0) {
                sensorDistancesLeftList.add(stackDistanceLeft);
            }

            if (counter >= 1 && stackDistanceRight > 6.0 && stackDistanceRight < 16.0) {
                sensorDistancesRightList.add(stackDistanceRight);
            }

            try {
                Log.d("StackIntakePosition_Logger", "Current Drive Pose: " + new PoseMessage(drive.pose)
                        + " | Target Stack Pose: " + new PoseMessage(stackPose) + " | count: " + counter
                        + " | stackDistance Left (in): " + String.format("%3.2f", stackDistanceLeft)
                        + " | stackDistance Right (in): " + String.format("%3.2f", stackDistanceRight)
                        + " | stackDistance Delta (in): " + String.format("%3.2f", (stackDistanceLeft - stackDistanceRight))
                        + " | Elapsed time: " + (System.currentTimeMillis() - startTime)
                );
            } catch (Exception e) {
                Log.d("StackIntakePosition_Logger", "Exception 1:  " + e.getMessage());
            }

            return true;
        }
    }

    public Pose2d getStackPosition() {
        return this.stackPosition;
    }

    public final static void setStackPositionStatic(Pose2d stackPosition) {
        AutoBase.stackPosition = stackPosition;
    }

    public Action driveToStack() {
        return new NullAction();
    }

    public Action driveToBackdrop() {
        return new NullAction();
    }

    protected static Vector2d backdropAdjustment = new Vector2d(0.0, 0.0);

    public static class BackdropRelocalizationAction implements Action {
        MecanumDrive drive;
        Outtake outtake;
        Pose2d backdropPose;
        MovingArrayList backdropDistanceList = new MovingArrayList(5);
        boolean firstTime = true;
        ElapsedTime timer = null;
        int counter = 0;

        public BackdropRelocalizationAction(MecanumDrive drive, Outtake outtake, Pose2d backdropPose) {
            this.drive = drive;
            this.outtake = outtake;
            this.backdropPose = backdropPose;
        }

        @Override
        public boolean run(TelemetryPacket packet) {

            if (timer == null) {
                timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
            }
            if (counter++ >= 3) {
                double avg_distance = backdropDistanceList.getAvg();

                double base_distance = 26.0;

                double adjustment = Range.clip((base_distance - avg_distance) * 0.5, -1.25, 1.25);

                Pose2d currentPose = drive.pose;
                drive.pose = new Pose2d(currentPose.position.plus(new Vector2d(adjustment, 0)), currentPose.heading);
                drive.updatePoseEstimate();

                Log.d("BackdropDistance_Logger", "Current Drive Pose: " + new PoseMessage(currentPose)
                        + " | avg backdrop distance: " + String.format("%3.2f", avg_distance)
                        + " | new drive pose: " + new PoseMessage(drive.pose)
                        + " | adjustment: " + String.format("%3.2f", adjustment)
                        + " | Target backdrop Pose: " + new PoseMessage(backdropPose));
                return false;
            }
            double distance = outtake.getBackdropDistance();
            if (distance < 28.5 && distance > 23.0) {
                backdropDistanceList.add(distance);
            }

            Log.d("BackdropDistance_Logger", "Count: " + counter + " | Current Drive Pose: " + new PoseMessage(drive.pose)
                    + " | backdrop distance: " + String.format("%3.2f", distance)
                    + " | Target backdrop Pose: " + new PoseMessage(backdropPose)
                    + " | elapsed time: " + String.format("%3.2f", timer.milliseconds())
            );

            return true;
        }
    }

    protected boolean pidDriveActivated = false;
    protected boolean pidDriveStarted = false;
    protected double straightDistance = 0.0;

    ElapsedTime pidDriverTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    private void pidDriveUpdate() {

        if (pidDriveActivated && Math.abs(straightDistance) > 0.0) {
            double slidePivotVoltage = outtake.getSlidePivotServoVoltage();
            if (!pidDriveStarted) {
                pidDriveStraight.setTargetPosition((int) (straightDistance / MecanumDrive.PARAMS.inPerTick));
                pidDriveStarted = true;
                Log.d("backdrop_pidDriveUpdate_logger", "straightDistance: "
                        + straightDistance + " | pidDriveActivated: " + pidDriveActivated
                        + " | pidDriveStarted: " + pidDriveStarted + " | isBusy: " + pidDriveStraight.isBusy() +
                        "| drive pose: " + new PoseMessage(drive.pose));

                pidDriveStraight.update();
            }

            if (pidDriveStarted && pidDriveStraight.isBusy()) {
                pidDriveStraight.update();
                Log.d("Backdrop_distance_Logger", "Update() called. Adjustment: " + straightDistance +
                        "| slide voltage: " + String.format("%3.2f", slidePivotVoltage));
            }

            if (!pidDriveStraight.isBusy()) {
                Log.d("Backdrop_distance_Logger", "End of Adjustment: " + straightDistance +
                        "| ending slide voltage: " + String.format("%3.2f", slidePivotVoltage) +
                        "| end drive pose: " + new PoseMessage(drive.pose));

                pidDriveActivated = false;
                pidDriveStraight.resetStartTime();
            }
        }
    }

    protected Action getBackdropDistanceAdjustmentAction() {
        return
                new ActionUtil.RunnableAction(() -> {
                    pidDriveActivated = true;
                    pidDriveStarted = false;
                    straightDistance = 0.0;
                    double slidePivotVoltage = outtake.getSlidePivotServoVoltage();
                    if (outtake.hasOuttakeReached()) {
                        if (slidePivotVoltage > Outtake.SLIDE_PIVOT_DUMP_VOLTAGE_SUPER_MAX + 0.15) {
                            straightDistance = 0.65;
                        } else if (slidePivotVoltage > Outtake.SLIDE_PIVOT_DUMP_VOLTAGE_SUPER_MAX) {
                            straightDistance = 0.35;
                        }
                    } else {
                        straightDistance = -1.15;
                    }

                    Log.d("Backdrop_distance_Logger", "starting Adjustment: " + straightDistance +
                            "| starting slide voltage: " + String.format("%3.2f", slidePivotVoltage) +
                            " | drive pose: " + new PoseMessage(drive.pose));
                    return false;
                });
    }

    public class PreloadPositionDetectionAction implements Action {
        MecanumDrive drive;

        MovingArrayList preloadLeftZoneList = new MovingArrayList(5);
        MovingArrayList preloadRightZoneList = new MovingArrayList(5);

        boolean firstTime = true;
        ElapsedTime timer = null;
        int counter = 0;

        public PreloadPositionDetectionAction(MecanumDrive drive) {
            this.drive = drive;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            if(firstTime) {
                drive.updatePoseEstimate();
                Log.d("Preload_detection_logger", "Drive Pose: " + new PoseMessage(drive.pose));
                firstTime = false;
                timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
            }

            if(timer.milliseconds() < 200.0 && (preloadPipeline.leftZoneAverage < 20 || preloadPipeline.rightZoneAverage< 20)) {
                return true;
            }

            if(counter++ > 2) {
                double leftMean = preloadLeftZoneList.getMean();
                double rightMean = preloadRightZoneList.getMean();

                Log.d("Preload_detection_logger", "Preload LEFT Zone MEAN: " + leftMean);
                Log.d("Preload_detection_logger", "Preload RIGHT Zone MEAN: " + rightMean);
                if(leftMean > 25 && rightMean > 25) {
                    AutoBase.preloadPosition = (leftMean > (rightMean +25))? Side.LEFT: Side.RIGHT;
                }

                try {
                    backVisionPortal.setProcessorEnabled(aprilTag, false);
                    backVisionPortal.setProcessorEnabled(preloadPipeline, false);
                    backVisionPortal.close();
                } catch(Exception e) {
                    Log.e("AutoBase_Preload_logger", e.getLocalizedMessage());
                }
                return false;
            }

            if(preloadPipeline != null) {
                preloadLeftZoneList.add(preloadPipeline.leftZoneAverage);
                preloadRightZoneList.add(preloadPipeline.rightZoneAverage);
                Log.d("Preload_detection_logger", "Count " + counter + " | Number of Detections: " + preloadPipeline.numOfDetections
                        + " | Target ID: " + preloadPipeline.getTargetAprilTagID());
                Log.d("Preload_detection_logger", "Preload LEFT Zone AVG: " + preloadPipeline.leftZoneAverage +
                                                         " | Preload RIGHT Zone AVG: " + preloadPipeline.rightZoneAverage +
                                                         " | Preload detected raw zone: " + preloadPipeline.getPreloadedZone());
            }
            return true;
        }
    }

    public class EnableVisionProcessorAction implements Action {
        MecanumDrive drive;

        Side spikeLocation;
        public EnableVisionProcessorAction(MecanumDrive drive, Side spikeLocation) {
            this.drive = drive;
            this.spikeLocation = spikeLocation;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            drive.updatePoseEstimate();
            Log.d("Enabled_Vision_Processor_logger", "Drive Pose: " + new PoseMessage(drive.pose));
            if(preloadPipeline != null) {
                backVisionPortal.setProcessorEnabled(aprilTag, true);
                backVisionPortal.setProcessorEnabled(preloadPipeline, true);
                preloadPipeline.setTargetAprilTagID(spikeLocation);
            }
            return false;
        }
    }

}

