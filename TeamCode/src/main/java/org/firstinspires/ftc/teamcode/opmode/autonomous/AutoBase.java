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

import org.apache.commons.math3.stat.StatUtils;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pipeline.AlliancePosition;
import org.firstinspires.ftc.teamcode.pipeline.ContourDetectionPipeline2;
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

import java.util.ArrayList;

@Config
public abstract class AutoBase extends LinearOpMode implements StackPositionCallback {
    protected MecanumDrive drive;
    protected Outtake outtake;
    protected Intake intake;
//    protected Vision vision;
    protected Drone drone;
    protected Hang hang;
    protected AutoActionScheduler sched;
    protected PropBasePipeline propPipeline;

    protected ContourDetectionPipeline2 stackPipeline;

    protected VisionPortal portal;
    public static Side side = Side.RIGHT;
    public static int SPIKE = 2;
    private GamePadController g1, g2;
    // configure a wait time to allow partner time to finish the backdrop
    //----------------------------------------------------------------
    public int farSideAutoWaitTimeInSeconds = 0;
    private int[] waitTimeOptions = {0,5,8};
    private int selectionIdx = 0;

    public static boolean displayDistanceSensor = true;

    protected static Pose2d stackPosition;
    ElapsedTime loopTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

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
        this.drone = new Drone(hardwareMap);
        this.hang = new Hang(hardwareMap);

        this.sched = new AutoActionScheduler(this::update);

        outtake.initialize();
        intake.initialize(true);
        drone.initialize();
        hang.initialize();

        if(getFieldPosition() == FieldPosition.NEAR) {
            outtake.setupForSlidingInAuto();
        }

        Globals.COLOR = getAlliance();

        if(getFieldPosition() == FieldPosition.NEAR) {
            propPipeline = new PropNearPipeline();
        } else {
            propPipeline = new PropFarPipeline();
        }

        stackPipeline = new ContourDetectionPipeline2();

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, Globals.FRONT_WEBCAM_NAME))
                .setCameraResolution(new Size(1920, 1080))
                .addProcessor(propPipeline)
                .addProcessor(stackPipeline)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

        loopTimer.reset();
        while(!portal.getCameraState().equals(VisionPortal.CameraState.STREAMING)) {
            telemetry.addLine(" Please wait, Webcam is streaming ... ");
            telemetry.update();
            idle();
        }

        double webCamReadyTime = loopTimer.milliseconds();

        portal.setProcessorEnabled(propPipeline, true);
        portal.setProcessorEnabled(stackPipeline, false);
        onInit();

        double maxLoopTime = 0.0;
        double minLoopTime = 100.0;
        loopTimer.reset();

        while (opModeInInit()) {
            double loopTimeBegin = loopTimer.milliseconds();
            g1.update();

            side = propPipeline.getLocation();

            SPIKE = side.ordinal();
            printDescription();
            telemetry.addLine("   ");
            telemetry.addLine(" <----- Team Prop Vision Detection -----> ");
            telemetry.addLine(" Wait a few seconds to capture the Maximum color value ");
            telemetry.addLine(" before placing the team prop on the field ");

            double loopTimeForProp = loopTimer.milliseconds();

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
                telemetry.addLine(   "    Wait Time to Score Yellow: " + farSideAutoWaitTimeInSeconds + " (seconds)");
            }

            double loopTimeForAfterProp = loopTimer.milliseconds();

            if(displayDistanceSensor) {
                telemetry.addLine(" Stack Distance sensor (in): " + String.format("%3.2f", intake.getStackDistance()));
            }

            double loopTimeForDistance = loopTimer.milliseconds();

            double loopTimeInMili = loopTimer.milliseconds();

            if(loopTimeInMili < minLoopTime) {
                minLoopTime = loopTimeInMili;
            } else if(loopTimeInMili > maxLoopTime) {
                maxLoopTime = loopTimeInMili;
            }

            telemetry.addLine(" Loop time: " + String.format("%.1f", loopTimeInMili)
                    + " | Min Loop: " + String.format("%.1f", minLoopTime) + " | Max Loop: " + String.format("%.1f", maxLoopTime) );

            telemetry.addLine(" Elapsed time, loop begin: " + String.format("%.1f", loopTimeBegin)+ " | For Prop: " + String.format("%.1f", loopTimeForProp)
                    + " | After Prop: " + String.format("%.1f", loopTimeForAfterProp) + " | For Distance: " + String.format("%.1f", loopTimeForDistance) );

            telemetry.addLine("Outtake Servo Positions: "+ outtake.getServoPositions());
            telemetry.addLine("Webcam ready time (ms):" + webCamReadyTime);

            loopTimer.reset();
            telemetry.update();
            idle();
        }

        // Auto start
        resetRuntime(); // reset runtime timer
        MecanumDrive.previousLogTimestamp = System.currentTimeMillis();
        MecanumDrive.autoStartTimestamp = System.currentTimeMillis();

        try {
            portal.setProcessorEnabled(propPipeline, false);
            portal.setProcessorEnabled(stackPipeline, false);
            //portal.close();
        } catch(Exception e) {
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

        try {
            portal.close();
        }catch(Exception e) {
            // ignore
        }

        drive.updatePoseEstimate();

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


    public static class StackIntakePositionAction implements Action {
        MecanumDrive drive;
        Intake intake;
        Pose2d stackPose;
        Boolean firstTime = null;

        ContourDetectionPipeline2 pipeline;
        VisionPortal portal;

        int counter = 0;
        long startTime = 0;

        ArrayList<Pose2d> positions = new ArrayList<>();

        ArrayList<Double> sensorDistances = new ArrayList<>();

        public StackIntakePositionAction(MecanumDrive drive, Intake intake, VisionPortal portal, ContourDetectionPipeline2 pipeline, Pose2d stackPose) {
            this.drive = drive;
            this.intake = intake;
            this.stackPose = stackPose;
            this.pipeline = pipeline;
            this.portal = portal;
        }

        @Override
        public boolean run(TelemetryPacket packet) {

            if(counter++ == 0) {
                startTime = System.currentTimeMillis();
                portal.setProcessorEnabled(this.pipeline, true);
            }

            if(counter > 5) {
                portal.setProcessorEnabled(this.pipeline, false);

                double avg_x = positions.stream()
                        .mapToDouble(obj ->obj.position.x).average().orElse(0.0);
                double avg_y = positions.stream()
                        .mapToDouble(obj ->obj.position.y).average().orElse(0.0);
                double avg_y_adj = sensorDistances.stream().mapToDouble(Double::doubleValue).average().orElse(0.0);

                Log.d("StackIntakePosition_Logger", "Current Drive Pose: " + new PoseMessage(drive.pose)
                        + " | Stack Target Pose: " + new PoseMessage(stackPose) + " | count: " + counter
                        + " | avg_stack_position (x,y, adj_y): (" + avg_x + "," + avg_y + "," + avg_y_adj + ")"
                );

                Pose2d proposedPose = stackPose;
                boolean isRed = (Globals.COLOR == AlliancePosition.RED);

                double adjustment_y = 0.0;
                double adjustment_x = 0.0;
                if(avg_y > 100) {
                    if (avg_x > 1350 && avg_x < 1450) {
                        adjustment_y = 1.0;
                    } else if (avg_x > 1450 || (avg_x < 1000 && avg_x > 800)) {
                        adjustment_y = 2.0;
                    } else if (avg_x < 1180 && avg_x > 1050) {
                        adjustment_y = -1.0;
                    } else if (avg_x <= 1050) {
                        adjustment_y = -2.0;
                    }
                }

                if(avg_y_adj > 10.0 && avg_y_adj < 20.0) {
                    adjustment_x = avg_y_adj - 5.5;
                }

                proposedPose = new Pose2d(drive.pose.position.x - adjustment_x, stackPose.position.y + adjustment_y , stackPose.heading.toDouble());

                Log.d("StackIntakePosition_Logger", "adjustment_x: " + adjustment_x + " | adjustment_y: " + adjustment_y);
                Log.d("StackIntakePosition_Logger", "Proposed Stack Pose: " + new PoseMessage(proposedPose) + " | target stack pose: " + new PoseMessage(stackPose));

                AutoBase.setStackPositionStatic(proposedPose);

                return false;
            }

            double stack_position_x = pipeline.getRectX()*1.0;
            double stack_position_y = pipeline.getRectY()*1.0;
            double stackDistanceY = intake.getStackDistance();

            if( counter >=2 && stack_position_x > 600 && stack_position_y > 100) {
                // this x,y are different than the field coordinates, will convert when the measurements are done
                // we take measurement 3 times, than use the average
                positions.add(new Pose2d(stack_position_x, stack_position_y, stackDistanceY));
            }

            if(counter >=2 && stackDistanceY > 8.0 && stackDistanceY < 20.0) {
                sensorDistances.add(stackDistanceY);
            }

            Log.d("StackIntakePosition_Logger", "Current Drive Pose: " + new PoseMessage(drive.pose)
                    + " | Target Stack Pose: " + new PoseMessage(stackPose) + " | count: " + counter
                    + " | stack_position (x,y): (" + stack_position_x + "," + stack_position_y + ")"
                    + " | stackDistance (in): " + stackDistanceY
                    + " | Elapsed time: " + (System.currentTimeMillis() - startTime)
            );

            return true;
        }
    }

    public Pose2d getStackPosition() {
        return this.stackPosition;
    }

    public void setStackPosition(Pose2d stackPosition) {
        AutoBase.stackPosition = stackPosition;
    }

    public final static void setStackPositionStatic(Pose2d stackPosition) {
        AutoBase.stackPosition = stackPosition;
    }

    public Action driveToStack() {
        return new NullAction();
    }
}

