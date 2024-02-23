package org.firstinspires.ftc.teamcode.opmode.teleops;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.messages.PoseMessage;
import org.firstinspires.ftc.teamcode.subsystem.AprilTag;
import org.firstinspires.ftc.teamcode.subsystem.Hang;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Memory;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;
import org.firstinspires.ftc.teamcode.subsystem.Drone;
import org.firstinspires.ftc.teamcode.utils.software.ActionScheduler;
import org.firstinspires.ftc.teamcode.utils.hardware.GamePadController;
import org.firstinspires.ftc.teamcode.utils.software.ActionUtil;
import org.firstinspires.ftc.teamcode.utils.software.AutoActionScheduler;
import org.firstinspires.ftc.teamcode.utils.software.DriveWithPID;
import org.firstinspires.ftc.teamcode.utils.software.SmartGameTimer;
import org.firstinspires.ftc.teamcode.wolfdrive.WolfDrive;

import java.util.Arrays;

@Config
@TeleOp(group = "Drive", name = "Manual Drive")
public class ManualDrive extends LinearOpMode {
    public static double TURN_SPEED = 0.75;
    public static double DRIVE_SPEED = 1.0;
    public static double SLOW_TURN_SPEED = 0.3;
    public static double SLOW_DRIVE_SPEED = 0.3;

    public static double STRAFE_DISTANCE = 1.25;
    private SmartGameTimer smartGameTimer;
    private GamePadController g1, g2;
    private MecanumDrive drive;
    private ActionScheduler sched;
    private AutoActionScheduler autoRunSched;
    private Intake intake;
    private Outtake outtake;
    private Hang hang;
    private Drone drone;

    private boolean useWolfDrive = true;

//    private AprilTag aprilTag;

    Long intakeReverseStartTime = null;
    Long intakeSlowdownStartTime = null;
    Long lastTimePixelDetected = null;
    boolean isPixelDetectionEnabled = true;
    Boolean hasBackdropTouched = null;
    int prevPixelCount = 0;
    VelConstraint velConstraintOverride;
    AccelConstraint accelConstraintOverride = new ProfileAccelConstraint(-30.0, 30.0);

    double slowModeForHanging = 0.35;
    double slowModeForBackdrop = 0.5;
    boolean isHangingActivated = false;

    boolean isDroneLaunched = false;
    long startTime = 0L;

    boolean pixelScored = false;

    private Double start_y = null;
    boolean strafeToAlign = false;
    public static boolean logLoopTime = false;

    boolean prevBackdropTouched = false;

    Boolean firstTimeSlideOut = null;

    Boolean isFixerServoOut = Boolean.FALSE;

    Boolean startFixedReset = null;

    private DriveWithPID pidDriveStrafe;

    private DriveWithPID pidDriveStraight;

    private WolfDrive wolfDrive;

    private ElapsedTime fixerDriveTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    Gamepad.LedEffect redEffect = new Gamepad.LedEffect.Builder()
            .addStep(1, 0, 0, 750) // Show red for 250ms
            .build();

    Gamepad.LedEffect greenEffect = new Gamepad.LedEffect.Builder()
            .addStep(0, 1, 0, 750) // Show green for 250ms
            .build();

    Gamepad.LedEffect whiteEffect = new Gamepad.LedEffect.Builder()
            .addStep(1, 1, 1, 750) // Show white for 250ms
            .build();

    private boolean resetSliderEnabled = false;

    @Override
    public void runOpMode() throws InterruptedException {

        long start_time = System.currentTimeMillis();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("Initializing ManualDrive ...");
        telemetry.update();

        // Init
        g1 = new GamePadController(gamepad1);
        g2 = new GamePadController(gamepad2);
        g1.update();
        g2.update();
        sched = new ActionScheduler();
        drive = new MecanumDrive(hardwareMap, Memory.LAST_POSE, false);
        intake = new Intake(hardwareMap, false);
        outtake = new Outtake(hardwareMap, false);
        hang = new Hang(hardwareMap);
        drone = new Drone(hardwareMap);
//        aprilTag = new AprilTag(hardwareMap, telemetry);

        wolfDrive = new WolfDrive(drive);

        smartGameTimer = new SmartGameTimer(false);

        if (Globals.RUN_AUTO) {
            drive.pose = Globals.drivePose;
        }
        outtake.isAuto = false;
        Globals.RUN_AUTO = false;

        Log.d("ManualDrive_Logger", "Robot init Pose: " + new PoseMessage(drive.pose));
        Log.d("ManualDrive_Logger", "Slide Position: " + Globals.OUTTAKE_SLIDE_POSITION);

        intake.initialize(false);
        // Init opmodes
        outtake.initialize();
        drone.initialize();
        hang.initialize();

        drive.startIMUThread(this);

//        long current_time = System.currentTimeMillis();
//        telemetry.addLine("Initializing AprilTag: " + (current_time - start_time));
//        telemetry.update();
//        aprilTag.initialize();

        pidDriveStrafe = new DriveWithPID(drive, null, DriveWithPID.DriveDirection.STRAFE);
        pidDriveStraight = new DriveWithPID(drive, null, DriveWithPID.DriveDirection.STRAIGHT);

        velConstraintOverride = new MinVelConstraint(Arrays.asList(
                this.drive.kinematics.new WheelVelConstraint(45.0),
                new AngularVelConstraint(Math.PI / 2)));

        // Ready!
        telemetry.addLine("Manual Drive is Ready! Complete in " + (System.currentTimeMillis() - start_time) + " (ms)");
        telemetry.addLine("Drive Pose: " + new PoseMessage(drive.pose));
        telemetry.update();

        Action sliderAction = outtake.prepTeleop(Globals.OUTTAKE_SLIDE_POSITION);

        sched.queueAction(sliderAction);
        sched.update();

        Long aTimer = System.currentTimeMillis();
        while (opModeInInit() && !isStopRequested()) {
            if (aTimer != null && (System.currentTimeMillis() - aTimer > 1000)) {
                outtake.finishPrepTeleop();
                if(Globals.OUTTAKE_SLIDE_POSITION < 30) {
                    outtake.resetSlideEncoder();
                }
                aTimer = null;
            }
            idle();
        }

        startTime = System.currentTimeMillis();

        resetRuntime();
        g1.reset();
        g2.reset();

        Globals.OUTTAKE_SLIDE_POSITION = 0;

        // Main loop
        while (!isStopRequested() && opModeIsActive()) {
            start_time = System.currentTimeMillis();

            g1.update();
            g2.update();

            move();
            long current_time_0 = System.currentTimeMillis();
            logLoopTime("Move() elapsed time: ", (current_time_0 - start_time));

            subsystemControls();
            long current_time_1 = System.currentTimeMillis();
            logLoopTime("subsystemControls() elapsed time: ", (current_time_1 - current_time_0));

            pixelDetection();
            long current_time_2 = System.currentTimeMillis();
            logLoopTime("pixelDetection() elapsed time: ", (current_time_2 - current_time_1));

            drive.updatePoseEstimate();
            long current_time_3 = System.currentTimeMillis();
            logLoopTime("updatePoseEstimate() elapsed time: ", (current_time_3 - current_time_2));

            outtake.update();
            long current_time_4 = System.currentTimeMillis();
            logLoopTime("outtake.update() elapsed time: ", (current_time_4 - current_time_3));

            intake.update();
            long current_time_5 = System.currentTimeMillis();
            logLoopTime("intake.update() elapsed time: ", (current_time_5 - current_time_4));

            sched.update();
            long current_time_6 = System.currentTimeMillis();
            logLoopTime("sched.update() elapsed time: ", (current_time_6 - current_time_5));

            backdropTouchedDetection();
            long current_time_7 = System.currentTimeMillis();
            logLoopTime("backdropTouchedDetection elapsed time: ", (current_time_7 - current_time_6));

            telemetry.addData("Time left: ", smartGameTimer.formattedString() + " (" + smartGameTimer.status() + ")" + " Loop time: " +
                    (System.currentTimeMillis() - start_time) + " (ms)");
            //telemetry.addLine(intake.getStackServoPositions());
            telemetry.addLine("Current Pixel count: " + Intake.pixelsCount +
                    " | Total count: " + Intake.totalPixelCount + " | Prev count: " + prevPixelCount);
            telemetry.addLine(outtake.getServoPositions());
            telemetry.addLine("Intake state: " + intake.intakeState);
            telemetry.addLine(hang.getCurrentPosition());
            telemetry.addData("Slide_current_position", outtake.getMotorCurrentPosition());
            telemetry.addData("Slide_target_position", outtake.getMotorTargetPosition());
            telemetry.addData("Slide motor busy", outtake.isMotorBusy());
            telemetry.addData("backdrop distance", String.format("%3.2f",outtake.getBackdropDistance()));

            telemetry.update();

            if (System.currentTimeMillis() - start_time > 80) {
                Log.d("Loop_Logger", " --- Loop time: " + (System.currentTimeMillis() - start_time) + " ---");
            }

            int secondsLeft = 120 - (int)(smartGameTimer.seconds());
            if( secondsLeft == 30 ) {
                g1.rumble(300);
            }

            if(secondsLeft == 15 ) {
                g1.rumble(1000);
            }
        }

        Log.d("ManualDrive_Logger", " --- Total Pixels: " + this.intake.totalPixelCount + " ---");
        Log.d("ManualDrive_Logger", " --- Program ended at " + System.currentTimeMillis() + " ---");

        // On termination
        Memory.LAST_POSE = drive.pose;
    }

    private void backdropTouchedDetection() {
        if (outtake.hasOuttakeReached()) {
            if (!hasBackdropTouched) {
                g1.rumble(500);
                hasBackdropTouched = true;
            }
        } else {
            hasBackdropTouched = false;
        }
    }

    private void pixelDetection() {
        if (isPixelDetectionEnabled) {

            // if the 2nd pixel is too close to the 1st, reverse intake for a moment
            if (prevPixelCount != Intake.pixelsCount && lastTimePixelDetected != null) {
                long elapsedTime = System.currentTimeMillis() - lastTimePixelDetected.longValue();
                Log.d("TeleOps_Pixel_detection", prevPixelCount + "->" + Intake.pixelsCount + ", elapsed time (ms) " + elapsedTime);

                if (elapsedTime < 600) {
                    if (Intake.pixelsCount == 2) {
                        Intake.totalPixelCount--;
                        prevPixelCount = Intake.pixelsCount;

                        Log.d("TeleOps_Pixel_detection", "deduct 1 from the counts. Total: " + Intake.totalPixelCount + " current: " + Intake.pixelsCount);

                        sched.queueAction(intake.intakeReverse());

                        intakeSlowdownStartTime = new Long(System.currentTimeMillis());
                        lastTimePixelDetected = null;
                        Log.d("TeleOps_Pixel_detection", "slow down 2nd pixel at " + (intakeSlowdownStartTime - startTime));

                        g1.runLedEffect(whiteEffect);
                    }
                }

                // if 0-->1, slowdown a bit to avoid collision
                // the 2nd pixel could push the 1st, potentially it could mess up the transfer
                if (prevPixelCount == 0) {
                    sched.queueAction(intake.intakeSlowdown());
                }
            }
            // log the count
            if (prevPixelCount != Intake.pixelsCount && Intake.pixelsCount >= 2 && intakeReverseStartTime == null) {
                intakeReverseStartTime = new Long(System.currentTimeMillis());

                g1.runLedEffect(greenEffect);

                Log.d("TeleOps_Pixel_detection", "Pixel counted changed to "
                        + Intake.pixelsCount + ", need to reverse after intake, detected at " + (intakeReverseStartTime - startTime));
            }

            if (intakeSlowdownStartTime != null) {
                if ((System.currentTimeMillis() - intakeSlowdownStartTime.longValue()) > 500) {
                    sched.queueAction(intake.intakeOn());
                    // let's retake the pixel
                    // hope the pixel is moving down enough, so we recount
                    Intake.pixelsCount--;
                    prevPixelCount=Intake.pixelsCount;
                    intakeSlowdownStartTime = null;
                    intakeReverseStartTime = new Long(System.currentTimeMillis());
                }
            }

            // intake reverse is on
            if (intakeReverseStartTime != null && intakeSlowdownStartTime == null) {

                long elapsedTimeMs = System.currentTimeMillis() - intakeReverseStartTime.longValue();

                if(Intake.pixelsCount >2 && intake.intakeState.equals(Intake.IntakeState.ON)) {
                    sched.queueAction(intake.intakeReverse());
                    Log.d("TeleOps_Pixel_detection", "Pixel count changed to "
                            + Intake.pixelsCount + ", reversing right away at " + (intakeReverseStartTime - startTime));
                }
                else if (elapsedTimeMs > 800 && elapsedTimeMs < 1000 &&
                        intake.intakeState.equals(Intake.IntakeState.ON)) {
                    sched.queueAction(intake.intakeReverse());
                    Log.d("TeleOps_Pixel_detection", "Pixel count changed to "
                            + Intake.pixelsCount + ", reversing started at " + (intakeReverseStartTime - startTime));
                }

                if (elapsedTimeMs > 2800 &&
                        intake.intakeState.equals(Intake.IntakeState.REVERSING)) {
                    sched.queueAction(intake.intakeOff());
                    intakeReverseStartTime = null;

                    Log.d("TeleOps_Pixel_detection", "Intake reverse is completed.");
                }
            }

            // detect the first pixel in time
            if (Intake.pixelsCount != prevPixelCount) {
                lastTimePixelDetected = new Long(System.currentTimeMillis());
                Log.d("TeleOps_Pixel_detection", "Pixel count changed from " + prevPixelCount + " -> " + Intake.pixelsCount + " | detected at " + (lastTimePixelDetected - startTime));

                if(Intake.pixelsCount == 1) {
                    g1.rumble(200);
                }

                if(Intake.pixelsCount == 2) {
                    g1.rumble(750);
                }

                g1.runLedEffect(redEffect);
            }

            prevPixelCount = Intake.pixelsCount;
        }
    }

    private void move() {
        double speed = (1 - Math.abs(g1.right_stick_x)) * (DRIVE_SPEED - SLOW_DRIVE_SPEED) + SLOW_DRIVE_SPEED;

        if (isHangingActivated) {
            speed = speed * slowModeForHanging;
        }

        double input_x = Math.pow(-g1.left_stick_y, 3) * speed;
        double input_y = Math.pow(-g1.left_stick_x, 3) * speed;

        if ((isSlideOut || isFixerServoOut) && input_x < -0.05) {
            input_x = Range.clip(input_x, -0.25, -0.05);
        }

        // if outtake has touched the backdrop, don't move further
        if (outtake.hasOuttakeReached()) {
            if (input_x < -0.1) {
                input_x = 0.0;
                Log.d("Drive_power", String.format("input_x: %3.2f to 0.0", input_x) + String.format(" | input_y: %3.2f", input_y));
            }

//            if (outtake.checkSlidePivotOverreached()) {
//                input_x = 0.05;
//            }
        }

        prevBackdropTouched = outtake.hasOuttakeReached();

        double input_turn = Math.pow(g1.left_trigger - g1.right_trigger, 3) * TURN_SPEED;
        if (g1.leftBumper()) input_turn += SLOW_TURN_SPEED;
        if (g1.rightBumper()) input_turn -= SLOW_TURN_SPEED;

        Vector2d input = new Vector2d(input_x, input_y);

        if(autoBackdropAdjustmentActivated && input_x == 0.0 && input_y == 0.0 && input_turn == 0.0) {
            // do another
            // not set upset the power
        } else {
            // test wolfPack drive
            if(useWolfDrive) {
                PoseVelocity2d currentVel = drive.updatePoseEstimate();
                wolfDrive.trackPosition(drive.pose);
                wolfDrive.driveWithCorrection(new PoseVelocity2d(input, input_turn), currentVel);
            } else {
                drive.setDrivePowers(new PoseVelocity2d(input, input_turn));
            }
        }

        if(g2.xOnce()) {
            useWolfDrive = false;
        } else if(g2.yOnce()) {
            useWolfDrive = true;
        }

        if (input_x > 0.1) {
            if (isSlideOut && outtake.hasOuttakeReached() && outtake.getSlidePivotServoVoltage() < Outtake.SLIDE_PIVOT_DUMP_VOLTAGE_MAX) {
                outtake.backdropTouched = false;
            }
        }

        if (input_x > 0.5) {
            if (isSlideOut && Intake.pixelsCount == 0 && pixelScored) {
                retractSlide();
            } else if (isFixerServoOut && input_x > 0.65) {
                Log.d("ManualDrive_logger", " input_x: " + String.format("%3.2f", input_x) + " | g1.left_stick_y: " + g1.left_stick_y +
                        " | Elapsed time: " + fixerDriveTimer.milliseconds());
                if(startFixedReset == null) {
                    fixerDriveTimer.reset();
                    startFixedReset = Boolean.TRUE;
                }

                if(startFixedReset && fixerDriveTimer.milliseconds() > 120.0) {
                    startFixedReset = null;
                    isFixerServoOut = false;
                    sched.queueAction(outtake.resetOuttakeFixerServo());
                }
            }
        } else {
            startFixedReset = null;
        }

        if(Math.abs(input_y) > 0.1 && isSlideOut && outtake.hasOuttakeReached()) {
            sched.queueAction(outtake.strafeToAlign());
            manualStrafeToScore = true;
        } else if(isSlideOut && input_y == 0.0 && manualStrafeToScore){
            sched.queueAction(outtake.prepareToScore());
            manualStrafeToScore = false;
        }

        if(input_x != 0.0 ) {
            autoBackdropDistance = false;
        }

        telemetry.addData("drive_power", "input_x: %3.2f | input_y: %3.2f | speed: %3.2f", input_x, input_y, speed);

        if(outtake.backdropTouched && Math.abs(input_turn) > 0.0) {
            Log.d("ManualDrive_logger", "turn_power: " + String.format("%3.2f", input_turn));
        }
    }

    boolean isSlideOut = false;
    boolean isAprilTagDetected = false;
    boolean isStackIntakeOn = false;
    boolean manualStrafeToScore = false;

    private void subsystemControls() {
        // Intake controls
        //
        //   xOnce for toggle intake ON/OFF
        //
        if (g1.aLong()) {
            isPixelDetectionEnabled = false;
        } else if (!g1.start() && g1.xOnce()) {
            if (intake.intakeState == Intake.IntakeState.ON) {
                sched.queueAction(intake.intakeOff());
            } else {
                Intake.pixelsCount = 0;
                sched.queueAction(new SequentialAction(
                        intake.intakeOn(),
                        outtake.prepareToTransfer()));
            }
        }

        if (g1.b() && !g1.start()) {
            if (isSlideOut) {
                retractSlide();
            } else {
                sched.queueAction(intake.intakeReverse());
            }
        }
        if (!g1.b() && intake.intakeState == Intake.IntakeState.REVERSING && intakeReverseStartTime == null) {
            sched.queueAction(intake.intakeOff());
        }

        // Outtake controls
        if (g1.yLong()) {
            sched.queueAction(outtake.latchScore2());
            pixelScored = true;
        } else if (g1.yOnce()) {
            if (isSlideOut) {
                if (outtake.latchState == Outtake.OuttakeLatchState.LATCH_1) {
                    sched.queueAction(outtake.latchScore2());
                    pixelScored = true;
                } else {
                    sched.queueAction(outtake.latchScore1());
                    if(Intake.pixelsCount == 0) {
                        pixelScored = true;
                    }
                }
            } else {
                isSlideOut = true;
                if (isFixerServoOut) {
                    sched.queueAction(outtake.resetOuttakeFixerServo());
                    isFixerServoOut = false;
                }
                sched.queueAction(intake.intakeOff());
                sched.queueAction(outtake.prepareToSlide());
                sched.queueAction(outtake.extendOuttakeTeleOps());
                sched.queueAction(new SleepAction(0.5));
                sched.queueAction(outtake.prepareToScore());
//                sched.queueAction(aprilTag.updatePosition());

                if (firstTimeSlideOut == null) {
                    sched.queueAction(new SleepAction(0.5));
                    sched.queueAction(outtake.resetSlidePivotServoDumpVoltageLimit());
                    firstTimeSlideOut = false;
                }
                isAprilTagDetected = true;
            }
        }

        if(!outtake.backdropTouched || !isSlideOut) {
            autoBackdropDistance = false;
        }

        if (isSlideOut && isAprilTagDetected && AprilTag.yaw != null) {
//            isAprilTagDetected = false;
//            final double yaw = AprilTag.yaw.doubleValue();
//            Log.d("AprilTag_Localization", String.format("%3.2f", yaw));

//            AutoActionScheduler autoActionSched = new AutoActionScheduler(this::update);
//            autoActionSched.addAction(drive.actionBuilder(drive.pose)
//                    .turn(Math.toRadians(yaw))
//                    .build());
//
//            autoActionSched.run();
//            AprilTag.yaw = null;
        }

        if (Math.abs(g1.right_stick_y) > 0.03 && isSlideOut) {
            sched.queueAction(outtake.moveSliderBlocking(-g1.right_stick_y));
        }

//        if (Math.abs(g1.right_stick_y) > 0.03) {
//            outtake.slidePIDEnabled = false;
//            outtake.setSlidePower(-g1.right_stick_y);
//        } else if (!outtake.slidePIDEnabled) {
//            outtake.slidePIDEnabled = true;
//            sched.queueAction(outtake.lockPosition());
//        }

        // Hang arms up/down
        if(g1.dpadUpLong()) {
            if(isFixerServoOut) {
                sched.queueAction(outtake.resetOuttakeFixerServo());
                isFixerServoOut = false;
            }

            if (isHangingActivated) {
                sched.queueAction(hang.hang());
            }
        }
        else if (g1.dpadUpOnce()) {
            if(!isSlideOut && !isDroneLaunched) {
                if (!isFixerServoOut) {
                    sched.queueAction(outtake.moveUpOuttakeFixerServo());
                    startFixedReset = null;
                } else {
                    sched.queueAction(outtake.moveUpOuttakeFixerServoSlowly());
                }
                isFixerServoOut = true;
            }
        }

        if (g1.dpadDownLong()) {
            if (isHangingActivated) {
                sched.queueAction(hang.hang());
            }
            else if(isFixerServoOut) {

                double forwardDistance =-0.85;
                drive.updatePoseEstimate();

                int level = outtake.getFixerServoLevel().level;
                int nextLevel1 = level + 10;
                int nextLevel2 = level + 15;
                if(nextLevel1 > Outtake.FixerServoPosition.MAX_FIXER_LEVEL) {
                    nextLevel1 = Outtake.FixerServoPosition.MAX_FIXER_LEVEL;
                }

                if(nextLevel2 > Outtake.FixerServoPosition.MAX_FIXER_LEVEL) {
                    nextLevel2 = Outtake.FixerServoPosition.MAX_FIXER_LEVEL;
                }

                double nextLevelPosition1 = outtake.getFixerServoPositionByLevel(nextLevel1);
                double nextLevelPosition2 = outtake.getFixerServoPositionByLevel(nextLevel2);

                Log.d("DriveWithPID_Logger_0_Teleops", "Pose before straight forward: " + new PoseMessage(this.drive.pose) + " | target=" + forwardDistance);
                start_y = drive.pose.position.y;

                pidDriveStraight.setMaxPower(0.35);

                AutoActionScheduler autoActionSched = new AutoActionScheduler(this::update,hardwareMap);
                autoActionSched.addAction(
                        new ParallelAction(
                            pidDriveStraight.setTargetPositionActionBlocking((int) (forwardDistance / MecanumDrive.PARAMS.inPerTick)),
                            new SequentialAction(
                                    new SleepAction(0.15),
                                new ActionUtil.ServoPositionAction(outtake.getOuttakeFixerServo(),nextLevelPosition1,"outtake_fixer"),
                                    new SleepAction(0.1),
                                    new ActionUtil.ServoPositionAction(outtake.getOuttakeFixerServo(),nextLevelPosition2,"outtake_fixer")
                            )
                        )
                );

                autoActionSched.addAction(
                        new MecanumDrive.DrivePoseLoggingAction(drive,"fixer_move_action")
                );

                autoActionSched.run();
            }
        } else if (g1.dpadDownOnce()) {
            if (isHangingActivated) {
                sched.queueAction(hang.hangSlowly());
            }
            else if(!isSlideOut){
                sched.queueAction(outtake.moveDownOuttakeFixerServoSlowly());
            }
        }

        if (start_y != null) {
            this.drive.updatePoseEstimate();
            Log.d("ManualDrive", "Pose after strafe: " + new PoseMessage(this.drive.pose) + " | actual=" + (drive.pose.position.y - start_y));
            start_y = null;
        }

        if(outtake.backdropTouched && !autoBackdropDistance && isSlideOut &&
                outtake.getSlidePivotServoVoltage() > Outtake.SLIDE_PIVOT_DUMP_VOLTAGE_SUPER_MAX
                && !pidDriveStraight.isBusy()
        && g1.left_stick_x == 0.0 && g1.left_stick_y == 0.0 ) {
            autoBackdropDistance = true;

            double forwardDistance =0.32;
            double voltage = outtake.getSlidePivotServoVoltage();
            if(voltage > Outtake.SLIDE_PIVOT_DUMP_VOLTAGE_EXTREME) {
                forwardDistance =0.5;
            }

            Log.d("DriveWithPID_Logger_0_Teleops", "Pose before straight forward: "
                    + new PoseMessage(this.drive.pose) + " | target=" + forwardDistance + " | voltage: " + String.format("%3.2f", voltage));
            drive.updatePoseEstimate();
            start_y = drive.pose.position.y;

            AutoActionScheduler autoActionSched = new AutoActionScheduler(this::update,hardwareMap);
            autoActionSched.addAction(
                        pidDriveStraight.setTargetPositionActionBlocking((int) (forwardDistance / MecanumDrive.PARAMS.inPerTick))
                );
            autoActionSched.addAction(
                    new MecanumDrive.DrivePoseLoggingAction(drive,"aut_adjust_backdrop_distance")
            );

            autoActionSched.run();
        }

        int multiplier = (Globals.RUN_AUTO) ? 1 : -1;

        if(g1.dpadLeftLong() || g1.dpadRightLong() ) {
            multiplier = multiplier*2;
        }
        // move left and right by one slot
        if ((g1.dpadLeft() || g1.dpadRight()) && !pidDriveStrafe.isBusy()) {

            double strafeDistance = (g1.dpadLeft() ? STRAFE_DISTANCE : -STRAFE_DISTANCE) * multiplier;

            Log.d("DriveWithPID_Logger_0_Teleops", "Pose before strafe: " + new PoseMessage(this.drive.pose) + " | target=" + strafeDistance + " | multiplier=" + multiplier);
            start_y = drive.pose.position.y;

            AutoActionScheduler autoActionSched = new AutoActionScheduler(this::update,hardwareMap);
            if (isSlideOut) {
                autoActionSched.addAction(
                        new SequentialAction(
                                outtake.strafeToAlign(),
                                new SleepAction(0.2),
                                pidDriveStrafe.setTargetPositionActionBlocking((int) (strafeDistance / MecanumDrive.PARAMS.inPerTick)),
                                outtake.prepareToScore())
                );
            } else {
                autoActionSched.addAction(
                        pidDriveStrafe.setTargetPositionActionBlocking((int) (strafeDistance / MecanumDrive.PARAMS.inPerTick))
                );
            }

            autoActionSched.run();
        }

        // drone launch
        if (g1.backOnce()) {
            if(!isDroneLaunched) {
                isDroneLaunched=true;
                sched.queueAction(drone.scoreDrone());
            } else if(!isHangingActivated) {
                sched.queueAction(hang.unblockHook());
                sched.queueAction(new SleepAction(0.25));
                sched.queueAction(outtake.outtakeWireForHanging());
                isHangingActivated = true;
            } else if(isHangingActivated) {
                sched.queueAction(outtake.hangingHookSafeDown());
                isHangingActivated = false;
            }
        }

        // stack intake
        if (g1.start() && g1.x()) {
            if (!isStackIntakeOn) {
                isStackIntakeOn = true;
                sched.queueAction(intake.intakeTwoStackedPixels2());
            }
        }

        if (g1.aOnce()) {
            isStackIntakeOn = false;
            if (intake.stackIntakeState == Intake.StackIntakeState.DOWN) {
                sched.queueAction(new SequentialAction(
                        intake.prepareTeleOpsIntake(),
                        intake.intakeOff()));
            } else {
                Intake.pixelsCount = 0;
                sched.queueAction(
                        new SequentialAction(
                                outtake.prepareToTransfer(),
                                intake.prepareStackIntake(),
                                intake.intakeOn()));
            }
        }

        if (g1.start() && g1.guideOnce()) {
            sched.queueAction(outtake.resetSliderZeroPosition());
        } else if (g1.guideOnce()) {
//            sched.queueAction(outtake.reverseDump());
//            if (isHangingActivated) {
//                Log.d("Hang_drop_down", "Hang current position: " + hang.getCurrentPosition());
//                hang.dropdownFromHang();
//                long startTime = System.currentTimeMillis();
//                while (hang.isMotorBusy() && (System.currentTimeMillis() - startTime) < 2000) {
//                    idle();
//                }
//
//                Log.d("Hang_drop_down", "Hang after position: " + hang.getCurrentPosition());
//            } else {
//                sched.queueAction(outtake.reverseDump());
//            }
        }
    }

    private void field_centric_move() {
        double speed = (1 - Math.abs(g1.right_stick_x)) * (DRIVE_SPEED - SLOW_DRIVE_SPEED) + SLOW_DRIVE_SPEED;
        double input_x = Math.pow(-g1.left_stick_y, 3) * speed * (isHangingActivated ? slowModeForHanging : 1.0);
        double input_y = Math.pow(-g1.left_stick_x, 3) * speed * (isHangingActivated ? slowModeForHanging : 1.0);
        Vector2d input = new Vector2d(input_x, input_y);
        input = drive.pose.heading.times(input);

        double input_turn = Math.pow(g1.left_trigger - g1.right_trigger, 3) * TURN_SPEED;
        if (g1.leftBumper()) input_turn += SLOW_TURN_SPEED;
        if (g1.rightBumper()) input_turn -= SLOW_TURN_SPEED;

        drive.setDrivePowers(new PoseVelocity2d(input, input_turn));
    }

    final public void update() {
        outtake.update();
        telemetry.update();
    }

    private void retractSlide() {
//        sched.queueAction(new SequentialAction(outtake.latchScore0(), new SleepAction(0.2)));
        sched.queueAction(outtake.retractOuttake());
        sched.queueAction(new SleepAction(0.2));
        sched.queueAction(new ActionUtil.RunnableAction(() -> {
            isSlideOut = false;
            pixelScored = false;
            return false;
        }));
    }

    private void logLoopTime(String msg, long elapsedTime) {
        if (logLoopTime || elapsedTime > 50) {
            Log.d("Loop_Logger", msg + " " + elapsedTime);
        }
    }
//    private class AutoDriveStraightAction implements Action {
//        MecanumDrive drive;
//        Servo servo;
//
//        double increment = 0.0;
//        double maxPosition = 0.0;
//
//        double power;
//        long elapsedTime;
//        Long startTime = null;
//
//        public AutoDriveStraightAction(MecanumDrive drive, double power, Servo servo, double increment, double maxPosition, long elapsedTime) {
//            this.drive = drive;
//            this.power = power;
//            this.servo = servo;
//            this.increment = increment;
//            this.maxPosition = maxPosition;
//            this.elapsedTime = elapsedTime;
//        }
//
//        @Override
//        public boolean run(TelemetryPacket packet) {
//
//            if(startTime == null) {
//                startTime = System.currentTimeMillis();
//                Log.d("AutoDriveStraight_Logger", "Motor Start time: " + startTime + ", power: " + String.format("%3.2f", power));
//
//                if(servo != null) {
//                    double currentPosition = servo.getPosition();
//                    double targetPosition = currentPosition + increment;
//                    Log.d("AutoDriveStraight_Logger", "Servo Start time: " +startTime + ", position: " + String.format("%3.2f", targetPosition));
//                }
//            }
//
//            if (this.drive != null ) {
//                Vector2d input = new Vector2d(power, 0.0);
//                drive.setDrivePowers(new PoseVelocity2d(input, 0.0));
//
//                if (System.currentTimeMillis() - startTime > elapsedTime) {
//                    Log.d("AutoDriveStraight_Logger", "Motor End time: " + System.currentTimeMillis() + ", power: " + String.format("%3.2f", power));
//                    drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0));
//                    return false;
//                }
//            }
//
//            if(servo != null) {
//                double currentPosition = servo.getPosition();
//                double targetPosition = currentPosition + increment;
//
//                if( (increment > 0 && targetPosition < maxPosition) ||
//                        (increment < 0 && targetPosition > maxPosition) ) {
//                    servo.setPosition(targetPosition);
//                }
//
//                if((System.currentTimeMillis() - startTime > elapsedTime)) {
//                    if(drive != null) {
//                        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0));
//                    }
//
//                    Log.d("AutoDriveStraight_Logger", "Servo End time: " + System.currentTimeMillis() + ", position: " + String.format("%3.2f", targetPosition));
//
//                    return false;
//                }
//            }
//            return true;
//        }
//    }
//
//    private class AutoBackdropDistanceAdjustmentAction implements Action {
//        MecanumDrive drive;
//        Outtake outtake;
//
//        double power;
//        long elapsedTime;
//        Long startTime = null;
//        boolean firstTime = true;
//
//        double voltage_limit = Outtake.SLIDE_PIVOT_DUMP_VOLTAGE_SUPER_MIN;
//
//        public AutoBackdropDistanceAdjustmentAction(MecanumDrive drive, double power, Outtake outtake, long elapsedTime) {
//            this.drive = drive;
//            this.power = power;
//            this.outtake = outtake;
//            this.elapsedTime = elapsedTime;
//        }
//
//        @Override
//        public boolean run(TelemetryPacket packet) {
//
//            if(!autoBackdropDistance) {
//                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0));
//                autoBackdropAdjustmentActivated = false;
//                return false;
//            }
//
//            if(firstTime) {
//                firstTime = false;
//                Log.d("AutoBackdropDistanceAdjustment_Logger", "Slide position monitoring: " + System.currentTimeMillis()
//                        + " | outtake reached: " + outtake.hasOuttakeReached()
//                        + ", voltage: " + String.format("%3.2f", outtake.getSlidePivotServoVoltage()));
//            }
//
//            if(outtake.backdropTouched ) {
//                if(startTime == null) {
//                    startTime = System.currentTimeMillis();
//                    Log.d("AutoBackdropDistanceAdjustment_Logger", "Motor Start time: " + startTime + ", power: " + String.format("%3.2f", power));
//
//                    if(outtake != null) {
//                        Log.d("AutoBackdropDistanceAdjustment_Logger", "Servo Start time: "
//                                + startTime
//                                + " | outtake reached: " + outtake.hasOuttakeReached()
//                                + ", voltage: " + String.format("%3.2f", outtake.getSlidePivotServoVoltage()));
//                    }
//                }
//            }
//
//            if (this.drive != null ) {
//                if (autoBackdropDistance &&
//                        ( (startTime!= null && System.currentTimeMillis() - startTime > elapsedTime) ||
//                                (outtake.getSlidePivotServoVoltage() < voltage_limit && outtake.backdropTouched))) {
//                    Log.d("AutoBackdropDistanceAdjustment_Logger", "Motor End time: " + System.currentTimeMillis()
//                            + ", power: " + String.format("%3.2f", power)
//                            + ", voltage: " + String.format("%3.2f", outtake.getSlidePivotServoVoltage())
//                            + " , outtake reached: " + outtake.hasOuttakeReached()
//                    );
//                    drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0));
//                    autoBackdropAdjustmentActivated = false;
//                    return false;
//                }
//
//                if(autoBackdropDistance && outtake.hasOuttakeReached() && isSlideOut) {
//                    Vector2d input = new Vector2d(power, 0.0);
//                    drive.setDrivePowers(new PoseVelocity2d(input, 0.0));
//                    autoBackdropAdjustmentActivated = true;
//
//                    Log.d("AutoBackdropDistanceAdjustment_Logger", "Motor power: "  + String.format("%3.2f", power)
//                            + ", voltage: " + String.format("%3.2f", outtake.getSlidePivotServoVoltage())
//                            + " , outtake reached: " + outtake.hasOuttakeReached()
//                    );
//                }
//            }
//
//            return true;
//        }
//    }

    protected boolean autoBackdropDistance = false;
    protected boolean autoBackdropAdjustmentActivated = false;

}

