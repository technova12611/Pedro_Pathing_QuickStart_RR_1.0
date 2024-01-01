package org.firstinspires.ftc.teamcode.opmode.teleops;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.messages.PoseMessage;
import org.firstinspires.ftc.teamcode.subsystem.Hang;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Memory;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;
import org.firstinspires.ftc.teamcode.subsystem.Drone;
import org.firstinspires.ftc.teamcode.utils.software.ActionScheduler;
import org.firstinspires.ftc.teamcode.utils.hardware.GamePadController;
import org.firstinspires.ftc.teamcode.utils.software.AutoActionScheduler;
import org.firstinspires.ftc.teamcode.utils.software.SmartGameTimer;

import java.util.Arrays;

@Config
@TeleOp(group = "Drive", name = "Manual Drive")
public class ManualDrive extends LinearOpMode {
    public static double TURN_SPEED = 0.75;
    public static double DRIVE_SPEED = 1.0;
    public static double SLOW_TURN_SPEED = 0.3;
    public static double SLOW_DRIVE_SPEED = 0.3;

    public static double STRAFE_DISTANCE = 3.0;

    private SmartGameTimer smartGameTimer;
    private GamePadController g1, g2;
    private MecanumDrive drive;
    private ActionScheduler sched;

    private AutoActionScheduler autoRunSched;
    private Intake intake;
    private Outtake outtake;
    private Hang hang;
    private Drone drone;

    Long intakeReverseStartTime = null;

    Long intakeSlowdownStartTime = null;

    Long lastTimePixelDetected = null;
    boolean isPixelDetectionEnabled = true;

    Boolean hasBackdropTouched = null;

    int prevPixelCount = 0;

    VelConstraint velConstraintOverride;
    AccelConstraint accelConstraintOverride = new ProfileAccelConstraint(-30.0, 30.0);

    double slowModeForHanging = 0.5;

    double slowModeForBackdrop = 0.5;
    boolean isHangingActivated = false;

    long startTime = 0L;

    Gamepad.LedEffect redEffect = new Gamepad.LedEffect.Builder()
            .addStep(1, 0, 0, 750) // Show red for 250ms
            .build();

    Gamepad.LedEffect greenEffect = new Gamepad.LedEffect.Builder()
            .addStep(0, 1, 0, 750) // Show green for 250ms
            .build();

    Gamepad.LedEffect whiteEffect = new Gamepad.LedEffect.Builder()
            .addStep(1, 1, 1, 750) // Show white for 250ms
            .build();

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addLine("Initializing...");
        telemetry.update();

        // Init
        g1 = new GamePadController(gamepad1);
        g2 = new GamePadController(gamepad2);
        g1.update();
        g2.update();
        sched = new ActionScheduler();
        drive = new MecanumDrive(hardwareMap, Memory.LAST_POSE);
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
        hang = new Hang(hardwareMap);
        drone = new Drone(hardwareMap);

        smartGameTimer = new SmartGameTimer(false);

        if (Globals.RUN_AUTO) {
            drive.pose = Globals.drivePose;
        }
        outtake.isAuto = false;
        Globals.RUN_AUTO = false;
        outtake.prepTeleop();

        intake.initialize(false);
        // Init opmodes
        outtake.initialize();
        drone.initialize();
        hang.initialize();

        // Ready!
        telemetry.addLine("Manual Drive is Ready!");
        telemetry.addLine("Drive Pose: " + new PoseMessage(drive.pose));
        telemetry.update();

        velConstraintOverride = new MinVelConstraint(Arrays.asList(
                this.drive.kinematics.new WheelVelConstraint(45.0),
                new AngularVelConstraint(Math.PI / 2)));

        waitForStart();

        startTime = System.currentTimeMillis();

        // Opmode start
        if (opModeIsActive()) {
            resetRuntime();
            g1.reset();
            g2.reset();

            // Finish pulling in slides
            if (!smartGameTimer.isNominal()) {
                outtake.finishPrepTeleop();
            }
        }

        // Main loop
        while (opModeIsActive()) {
            g1.update();
            g2.update();

            move();
            subsystemControls();
            pixelDetection();

            drive.updatePoseEstimate();
            sched.update();
            outtake.update();
            intake.update();

            backdropTouchedDetection();

            telemetry.addData("Time left: ", smartGameTimer.formattedString() + " (" + smartGameTimer.status() + ")");
            //telemetry.addLine(intake.getStackServoPositions());
            telemetry.addLine(outtake.getServoPositions());
            telemetry.addLine("Current Pixel count: " + Intake.pixelsCount + " | Total count: " + Intake.totalPixelCount + " | Prev count: " + prevPixelCount);
            telemetry.addLine("Intake state: " + intake.intakeState);
            telemetry.addLine(hang.getCurrentPosition());
            telemetry.update();
        }

        // On termination
        Memory.LAST_POSE = drive.pose;
    }

    private void backdropTouchedDetection() {
        if(outtake.hasOuttakeReached()) {
            if(!hasBackdropTouched) {
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
            if (prevPixelCount != Intake.pixelsCount && lastTimePixelDetected != null ) {
                long elapsedTime = System.currentTimeMillis() - lastTimePixelDetected.longValue();
                Log.d("TeleOps_Pixel_detection", prevPixelCount + "->" + Intake.pixelsCount + ", elapsed time (ms) " + elapsedTime);

                if(elapsedTime < 600) {
                    if (Intake.pixelsCount > 1) {
                        Intake.totalPixelCount--;
                        prevPixelCount = Intake.pixelsCount;

                        Log.d("TeleOps_Pixel_detection", "deduct 1 from the counts. Total: " + Intake.totalPixelCount + " current: " + Intake.pixelsCount);
                    }

                    sched.queueAction(intake.intakeReverse());

                    intakeSlowdownStartTime = new Long(System.currentTimeMillis());
                    lastTimePixelDetected = null;
                    Log.d("TeleOps_Pixel_detection", "slow down 2nd pixel at " + (intakeSlowdownStartTime - startTime));

                    g1.runLedEffect(whiteEffect);
                }
            }
            // log the count
            if(prevPixelCount != Intake.pixelsCount && Intake.pixelsCount >= 2 && intakeReverseStartTime == null ) {
                intakeReverseStartTime = new Long(System.currentTimeMillis());

                g1.runLedEffect(greenEffect);

                Log.d("TeleOps_Pixel_detection", "Pixel counted changed to "
                        +  Intake.pixelsCount + ", detected at " + (intakeReverseStartTime - startTime));
            }

            if (intakeSlowdownStartTime != null) {
                if ((System.currentTimeMillis() - intakeSlowdownStartTime.longValue()) > 500) {
                    sched.queueAction(intake.intakeOn());
                    intakeSlowdownStartTime = null;
                    intakeReverseStartTime = new Long(System.currentTimeMillis());
                }
            }

            // intake reverse is on
            if (intakeReverseStartTime != null && intakeSlowdownStartTime == null) {

                long elapsedTimeMs = System.currentTimeMillis() - intakeReverseStartTime.longValue();

                if (elapsedTimeMs > 500 && elapsedTimeMs < 600 &&
                        intake.intakeState.equals(Intake.IntakeState.ON)) {
                    sched.queueAction(intake.intakeReverse());
                    Log.d("TeleOps_Pixel_detection", "Pixel count changed to "
                            + Intake.pixelsCount + ", reversing started at " + (intakeReverseStartTime - startTime));
                }

                if (elapsedTimeMs > 1800 &&
                        intake.intakeState.equals(Intake.IntakeState.REVERSING))  {
                    sched.queueAction(intake.intakeOff());
                    intakeReverseStartTime = null;

                    Log.d("TeleOps_Pixel_detection", "Intake reverse is completed.");
                }
            }

            // detect the first pixel in time
            if (Intake.pixelsCount != prevPixelCount) {
                if (intake.stackIntakeState == Intake.StackIntakeState.UP ) {
                    lastTimePixelDetected = new Long(System.currentTimeMillis());
                }

                Log.d("TeleOps_Pixel_detection", "Pixel count changed from " + prevPixelCount + " -> " + Intake.pixelsCount +" | detected at " + (lastTimePixelDetected - startTime));

                g1.runLedEffect(redEffect);
            }

            prevPixelCount = Intake.pixelsCount;
        }
    }

    private void move() {
        double speed = (1 - Math.abs(g1.right_stick_x)) * (DRIVE_SPEED - SLOW_DRIVE_SPEED) + SLOW_DRIVE_SPEED;

        if(isHangingActivated) {
            speed = speed*slowModeForHanging;
        }
        else if(isSlideOut) {
            speed = speed*slowModeForBackdrop;
        }

        double input_x = Math.pow(-g1.left_stick_y, 3) * speed;
        double input_y = Math.pow(-g1.left_stick_x, 3) * speed;

        // if outtake has touched the backdrop, don't move further
        if(outtake.hasOuttakeReached()) {
            if(input_x < -0.05) {
                input_x = 0.0;
                Log.d("Drive_power", String.format("input_x: %3.2f to 0.0", input_x) + String.format(" | input_y: %3.2f", input_y));
            }
        }

        double input_turn = Math.pow(g1.left_trigger - g1.right_trigger, 3) * TURN_SPEED;
        if (g1.leftBumper()) input_turn += SLOW_TURN_SPEED;
        if (g1.rightBumper()) input_turn -= SLOW_TURN_SPEED;

        Vector2d input = new Vector2d(input_x, input_y);
        drive.setDrivePowers(new PoseVelocity2d(input, input_turn));

        telemetry.addData("drive_power", "input_x: %3.2f | input_y: %3.2f | speed: %3.2f", input_x, input_y, speed);
    }

    boolean isSlideOut = false;
    boolean isStackIntakeOn = false;

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

        if (g1.b()) {
            if (isSlideOut) {
                isSlideOut = false;
                sched.queueAction(new SequentialAction(outtake.latchScore0(), new SleepAction(0.2)));
                sched.queueAction(outtake.retractOuttake());
            }
            sched.queueAction(intake.intakeReverse());
        }
        if (!g1.b() && intake.intakeState == Intake.IntakeState.REVERSING && intakeReverseStartTime == null) {
            sched.queueAction(intake.intakeOff());
        }

        // Outtake controls
        if (g1.yLong()) {
            sched.queueAction(outtake.latchScore2());
        } else if (g1.yOnce()) {
            if (isSlideOut) {
                if (outtake.latchState == Outtake.OuttakeLatchState.LATCH_1) {
                    sched.queueAction(outtake.latchScore2());
                } else {
                    sched.queueAction(outtake.latchScore1());
                }
            } else {
                isSlideOut = true;
                sched.queueAction(intake.intakeOff());
                sched.queueAction(outtake.prepareToSlide());
                sched.queueAction(outtake.extendOuttakeTeleOps());
                sched.queueAction(new SleepAction(0.5));
                sched.queueAction(outtake.prepareToScore());
            }
        }

        if (Math.abs(g1.right_stick_y) > 0.25 && isSlideOut) {
            sched.queueAction(outtake.moveSliderBlocking(-g1.right_stick_y));
        }

        // Hang arms up/down
        if (g1.dpadUpOnce()) {

            sched.queueAction(hang.unblockHook());
            sched.queueAction(new SleepAction(0.35));
            sched.queueAction(outtake.outtakeWireForHanging());

            if (outtake.isHangingHookUp) {
                isHangingActivated = true;
            } else {
                isHangingActivated = false;
            }
        }

        if (g1.dpadDownLong()) {
            if(isHangingActivated) {
                sched.queueAction(hang.hang());
            }
        } else if (g1.dpadDownOnce()) {
            if(isHangingActivated) {
                sched.queueAction(hang.hangSlowly());
            }
        }

        // move left and right by one slot
        if (g1.dpadLeftOnce() || g1.dpadRightOnce()) {

            int multiplier = (Globals.RUN_AUTO) ? 1 : -1;

            double strafeDistance = (g1.dpadLeftOnce() ? STRAFE_DISTANCE : -STRAFE_DISTANCE) * multiplier;

            autoRunSched = new AutoActionScheduler(this::update);
            Log.d("ManualDrive", "Pose before strafe: " + new PoseMessage(this.drive.pose) + " | target=" + strafeDistance);
            double start_y = drive.pose.position.y;
            autoRunSched.addAction(
                    new SequentialAction(
                            outtake.afterScore(),
                            drive.actionBuilder(drive.pose)
                                    .strafeToConstantHeading(new Vector2d(drive.pose.position.x, drive.pose.position.y + strafeDistance),
                                            velConstraintOverride,
                                            accelConstraintOverride)
                                    .build(),
                            outtake.prepareToScore())

            );

            autoRunSched.run();

            Log.d("ManualDrive", "Pose after strafe: " + new PoseMessage(this.drive.pose) + " | actual=" + (drive.pose.position.y - start_y));
        }

        // drone launch
        if (g1.backOnce()) {
            sched.queueAction(intake.stackIntakeLinkageDown());
            sched.queueAction(new SleepAction(0.25));
            sched.queueAction(drone.scoreDrone());
            sched.queueAction(new SleepAction(0.25));
            sched.queueAction(intake.stackIntakeLinkageUp());
        }

        // stack intake
        if (g1.start() && g1.x()) {
            if (!isStackIntakeOn) {
                isStackIntakeOn = true;
                sched.queueAction(intake.intakeTwoStackedPixels());
            }
        } else if (g1.aOnce()) {
            isStackIntakeOn = false;
            if (intake.stackIntakeState == Intake.StackIntakeState.DOWN) {
                sched.queueAction(new SequentialAction(
                        intake.prepareTeleOpsIntake(),
                        intake.intakeOff()));
            } else {
                sched.queueAction(
                        new SequentialAction(
                                outtake.prepareToTransfer(),
                                intake.prepareStackIntake(),
                                intake.intakeOn()));
            }
        }

        if(g1.start() && g1.guide()) {
            outtake.resetSlideEncoder();
        }
        else if (g1.guideOnce()) {

            if(isHangingActivated) {
                Log.d("Hang_drop_down", "Hang current position: " + hang.getCurrentPosition());
                hang.dropdownFromHang();
                long startTime = System.currentTimeMillis();
                while (hang.isMotorBusy() && (System.currentTimeMillis() - startTime) < 2000) {
                    idle();
                }

                Log.d("Hang_drop_down", "Hang after position: " + hang.getCurrentPosition());
            } else {
                sched.queueAction(outtake.reverseDump());
            }
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
        this.drive.updatePoseEstimate();
        telemetry.update();
    }
}

