package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.hardware.HardwareCreator;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicReference;

/**
 * This is the intake mechanism. It includes 3 servo and 1 motor
 * H-SLide motor: extend the intake claw back and forth
 * Lift servo (need torque): lift up the claw to the right height to grab the cone
 * Claw Arm servo (need torque): rotate the claw to transfer the cone from intake claw to outtake claw
 * Claw servo, grab and drop the cone
 */
public class Intake extends iSubsystem{
    // for intake claw positions
    public static double INTAKE_CLAW_CLOSE_POSITION = 0.66;
    public static double INTAKE_CLAW_INIT_POSITION = 0.55;
    public static double INTAKE_CLAW_OPEN_POSITION = 0.36;
    public static double INTAKE_CLAW_TRANSFER_OPEN_POSITION = 0.45;
    public static double INTAKE_CLAW_HALF_OPEN_POSITION = 0.54;

//    public static double INTAKE_CLAW_HALF_OPEN_VOLTAGE = 1.578;
//    public static double INTAKE_CLAW_OPEN_VOLTAGE = 1.75;

    public static double OUTTAKE_CLAW_CLOSE_VOLTAGE = 0.20;

    // for intake lift positions
    public static double INTAKE_LIFT_INIT_POSITION = 0.30;
    public static double INTAKE_LIFT_LEVEL_1_POSITION = 0.30;
    public static double INTAKE_LIFT_LEVEL_2_POSITION = 0.36;
    public static double INTAKE_LIFT_LEVEL_3_POSITION = 0.41;
    public static double INTAKE_LIFT_LEVEL_4_POSITION = 0.45;
    public static double INTAKE_LIFT_LEVEL_5_POSITION = 0.49;
    public static double INTAKE_LIFT_LEVEL_6_POSITION = 0.54;
    public static double INTAKE_LIFT_LEVEL_7_POSITION = 0.57;
    public static double INTAKE_LIFT_LEVEL_MAX_POSITION = 0.57;

    // for intake arm positions
    public static double INTAKE_ARM_INIT_POSITION = 0.34;
    public static double INTAKE_ARM_GRAB_POSITION = 0.92;
    public static double INTAKE_ARM_PUSH_SIGNAL_SLEEVE_POSITION = 0.94;
    public static double INTAKE_ARM_PRE_TRANSFER_POSITION = 0.20;
    public static double INTAKE_ARM_TRANSFER_POSITION = 0.11;
    public static double INTAKE_ARM_AFTER_TRANSFER_POSITION = 0.15;
    public static double INTAKE_ARM_LOW_SCORE_POSITION = 0.66;
    public static double INTAKE_ARM_LOW_DROP_POSITION = 0.79;
    public static double INTAKE_ARM_PRELOAD_POSITION = 0.8;
//    public static double INTAKE_ARM_TRANSFER_VOLTAGE = 2.263;
//    public static double INTAKE_ARM_GRAB_VOLTAGE = 1.045;
//    public static double INTAKE_ARM_AFTER_GRAB_VOLTAGE = 1.50;

    public static int HSLIDER_NEAR_HIGH_GRAB_POSITION = -150;
    public static int HSLIDER_NEAR_HIGH_PRE_GRAB_POSITION = -80;

    public static int HSLIDER_INIT_POSITION = 30;
    public static int HSLIDER_TRANSFER_POSITION = 0;

    public static int HSLIDER_FAR_HIGH_GRAB_POSITION = -660;
    public static int HSLIDER_FAR_HIGH_PRE_GRAB_POSITION = -600;

    public static int HSLIDER_MID_GRAB_POSITION = -270;
    public static int HSLIDER_MID_PRE_GRAB_POSITION = -190;

    public static double motorControllerDelayInMilli = 10.0;

    private final static String hSlideMotorName = "hSlide";

    private final static String intakeClaw = "intake";
    private final static String intakeLift = "lift";
    private final static String intakeArm = "i-arm";

    private DcMotorEx hSlideMotor;

    private Servo intakeClawServo;
    private Servo intakeLiftServo;
    private Servo intakeArmServo;

    private PIDCoefficients pid = new PIDCoefficients(0.01,0.0,0.001);
    private PIDController hSlideController = new PIDController(0.01,0.0,0.001);

    private boolean changePosition = false;
    private double previousPosition = 0.0;

    private int hSliderTargetPosition = 0;

    private int lastGrabbedHSliderPosition = 0;
    private boolean resetHSliderPosition = false;
    private int lastGrabbedLiftLevel = -1;

    // use 0 for RUN_TO_POSITION
    // use 1 for Custom PID control
    private int motorControlMode = 1;

    private boolean shutdown = false;
    private AtomicReference<Boolean> isTargetReached = new AtomicReference<>(Boolean.FALSE);

    private AnalogInput clawVoltage;
    private AnalogInput armVoltage;
    public Intake(HardwareMap hardwareMap) {
        initAttachmentMotors(hardwareMap);
        initAttachmentServos(hardwareMap);
        clawVoltage = hardwareMap.get(AnalogInput.class, "i-claw-fb");
        armVoltage = hardwareMap.get(AnalogInput.class, "i-arm-fb");
    }

    private void initAttachmentMotors(HardwareMap hardwareMap) {
        hSlideMotor = initMotor(hardwareMap,hSlideMotorName);
    }

    enum ARM_DIRECTION {
        DOWN,
        UP
    }

    /**
     * Initialize the servos. to the init position
     * @param hardwareMap
     */
    private void initAttachmentServos(HardwareMap hardwareMap) {
        intakeClawServo = HardwareCreator.createServo(hardwareMap, intakeClaw);
        intakeLiftServo = HardwareCreator.createServo(hardwareMap, intakeLift);
        intakeArmServo = HardwareCreator.createServo(hardwareMap, intakeArm);

        intakeClawServo.setPosition(INTAKE_CLAW_INIT_POSITION);
        intakeLiftServo.setPosition(INTAKE_LIFT_INIT_POSITION);
        intakeArmServo.setPosition(INTAKE_ARM_INIT_POSITION);
    }

    /**
     * reset servos to the initial position
     * do this at the end of the autonomous
     */
    public void resetAll() {
        intakeArmServo.setPosition(INTAKE_ARM_INIT_POSITION);
        sleep(100);
        intakeClawServo.setPosition(INTAKE_CLAW_INIT_POSITION);
        intakeLiftServo.setPosition(INTAKE_LIFT_INIT_POSITION);

        hSliderTargetPosition = HSLIDER_INIT_POSITION;
        isTargetReached.set(Boolean.FALSE);
        sleep(300);
        hSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * This is the main thread to control the h-slider position. Just set a target, the PID controller
     * will move to the new position and maintain it
     */
    private void start() {
        RobotLog.d("** Intake v-slider controller thread started !!!!" + "Thread-Id: " + Thread.currentThread().getId() );
        ExecutorService executor = Executors.newSingleThreadExecutor();
        executor.submit(() -> {

            ElapsedTime timer1 = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
            ElapsedTime loopTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
            ElapsedTime positionTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
            hSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            int currentPosition = hSlideMotor.getCurrentPosition();
            while(!shutdown && !Thread.currentThread().isInterrupted()) {

                if(hSliderTargetPosition > 0 && positionTimer.milliseconds() > 1000.0) {
                    hSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    sleep(100);
                    hSliderTargetPosition = 0;
                    continue;
                }

                // use RUN_TO_POSITION
                if(motorControlMode == 0) {
                    hSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hSlideMotor.setPower(0.9);
                    hSlideMotor.setTargetPosition(hSliderTargetPosition);
                    continue;
                }

                hSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                double previousTarget = hSlideController.getSetpoint();

                hSlideController.setSetpoint(hSliderTargetPosition);
                hSlideController.setOutputRange(-0.9, 0.9);

                double correction = hSlideController.update(currentPosition);
                hSlideMotor.setPower(correction);

                int error = Math.abs(hSliderTargetPosition - currentPosition);

                if(Math.abs(previousTarget -hSliderTargetPosition) >1) {
                    RobotLog.d("** h-slider new target position: " + "prev: " + previousTarget + " | new: " + hSliderTargetPosition);
                    timer1.reset();
                    if(hSliderTargetPosition > 0 ) {
                        positionTimer.reset();
                    }
                }

                if(error < 10 || timer1.milliseconds() > 800.0) {
                    if (!isTargetReached.get()) {
                        RobotLog.d("** h-slider current position:" +currentPosition);
                        RobotLog.d("** h-slider position error:" + error);
                        RobotLog.d("** h-slider target reached:" + isTargetReached.get());
                        RobotLog.d("** h-slider elapsed time:" + String.format("%.3f",timer1.milliseconds()));
                    }
                    isTargetReached.set(Boolean.TRUE);
                }

                if(loopTimer.milliseconds() > motorControllerDelayInMilli) {
                    currentPosition = hSlideMotor.getCurrentPosition();
//                    hSlideMotor.setPower(correction);
                   // RobotLog.d("Intake -> start(). h-slide: " + currentPosition + " | " + String.format("%.3f", loopTimer.milliseconds()));
                    loopTimer.reset();
//                    RobotLog.d("Robot Voltage: " + String.format("%.3f", PowerPlayMecanumDrive.batteryVoltageSensor.getVoltage()));
                }
            }

            RobotLog.d(" !!! Intake is shutting down !!!");
        });
    }

    /**
     * Get the intake mechanism ready to grab the right level of the cone.
     * @param level
     */
    public void prepareToGrab(int level) {
        RobotLog.d("   intake prepareToGrab() starting:" + level);

        int vLevel = Range.clip(level,1,5);
        long timer0 = System.currentTimeMillis();
        if (vLevel==1) {
            intakeLiftServo.setPosition(INTAKE_LIFT_LEVEL_1_POSITION);
        } else if (vLevel==2) {
            intakeLiftServo.setPosition(INTAKE_LIFT_LEVEL_2_POSITION);
        } else if (vLevel==3) {
            intakeLiftServo.setPosition(INTAKE_LIFT_LEVEL_3_POSITION);
        } else if (vLevel==4) {
            intakeLiftServo.setPosition(INTAKE_LIFT_LEVEL_4_POSITION);
        } else if (vLevel==5) {
            intakeLiftServo.setPosition(INTAKE_LIFT_LEVEL_5_POSITION);
        } else {
            intakeLiftServo.setPosition(INTAKE_LIFT_LEVEL_1_POSITION);
        }
        // if the arm position is past the init position and claw is open
        // we need to close the claw first, so it doesn't hit the mini slide
        if(intakeArmServo.getPosition() < INTAKE_ARM_INIT_POSITION) {
            intakeClawServo.setPosition(INTAKE_CLAW_CLOSE_POSITION);
        }
        sleep(50);

        if(intakeArmServo.getPosition() < INTAKE_ARM_GRAB_POSITION ) {
            intakeArmServo.setPosition(INTAKE_ARM_GRAB_POSITION);
//        double armV = armVoltage.getVoltage();
            ElapsedTime timer1 = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
            while (
                //armV > INTAKE_ARM_GRAB_VOLTAGE &&
                    timer1.milliseconds() < 150) {
                sleep(10);
//            armV = armVoltage.getVoltage();

                //RobotLog.d("   Intake Arm voltage! " + String.format("%.3f", armV));
            }
        }

        if(intakeClawServo.getPosition() > INTAKE_CLAW_OPEN_POSITION) {
            intakeClawServo.setPosition(INTAKE_CLAW_OPEN_POSITION);
//        double clawV = clawVoltage.getVoltage();
            ElapsedTime timer2 = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
            while ( //clawV < INTAKE_CLAW_OPEN_VOLTAGE &&
                    timer2.milliseconds() < 100) {
                sleep(10);
                //clawV = clawVoltage.getVoltage();
            }
        }

        RobotLog.d("   intake prepareToGrab() finished! " + (System.currentTimeMillis() - timer0) + " (ms)");
    }

    /**
     * Get the intake mechanism ready to grab the 5th cone.
     */
    public void prepareToGrabStep0() {
        RobotLog.d("   intake prepareToGrabStep0");

        intakeLiftServo.setPosition(INTAKE_LIFT_LEVEL_5_POSITION);
        intakeArmServo.setPosition(INTAKE_ARM_PRELOAD_POSITION);
        intakeClawServo.setPosition(INTAKE_CLAW_OPEN_POSITION);
    }

    /**
     * Use the claw to grab the cone, then move the arm to transfer position
     * @param level
     */
    public void grab(int level) {
        RobotLog.d(" --- Grab cone #: " + level);
        long timer0 = System.currentTimeMillis();
        intakeClawServo.setPosition(INTAKE_CLAW_CLOSE_POSITION);
        sleep(100);
        if(level > 1) {
            moveIntakeLift(level + 2);
            sleep(100);
        }

        intakeArmServo.setPosition(INTAKE_ARM_TRANSFER_POSITION);

//        double armV = armVoltage.getVoltage();
        ElapsedTime timer1 = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        while(//armV < INTAKE_ARM_AFTER_GRAB_VOLTAGE &&
                timer1.milliseconds() < 100) {
            sleep(10);
            //armV = armVoltage.getVoltage();
        }

        intakeLiftServo.setPosition(INTAKE_LIFT_LEVEL_1_POSITION);

        lastGrabbedHSliderPosition = hSliderTargetPosition;
        lastGrabbedLiftLevel = level;

        RobotLog.d(" --- Grab finished! " + (System.currentTimeMillis()-timer0) + " (ms)");
    }

    /**
     * Use the claw to grab the cone, then move the arm to transfer position
     * @param level
     */
    public void grab(int level, Outtake outtake) {
        RobotLog.d(" --- Grab cone #: " + level);
        long timer0 = System.currentTimeMillis();
        intakeClawServo.setPosition(INTAKE_CLAW_CLOSE_POSITION);
        sleep(150);

        // save 100ms retracting outtake to transfer position
        outtake.prepareTransferInAuto();

        if(level > 1) {
            moveIntakeLift(level + 2);
            sleep(150);
        }

        // delay moving outtake arm to prevent the claw got caught
        //---------------------------------------------------------
        outtake.moveArmToInitTransferPosition();

        intakeArmServo.setPosition(INTAKE_ARM_TRANSFER_POSITION);
        sleep(150);
        intakeLiftServo.setPosition(INTAKE_LIFT_LEVEL_1_POSITION);

        lastGrabbedHSliderPosition = hSliderTargetPosition;
        lastGrabbedLiftLevel = level;

        RobotLog.d(" --- Grab finished! " + (System.currentTimeMillis()-timer0) + " (ms)");
    }

    /**
     * Private method to lift the claw to the right height
     * @param level
     */
    private final void moveIntakeLift(int level) {
        if(level >= 8) {
            intakeLiftServo.setPosition(INTAKE_LIFT_LEVEL_MAX_POSITION);
        }
        else if(level == 7) {
            intakeLiftServo.setPosition(INTAKE_LIFT_LEVEL_7_POSITION);
        } else if(level == 6) {
            intakeLiftServo.setPosition(INTAKE_LIFT_LEVEL_6_POSITION);
        } else if(level == 5) {
            intakeLiftServo.setPosition(INTAKE_LIFT_LEVEL_5_POSITION);
        } else if(level == 4) {
            intakeLiftServo.setPosition(INTAKE_LIFT_LEVEL_4_POSITION);
        } else if(level == 3) {
            intakeLiftServo.setPosition(INTAKE_LIFT_LEVEL_3_POSITION);
        } else if(level == 2) {
            intakeLiftServo.setPosition(INTAKE_LIFT_LEVEL_2_POSITION);
        } else {
            intakeLiftServo.setPosition(INTAKE_LIFT_LEVEL_1_POSITION);
        }
    }

    /**
     * Transfer a cone from intake claw to outtake claw
     */
    public void transfer(Servo outtakeClawServo) {
        double oClawPosition = outtakeClawServo.getPosition();
        RobotLog.d("!!! transfer() in progress ... | " + String.format("%.3f", oClawPosition));
        long timer0 = System.currentTimeMillis();
        if(oClawPosition > Outtake.OUTTAKE_CLAW_HALF_OPEN_POSITION) {
            outtakeClawServo.setPosition(Outtake.OUTTAKE_CLAW_HALF_OPEN_POSITION);
            sleep(100);
        }
        intakeArmServo.setPosition(INTAKE_ARM_TRANSFER_POSITION);
        sleep(100);
        intakeClawServo.setPosition(INTAKE_CLAW_HALF_OPEN_POSITION);

        sleep(200);
        intakeClawServo.setPosition(INTAKE_CLAW_TRANSFER_OPEN_POSITION);
        sleep(150);
        // close outtake claw and open up the intake claw
        outtakeClawServo.setPosition(Outtake.OUTTAKE_CLAW_CLOSE_POSITION);
        sleep(100);
        intakeArmServo.setPosition(INTAKE_ARM_AFTER_TRANSFER_POSITION);

        RobotLog.d("!!! transfer() done !!! " + (System.currentTimeMillis() - timer0) + " (ms)");
    }

    public void moveToPosition(int position) {
        hSliderTargetPosition = position;
        isTargetReached.set(Boolean.FALSE);
        RobotLog.d("   moveToPosition() h-slider target position:" + hSliderTargetPosition);
        sleep(20);
    }

    public void moveToTransferPositionInAuto() {
        RobotLog.d("!!! h-slide moveToTransfer(): " + HSLIDER_TRANSFER_POSITION);
        if(hSliderTargetPosition != HSLIDER_TRANSFER_POSITION) {
            hSliderTargetPosition = HSLIDER_TRANSFER_POSITION;
            isTargetReached.set(Boolean.FALSE);
            sleep(20);
        }

        intakeArmServo.setPosition(INTAKE_ARM_PRE_TRANSFER_POSITION);
    }

    public void moveToTransferPositionInTeleOps() {
//        RobotLog.d("!!! h-slide moveToTransfer(): " + HSLIDER_TRANSFER_POSITION);
//        hSliderTargetPosition = 0;
//        isTargetReached.set(Boolean.FALSE);
//        sleep(20);
//
//        intakeArmServo.setPosition(INTAKE_ARM_TRANSFER_POSITION);

        moveToTransferPositionInAuto();

    }

    public void prepareToScoreOnLowPole() {
        moveToPosition(HSLIDER_INIT_POSITION);
        moveIntakeLift(8);
        double position = intakeArmServo.getPosition();
        if ( position > INTAKE_ARM_INIT_POSITION) {
            intakeArmServo.setPosition(INTAKE_ARM_INIT_POSITION);
        } else {
            intakeArmServo.setPosition(INTAKE_ARM_LOW_SCORE_POSITION);
        }
    }

    public void scoreOnLowPole() {
        intakeArmServo.setPosition(INTAKE_ARM_LOW_DROP_POSITION);
    }

    public void resetZeroPosition() {
        //moveToTransferPositionInAuto();
        hSliderTargetPosition = HSLIDER_INIT_POSITION;
        isTargetReached.set(Boolean.FALSE);
        sleep(300);

        hSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void closeIntakeClaw() {
        intakeClawServo.setPosition(INTAKE_CLAW_CLOSE_POSITION);
    }

    public void halfOpenIntakeClaw() {
        intakeClawServo.setPosition(INTAKE_CLAW_HALF_OPEN_POSITION);
    }

    public void openIntakeClaw() {
        intakeClawServo.setPosition(INTAKE_CLAW_OPEN_POSITION);
    }

    public void moveIntakeArmDown() {
        intakeArmServo.setPosition(INTAKE_ARM_PUSH_SIGNAL_SLEEVE_POSITION);
    }
    public void moveIntakeArmToAfterTransfer() {
        intakeArmServo.setPosition(INTAKE_ARM_AFTER_TRANSFER_POSITION);
    }

    public void moveIntakeArmUp() {
        intakeArmServo.setPosition(INTAKE_ARM_INIT_POSITION);
    }

    public void moveIntakeArm(int direction) {
        if(direction ==0) {
            intakeArmServo.setPosition(Range.clip(intakeArmServo.getPosition() + 0.03, 0.4, 0.92));
        } else {
            intakeArmServo.setPosition(Range.clip(intakeArmServo.getPosition() - 0.03, 0.4, 0.92));
        }
    }

    public void toggleIntakeClaw() {
        double position = intakeClawServo.getPosition();
//        if(position == INTAKE_CLAW_CLOSE_POSITION ) {
//            intakeClawServo.setPosition(INTAKE_CLAW_HALF_OPEN_POSITION);
//        } else if(position == INTAKE_CLAW_HALF_OPEN_POSITION ) {
//            intakeClawServo.setPosition(INTAKE_CLAW_OPEN_POSITION);
//        } else if(position == INTAKE_CLAW_OPEN_POSITION ) {
//            intakeClawServo.setPosition(INTAKE_CLAW_CLOSE_POSITION);
//        } else {
//            intakeClawServo.setPosition(INTAKE_CLAW_CLOSE_POSITION);
//        }

        if(position > INTAKE_CLAW_INIT_POSITION ) {
            intakeClawServo.setPosition(INTAKE_CLAW_OPEN_POSITION);
        } else if(position < INTAKE_CLAW_INIT_POSITION ) {
            intakeClawServo.setPosition(INTAKE_CLAW_CLOSE_POSITION);
        }  else {
            intakeClawServo.setPosition(INTAKE_CLAW_HALF_OPEN_POSITION);
        }
    }

    public int toggleLiftLevel() {
        if(lastGrabbedLiftLevel <= 1) {
            lastGrabbedLiftLevel = 5;
        }
        else {
            --lastGrabbedLiftLevel;
        }

        lastGrabbedLiftLevel = Range.clip(lastGrabbedLiftLevel,1,5);

        RobotLog.dd("lastGrabbedLiftPosition: ", lastGrabbedLiftLevel +"");
        prepareToGrab(lastGrabbedLiftLevel);

        return lastGrabbedLiftLevel;
    }

    public int getLiftLevel() {
        return lastGrabbedLiftLevel;
    }

    private ElapsedTime manualAdjustTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public void adjustHSlider(double increments) {
        int position = hSliderTargetPosition;
        boolean fastMode = false;
        double adjustmentDelay = 100.0;
        if (increments < 0.55 && increments > 0.1) {
            position -= 30;
            adjustmentDelay = 30.0;
        }
        else if (increments >= 0.55 && increments < 0.90) {
            position -= 60;
            adjustmentDelay = 60.0;
        }
        else if (increments >= 0.90) {
            position -=150;
            fastMode = true;
        }
        else if(increments > -0.55 && increments < -0.1) {
            position +=30;
            adjustmentDelay = 30.0;
        }
        else if (increments <= -0.55 && increments > -0.90) {
            position +=60;
            adjustmentDelay = 60.0;
        }
        else if (increments <= -0.90) {
            position +=150;
            fastMode = true;
        }

        if( fastMode || manualAdjustTimer.milliseconds() > adjustmentDelay) {
//            motorControlMode = fastMode?1:0;

            motorControlMode = 1;

            hSliderTargetPosition = Range.clip(position, -640, 200);
            isTargetReached.set(Boolean.FALSE);
            resetHSliderPosition = true;
            manualAdjustTimer.reset();
        }
    }

    public void stop() {
        resetAll();
        sleep(200);
        shutdown = true;
        sleep(20);
        hSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hSlideMotor.setPower(0.0);
    }

    public double getClawPosition() {
        return intakeClawServo.getPosition();
    }

    public double getLiftPosition() {
        return intakeLiftServo.getPosition();
    }

    public double getArmPosition() {
        return intakeArmServo.getPosition();
    }

    public void setupAttachmentsForAutoRun() {
        intakeArmServo.setPosition(INTAKE_ARM_INIT_POSITION);
        intakeClawServo.setPosition(INTAKE_CLAW_HALF_OPEN_POSITION);
        hSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // start a new thread to handle-slider movements
        start();
    }

    public void setupAttachmentsForTeleOps() {
        intakeArmServo.setPosition(INTAKE_ARM_INIT_POSITION);
        intakeClawServo.setPosition(INTAKE_CLAW_HALF_OPEN_POSITION);
        hSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // start a new thread to handle-slider movements
        hSliderTargetPosition=0;
        //hSliderTargetPosition=HSLIDER_TRANSFER_POSITION;
        start();
    }

    public int getHSliderPosition() {
        return hSlideMotor.getCurrentPosition();
    }

    public boolean isTargetPositionReached() {
        return isTargetReached.get();
    }

//    public boolean isArmInTransferPosition() {
//        return armVoltage.getVoltage() > INTAKE_ARM_TRANSFER_VOLTAGE;
//    }

    public void sendTelemetry(Telemetry telemetry) {
        telemetry.addData("intake claw position", intakeClawServo.getPosition());
        telemetry.addData("intake arm position", intakeArmServo.getPosition());
        telemetry.addData("intake lift position", intakeClawServo.getPosition());
        telemetry.addData("h-slider target position", hSliderTargetPosition);
        telemetry.addData("h-slider position", hSlideMotor.getCurrentPosition());
    }

    public void setMotorControllerDelayInMilli(double delay) {
        motorControllerDelayInMilli = delay;
    }

    public void setMotorControlMode(int mode) {
        motorControlMode = mode;
    }

    public void moveToLastGrabbedPosition(int level) {
        int hsliderPosition = HSLIDER_NEAR_HIGH_GRAB_POSITION;
        if(lastGrabbedHSliderPosition != 0 ) {
            hsliderPosition = lastGrabbedHSliderPosition;
        }
        if(level > 1) {
            hsliderPosition = hsliderPosition + 70;
        }

        moveToPosition(Range.clip(hsliderPosition, HSLIDER_FAR_HIGH_GRAB_POSITION, 0));
//        int level = lastGrabbedLiftLevel -1;
//        level = (level<0)?1:level;
//        prepareToGrab(level);
    }

    public void moveToGrabPosition() {
        if(lastGrabbedHSliderPosition == 0) {
            lastGrabbedHSliderPosition = HSLIDER_NEAR_HIGH_GRAB_POSITION;
        }

        moveToPosition(Range.clip(lastGrabbedHSliderPosition, HSLIDER_FAR_HIGH_GRAB_POSITION, 0));

    }
}
