package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.hardware.HardwareCreator;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicReference;

/**
 * This is the Outtake mechanism. It includes 2 servos and 2 motors
 * V-slider motor: extend up and down
 * Turret motor: turn the v-slider towards the pole
 * Claw arm servo (need big torque): rotate to pickup and dump the cone
 * Claw servo (speed): grab and drop the cone
 *
 */
public class Outtake extends iSubsystem{

    public static double VSLIDE_POSITION_P = 0.025;
    public static double VSLIDE_POSITION_D = 0.0001;
    public static double VSLIDE_POSITION_I = 0.0;

    public static double TURRET_POSITION_P = 0.02;
    public static double TURRET_POSITION_D = 0.0005;
    public static double TURRET_POSITION_I = 0.0;

    public static double TURRET_POSITION_PID = 7.0;

    // for outtake claw position
    public static double OUTTAKE_CLAW_HALF_OPEN_POSITION = 0.65;
    public static double OUTTAKE_CLAW_OPEN_POSITION = 0.60;
    public static double OUTTAKE_CLAW_CLOSE_POSITION_CHECK = 0.72;
    public static double OUTTAKE_CLAW_CLOSE_POSITION = 0.78;

    // for outtake arm positions
    public static double OUTTAKE_ARM_INIT_POSITION = 0.825;
    public static double OUTTAKE_ARM_LANDING_POSITION = 0.83;
    public static double OUTTAKE_ARM_PRELOAD_PRE_HIGH_SCORE_POSITION = 0.34;
    public static double OUTTAKE_ARM_PRE_HIGH_SCORE_POSITION = 0.32;
    public static double OUTTAKE_ARM_PRE_HIGH_SCORE_POSITION_TELEOPS = 0.29;
    public static double OUTTAKE_ARM_HIGH_SCORE_POSITION = 0.19;
    public static double OUTTAKE_ARM_PRE_MID_SCORE_POSITION = 0.15;
    public static double OUTTAKE_ARM_PRE_MIDDLE_SCORE_POSITION_TELEOPS = 0.17;
    public static double OUTTAKE_ARM_MEDIUM_SCORE_POSITION = 0.12;

    // Slider and turret positions
    public static int TURRET_INIT_POSITION = 0;
    public static int TURRET_FAR_RIGHT_HIGH_POSITION = 280;
    public static int TURRET_FAR_LEFT_HIGH_POSITION = -260;
    public static int TURRET_NEAR_RIGHT_HIGH_POSITION = 320;
    public static int TURRET_NEAR_LEFT_HIGH_POSITION = -320;
    public static int TURRET_RED_NEAR_RIGHT_HIGH_POSITION = 300;
    public static int TURRET_RED_NEAR_LEFT_HIGH_POSITION = -320;
    public static int TURRET_RED_FAR_RIGHT_HIGH_POSITION = 260;
    public static int TURRET_RED_FAR_LEFT_HIGH_POSITION = -260;
    public static int TURRET_RIGHT_MIDDLE_POSITION = 370;
    public static int TURRET_LEFT_MIDDLE_POSITION = -370;
    public static int TURRET_RED_LEFT_MIDDLE_POSITION = -380;
    public static int TURRET_TRANSFER_POSITION = 20;

    public static int VSLIDER_FAR_HIGH_POSITION = -680;
    public static int VSLIDER_NEAR_LEFT_HIGH_POSITION = -640;
    public static int VSLIDER_NEAR_RIGHT_HIGH_POSITION = -670;
    public static int VSLIDER_RED_NEAR_LEFT_HIGH_POSITION = -640;
    public static int VSLIDER_RED_NEAR_RIGHT_HIGH_POSITION = -690;
    public static int VSLIDER_NEAR_HIGH_POSITION_TELEOPS = -640;
    public static int VSLIDER_RED_FAR_HIGH_POSITION = -720;
    public static int VSLIDER_INIT_POSITION = -5;
    public static int VSLIDER_TELEOPS_TRANSFER_POSITION = -5;
    public static int VSLIDER_RIGHT_MIDDLE_POSITION = -370;
    public static int VSLIDER_LEFT_MIDDLE_POSITION = -370;

    public static int VSLIDER_RED_LEFT_MIDDLE_POSITION = -340;
    public static int VSLIDER_SAFE_TRANSFER_POSITION = -30;

    private final static String turretMotorName = "turret";
    private final static String vSlideMotorName = "vSlide";

    private final static String outtakeClaw = "outtake";
    private final static String outtakeArm = "o-arm";

    public static double motorControllerDelayInMilli = 10;

    private DcMotorEx turretMotor;
    private DcMotorEx vSlideMotor;

    private Servo outtakeClawServo;
    private Servo outtakeArmServo;

    // 0 for Autonomous
    // 1 for teleops
    // this is for turret motor
    private int outtakeOpMode = 0;

    // 0 for RUN_TO_POSITION
    // 1 for Custom PID
    // this is for v-slider motor
    private  int motorControlMode = 1;

    PIDController vSlideController = new PIDController(VSLIDE_POSITION_P,
            VSLIDE_POSITION_I,
            VSLIDE_POSITION_D);

    PIDController turretController = new PIDController(TURRET_POSITION_P,
            TURRET_POSITION_I,
            TURRET_POSITION_D);

    private int vSliderTargetPosition = 0;
    private int turretTargetPosition = 0;
    private int lastScoredVSliderTargetPosition = 0;
    private int lastScoredTurretTargetPosition = 0;
    private Pole_Position lastTargetedPosition = null;

    private int lastScoredLeftHighVSliderTargetPosition = 0;
    private int lastScoredLeftHighTurretTargetPosition = 0;

    private int lastScoredRightHighVSliderTargetPosition = 0;
    private int lastScoredRightHighTurretTargetPosition = 0;

    private int lastScoredLeftMiddleVSliderTargetPosition = 0;
    private int lastScoredLeftMiddleTurretTargetPosition = 0;

    private int lastScoredRightMiddleVSliderTargetPosition = 0;
    private int lastScoredRightMiddleTurretTargetPosition = 0;

    private boolean resetTargetPositions = false;
    private boolean shutdown= false;
    private AtomicReference<Boolean> isVSliderTargetReached = new AtomicReference<>(Boolean.FALSE);

    private AtomicReference<Boolean> isTurretTargetReached = new AtomicReference<>(Boolean.FALSE);

    public enum Pole_Position  {
        FAR_RIGHT_HIGH,
        FAR_LEFT_HIGH,
        RED_FAR_RIGHT_HIGH,
        RED_FAR_LEFT_HIGH,
        NEAR_RIGHT_HIGH,
        NEAR_LEFT_HIGH,
        RED_NEAR_RIGHT_HIGH,
        RED_NEAR_LEFT_HIGH,
        RIGHT_MIDDLE,
        LEFT_MIDDLE,
        RED_LEFT_MIDDLE
    };

    public Outtake(HardwareMap hardwareMap) {
        initAttachmentMotors(hardwareMap);
        initAttachmentServos(hardwareMap);
    }

    private void initAttachmentMotors(HardwareMap hardwareMap) {
        turretMotor = initMotor(hardwareMap,turretMotorName);
        vSlideMotor = initMotor(hardwareMap,vSlideMotorName);
    }

    private void initAttachmentServos(HardwareMap hardwareMap) {
        outtakeClawServo = HardwareCreator.createServo(hardwareMap, outtakeClaw);
        outtakeArmServo = HardwareCreator.createServo(hardwareMap, outtakeArm);

        outtakeClawServo.setPosition(OUTTAKE_CLAW_CLOSE_POSITION);
        outtakeArmServo.setPosition(OUTTAKE_ARM_INIT_POSITION);
    }

    /**
     * To score, move the arm down and open up the claw
     * @param numOfCones
     */
    public void score(int numOfCones) {
        //lowers the arm to prepare to release the cone
        RobotLog.d("--- Scoring cone #: " + numOfCones);
        double armPosition = outtakeArmServo.getPosition();

        // only move the arm down if the current position is higher than the scoring position
        //----------------------------------------------------------
        if(vSliderTargetPosition > VSLIDER_NEAR_LEFT_HIGH_POSITION) {
            if(armPosition > OUTTAKE_ARM_MEDIUM_SCORE_POSITION) {
                outtakeArmServo.setPosition(OUTTAKE_ARM_MEDIUM_SCORE_POSITION);
            }
        }
        else {
            if(armPosition > OUTTAKE_ARM_HIGH_SCORE_POSITION) {
                outtakeArmServo.setPosition(OUTTAKE_ARM_HIGH_SCORE_POSITION);
            }
        }
        sleep(100);

        //release cone and score
        outtakeClawServo.setPosition(OUTTAKE_CLAW_OPEN_POSITION);
        sleep(200);

        if( (lastScoredVSliderTargetPosition < -200 || lastScoredVSliderTargetPosition == 0) && lastTargetedPosition != null ) {

            switch (lastTargetedPosition) {
                case LEFT_MIDDLE:
                    // left turn is negative
                    // this is to avoid driver button mistake
                    // TURRET_LEFT_MIDDLE_POSITION
                    if(turretTargetPosition < 0) {
                        lastScoredLeftMiddleTurretTargetPosition = turretTargetPosition;
                        lastScoredLeftMiddleVSliderTargetPosition = vSliderTargetPosition;
                    } else {
                        lastScoredRightMiddleTurretTargetPosition = turretTargetPosition;
                        lastScoredRightMiddleVSliderTargetPosition = vSliderTargetPosition;

                        lastTargetedPosition = Pole_Position.RIGHT_MIDDLE;
                    }
                    break;
                case RIGHT_MIDDLE:
                    // right turn is negative
                    // this is to avoid driver button mistake
                    if(turretTargetPosition > 0) {
                        lastScoredRightMiddleTurretTargetPosition = turretTargetPosition;
                        lastScoredRightMiddleVSliderTargetPosition = vSliderTargetPosition;
                    } else {
                        lastScoredLeftMiddleTurretTargetPosition = turretTargetPosition;
                        lastScoredLeftMiddleVSliderTargetPosition = vSliderTargetPosition;
                        lastTargetedPosition = Pole_Position.LEFT_MIDDLE;
                    }
                    break;
                case NEAR_LEFT_HIGH:
                    if(turretTargetPosition < 0) {
                        lastScoredLeftHighTurretTargetPosition = turretTargetPosition;
                        lastScoredLeftHighVSliderTargetPosition = vSliderTargetPosition;
                    } else {
                        lastScoredRightHighTurretTargetPosition = turretTargetPosition;
                        lastScoredRightHighVSliderTargetPosition = vSliderTargetPosition;
                        lastTargetedPosition = Pole_Position.NEAR_RIGHT_HIGH;
                    }
                    break;
                case NEAR_RIGHT_HIGH:
                    if(turretTargetPosition > 0) {
                        lastScoredRightHighTurretTargetPosition = turretTargetPosition;
                        lastScoredRightHighVSliderTargetPosition = vSliderTargetPosition;
                    } else {
                        lastScoredLeftHighTurretTargetPosition = turretTargetPosition;
                        lastScoredLeftHighVSliderTargetPosition = vSliderTargetPosition;
                        lastTargetedPosition = Pole_Position.NEAR_LEFT_HIGH;
                    }
                    break;

            }

            lastScoredTurretTargetPosition = turretTargetPosition;
            lastScoredVSliderTargetPosition = vSliderTargetPosition;
        }

        RobotLog.d("--- Scored cone #: " + numOfCones + " ---");
    }

    /**
     * To score, move the arm down and open up the claw
     * @param numOfCones
     */
    public void scoreImmediately(int numOfCones) {
        //lowers the arm to prepare to release the cone
        RobotLog.d("--- Scoring cone #: " + numOfCones);
        long timer0 = System.currentTimeMillis();
        outtakeArmServo.setPosition(OUTTAKE_ARM_HIGH_SCORE_POSITION);
        sleep(150);

        //release cone and score
        outtakeClawServo.setPosition(OUTTAKE_CLAW_OPEN_POSITION);

        if(lastScoredVSliderTargetPosition < -200 || lastScoredVSliderTargetPosition == 0) {
            lastScoredTurretTargetPosition = turretTargetPosition;
            lastScoredVSliderTargetPosition = vSliderTargetPosition;
        }

        sleep(40);
        RobotLog.d("--- Scored cone #: " + numOfCones + " --- " + (System.currentTimeMillis() - timer0) + " (ms)");
    }

    /**
     * 1. Move the v-slider to the zero position
     * 2. move the outtake arm to the right position
     * 3. outtake claw in the half open position
     */
    public void prepareTransferInAuto() {
        RobotLog.d("!!!! prepareTransfer() | v-slider target position: " + VSLIDER_INIT_POSITION);
        long timer0 = System.currentTimeMillis();
        outtakeClawServo.setPosition(OUTTAKE_CLAW_HALF_OPEN_POSITION);
        outtakeArmServo.setPosition(OUTTAKE_ARM_PRE_HIGH_SCORE_POSITION);
        sleep(50);

        turn(TURRET_TRANSFER_POSITION);
        sleep(200);
        if(vSliderTargetPosition != VSLIDER_INIT_POSITION) {
            vSliderTargetPosition = VSLIDER_INIT_POSITION;
            isVSliderTargetReached.set(Boolean.FALSE);
            sleep(20);
        }

        // outtakeArmServo.setPosition(OUTTAKE_ARM_INIT_POSITION);

        RobotLog.d("!!!! prepareTransfer() completed! " + (System.currentTimeMillis() - timer0) +  " (ms)");
    }

    public void moveArmToInitTransferPosition() {
        outtakeArmServo.setPosition(OUTTAKE_ARM_INIT_POSITION);
    }

    /**
     * 1. Move the v-slider to the zero position
     * 2. move the outtake arm to the right position
     * 3. outtake claw in the half open position
     */
    public void prepareTransferInTeleOps() {
        RobotLog.d("!!!! prepareTransfer() v-slider target position: " + VSLIDER_INIT_POSITION);
        outtakeArmServo.setPosition(OUTTAKE_ARM_PRE_HIGH_SCORE_POSITION);
        sleep(70);
        turn(TURRET_TRANSFER_POSITION);
        vSliderTargetPosition = VSLIDER_TELEOPS_TRANSFER_POSITION;
        isVSliderTargetReached.set(Boolean.FALSE);
        sleep(100);
        outtakeClawServo.setPosition(OUTTAKE_CLAW_HALF_OPEN_POSITION);
        sleep(180);
        outtakeArmServo.setPosition(OUTTAKE_ARM_INIT_POSITION);
    }

    /**
     * This is used in the teleops to manually adjust the v-slider length
     * @param increments
     */
    public void adjustVSlider(double increments) {
        int vSliderPosition = vSliderTargetPosition;
        if(increments > 0.5) {
            vSliderPosition -= 20;
        } else if(increments < -0.5) {
            vSliderPosition += 20;
        }

        if(isVSliderTargetReached.get()) {

            vSliderTargetPosition = Range.clip(vSliderPosition, -770, 0);
            isVSliderTargetReached.set(Boolean.FALSE);

            resetTargetPositions = true;
            RobotLog.d("Adjusted v-slider target: " + vSliderTargetPosition);
        }
    }

    /**
     * This is used in the teleops to manually adjust turret angle
     * @param increments
     */
    public void adjustTurret(double increments) {

        int targetPosition = turretTargetPosition; // turretMotor.getTargetPosition();
        if(increments > 0.2) {
            targetPosition += 30;
        } else if(increments < -0.2) {
            targetPosition -= 30;
        }

        int currentPosition = turretMotor.getCurrentPosition();

        if(Math.abs(currentPosition-turretTargetPosition) < 10) {
            turretTargetPosition = targetPosition;
            isTurretTargetReached.set(false);
            //turn(turretTargetPosition);

            resetTargetPositions = true;

            RobotLog.d("Adjusted turret target: " + turretTargetPosition);
        }
    }

    /**
     * Turn the turret to a predefined position, like LEFT_NEAR_HIGH, LEF_MIDDLE etc.
     * @param position
     */
    public void turn (int position) {
        turretTargetPosition = position;
        isTurretTargetReached.set(false);
    }

    public void moveToPosition (int position) {
        if(position != vSliderTargetPosition) {
            vSliderTargetPosition = position;
            isVSliderTargetReached.set(Boolean.FALSE);
        }
    }

    public void moveArmUp() {
        double position = outtakeArmServo.getPosition();
        position += 0.05;

        outtakeArmServo.setPosition(position);
    }

    public void moveArmDown() {
        double position = outtakeArmServo.getPosition();
        position -= 0.05;

        outtakeArmServo.setPosition(position);
    }

    /**
     * Based on the different positions, extend the v-slide to different length and turret turn to
     * different angle.
     *
     * @param position
     */
    public void raiseUpOuttake(Pole_Position position) {
        raiseUpOuttake(position, false);
    }

    /**
     * this is used in autonomous, to account for the potential heading errors.
     * In teleops, drivers can adjust first.
     *
     * @param position
     * @param offset
     */
    public void raiseUpOuttake(Pole_Position position, boolean offset) {

        RobotLog.d("** raiseUpOuttake() " + position);

        int turretPosition =0;
        int vsliderPosition =0;

        // make sure outtake claw is closed
        outtakeClawServo.setPosition(OUTTAKE_CLAW_CLOSE_POSITION);

        // move the turret to v-slide to the scoring position
        switch (position) {
            case FAR_RIGHT_HIGH:
                turretPosition = TURRET_FAR_RIGHT_HIGH_POSITION;
                vsliderPosition = VSLIDER_FAR_HIGH_POSITION;
                break;
            case FAR_LEFT_HIGH:
                turretPosition = TURRET_FAR_LEFT_HIGH_POSITION;
                vsliderPosition = VSLIDER_FAR_HIGH_POSITION;
                break;
            case NEAR_RIGHT_HIGH:
                if(resetTargetPositions && lastScoredRightHighVSliderTargetPosition != 0) {
                    turretPosition = lastScoredRightHighTurretTargetPosition;
                    vsliderPosition = lastScoredRightHighVSliderTargetPosition;
                } else {
                    turretPosition = TURRET_NEAR_RIGHT_HIGH_POSITION;
                    if(outtakeOpMode == 1) {
                        vsliderPosition = VSLIDER_NEAR_HIGH_POSITION_TELEOPS;
                    } else {
                        vsliderPosition = VSLIDER_NEAR_RIGHT_HIGH_POSITION;
                    }
                }
                break;
            case NEAR_LEFT_HIGH:
                if(resetTargetPositions && lastScoredLeftHighVSliderTargetPosition != 0) {
                    turretPosition = lastScoredLeftHighTurretTargetPosition;
                    vsliderPosition = lastScoredLeftHighVSliderTargetPosition;
                } else {
                    turretPosition = TURRET_NEAR_LEFT_HIGH_POSITION;
                    if(outtakeOpMode == 1) {
                        vsliderPosition = VSLIDER_NEAR_HIGH_POSITION_TELEOPS;
                    } else {
                        vsliderPosition = VSLIDER_NEAR_LEFT_HIGH_POSITION;
                    }
                }
                break;
            case RED_NEAR_RIGHT_HIGH:
                turretPosition = TURRET_RED_NEAR_RIGHT_HIGH_POSITION;
                vsliderPosition = VSLIDER_RED_NEAR_RIGHT_HIGH_POSITION;
                break;

            case RED_NEAR_LEFT_HIGH:
                turretPosition = TURRET_RED_NEAR_LEFT_HIGH_POSITION;
                vsliderPosition = VSLIDER_RED_NEAR_LEFT_HIGH_POSITION;
                break;
            case RED_FAR_RIGHT_HIGH:
                turretPosition = TURRET_RED_FAR_RIGHT_HIGH_POSITION;
                vsliderPosition = VSLIDER_RED_FAR_HIGH_POSITION;
                break;

            case RED_FAR_LEFT_HIGH:
                turretPosition = TURRET_RED_FAR_LEFT_HIGH_POSITION;
                vsliderPosition = VSLIDER_RED_FAR_HIGH_POSITION;
                break;

            case RIGHT_MIDDLE:
                if(resetTargetPositions && lastScoredRightMiddleVSliderTargetPosition != 0) {
                    turretPosition = lastScoredRightMiddleTurretTargetPosition;
                    vsliderPosition = lastScoredRightMiddleVSliderTargetPosition;
                } else {
                    turretPosition = TURRET_RIGHT_MIDDLE_POSITION;
                    vsliderPosition = VSLIDER_RIGHT_MIDDLE_POSITION;
                }
                break;
            case LEFT_MIDDLE:
                if(resetTargetPositions && lastScoredLeftMiddleVSliderTargetPosition != 0) {
                    turretPosition = lastScoredLeftMiddleTurretTargetPosition;
                    vsliderPosition = lastScoredLeftMiddleVSliderTargetPosition;
                } else {
                    turretPosition = TURRET_LEFT_MIDDLE_POSITION;
                    vsliderPosition = VSLIDER_LEFT_MIDDLE_POSITION;
                }
                break;
            case RED_LEFT_MIDDLE:
                if(resetTargetPositions && lastScoredLeftMiddleVSliderTargetPosition != 0) {
                    turretPosition = lastScoredLeftMiddleTurretTargetPosition;
                    vsliderPosition = lastScoredLeftMiddleVSliderTargetPosition;
                } else {
                    turretPosition = TURRET_RED_LEFT_MIDDLE_POSITION;
                    vsliderPosition = VSLIDER_RED_LEFT_MIDDLE_POSITION;
                }
                break;
        }

        lastTargetedPosition = position;

        RobotLog.d("** raiseUpOuttake():: V-slider target position:" + vsliderPosition);
        RobotLog.d("** raiseUpOuttake():: Turret target position:" + turretPosition);
//        RobotLog.d("** Estimated Heading::" + String.format("%.3f",getCurrentEstimatedHeading()));
//
        // adjust the robot position heading error

        if(offset) {
            //turretPosition += getCurrentEstimatedHeading()*5;
        }
//        RobotLog.d("** raiseUpOuttake()::turret adjusted target position:" + turretPosition);

        if(lastScoredVSliderTargetPosition == 0) {
            vSliderTargetPosition = vsliderPosition + 0;
        }
        else {
            vSliderTargetPosition = vsliderPosition;
        }

        isVSliderTargetReached.set(Boolean.FALSE);

        sleep(100);

        turretTargetPosition = turretPosition;
        isTurretTargetReached.set(Boolean.FALSE);

//        sleep(50);

        // arm raise up above the high pole and move down when the v slide reaches height above pole
        if(offset) {

            if(position == Pole_Position.RIGHT_MIDDLE || position == Pole_Position.LEFT_MIDDLE) {
                outtakeArmServo.setPosition(OUTTAKE_ARM_PRE_MID_SCORE_POSITION);
            } else if(lastScoredVSliderTargetPosition == 0) {
                outtakeArmServo.setPosition(OUTTAKE_ARM_PRELOAD_PRE_HIGH_SCORE_POSITION);
            } else {
                outtakeArmServo.setPosition(OUTTAKE_ARM_PRE_HIGH_SCORE_POSITION);

            }
        } else {
            if(position == Pole_Position.RIGHT_MIDDLE || position == Pole_Position.LEFT_MIDDLE) {
                outtakeArmServo.setPosition(OUTTAKE_ARM_PRE_MIDDLE_SCORE_POSITION_TELEOPS);
            }
            else {
                outtakeArmServo.setPosition(OUTTAKE_ARM_PRE_HIGH_SCORE_POSITION_TELEOPS);
            }
        }
    }

    public void resetTargetPositions() {
        resetTargetPositions = true;
    }

    /**
     * This is the main thread to control the v-slide motor. Set a target position, the PID controller
     * will move to the new position and maintain the position.
     *
     */
    private void start() {
        RobotLog.d("** Outtake v-slider controller thread started !!!!" + "Thread-Id: " + Thread.currentThread().getId() );
        ExecutorService executor = Executors.newSingleThreadExecutor();
        executor.submit(() -> {
            ElapsedTime timer1 = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
            ElapsedTime timer2 = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
            ElapsedTime loopTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

            int currentPosition = vSlideMotor.getCurrentPosition();
            while(!shutdown && !Thread.currentThread().isInterrupted()) {

                if(motorControlMode == 0) {
                    vSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    vSlideMotor.setPower(0.95);
                    vSlideMotor.setTargetPosition(vSliderTargetPosition);
                    continue;
                }

                vSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                double previousTarget = vSlideController.getSetpoint();
                vSlideController.setSetpoint(vSliderTargetPosition);
                vSlideController.setOutputRange(-0.925, 0.925);

                double correction = vSlideController.update(currentPosition);
                vSlideMotor.setPower(correction);

                int error = Math.abs(vSliderTargetPosition - currentPosition);

                if(Math.abs(previousTarget -vSliderTargetPosition) >10) {
                    RobotLog.d("** v-slider new target position: " + "prev: " + previousTarget + " | new: " + vSliderTargetPosition +  " Thread_id: " + Thread.currentThread().getId());
                    timer1.reset();
                    timer2.reset();
                }

                if(!isVSliderTargetReached.get() && timer2.milliseconds() > 50.0) {
                    RobotLog.d("** v-slider controller:" + vSliderTargetPosition + " | " + currentPosition + " | " + error );
                    RobotLog.d("   v-slider power:" + String.format("%.3f",correction));
                    timer2.reset();
                }

                if(error < 15 ) {
                    if (!isVSliderTargetReached.get() && timer1.milliseconds() < 1500.0) {
                        RobotLog.d("** v-slider controller target reached!!!");
                        RobotLog.d("** v-slider controller:" + vSliderTargetPosition + " | " + currentPosition + " | " + error );
                        RobotLog.d("   v-slider elapsed time:" + String.format("%.3f",timer1.milliseconds()));
                    }
                    isVSliderTargetReached.set(Boolean.TRUE);
                }

                if(loopTimer.milliseconds() > motorControllerDelayInMilli) {
                    currentPosition = vSlideMotor.getCurrentPosition();
//                    vSlideMotor.setPower(correction);
                    //RobotLog.d("Outtake thread. v-slide: " + currentPosition + " | " +String.format("%.3f", loopTimer.milliseconds()));
                    loopTimer.reset();
                }
            }
            RobotLog.d("!!! Outtake V-Slider is shutting down !!!");
        });

        RobotLog.d("** Outtake turret controller thread started !!!!" + "Thread-Id: " + Thread.currentThread().getId() );
        ExecutorService executor1 = Executors.newSingleThreadExecutor();
        executor1.submit(() -> {
            double previousTarget = 0.0;
            int currentPosition = turretMotor.getCurrentPosition();;
            double correction = -0.001;
            ElapsedTime timer3 = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
            ElapsedTime loopTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
            while(!shutdown && !Thread.currentThread().isInterrupted()) {
                if (outtakeOpMode == 1) {
                    previousTarget = turretMotor.getTargetPosition();
                    turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    turretMotor.setPower(0.6);
                    turretMotor.setTargetPosition(turretTargetPosition);
                    correction = -0.001;
                }
                else {
                    previousTarget = turretController.getSetpoint();
                    turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    turretController.setSetpoint(turretTargetPosition);
                    turretController.setOutputRange(-0.7, 0.7);

                    correction = turretController.update(currentPosition);
                    turretMotor.setPower(correction);
                }

                int error = Math.abs(turretTargetPosition - currentPosition);

                if (Math.abs(previousTarget - turretTargetPosition) > 10.0) {
                    RobotLog.d("**** turret new target position: " + "prev: " + previousTarget + " | new: " + turretTargetPosition  + " Thread_id: " + Thread.currentThread().getId());
                    timer3.reset();
                }

                if (error < 20) {
                    if (!isTurretTargetReached.get()) {
                        RobotLog.d("**** turret controller target reached!!!");
                        RobotLog.d("     turret controller:" + turretTargetPosition + " | " + currentPosition + " | " + error );
                        RobotLog.d("     turret elapsed time:" + String.format("%.3f", timer3.milliseconds()));
                    }
                    isTurretTargetReached.set(Boolean.TRUE);
                }

                if(loopTimer.milliseconds() > motorControllerDelayInMilli) {
                    currentPosition = turretMotor.getCurrentPosition();
//                    if(correction >= 0.0) {
//                        turretMotor.setPower(correction);
//                    }
                    //RobotLog.d("Outtake thread. Turret: " + currentPosition + " | " + String.format("%.3f", loopTimer.milliseconds()));
                    loopTimer.reset();
                }
            }
            RobotLog.d("!!! Outtake Turret is shutting down !!!");
        });
    }

    public void stop() {
        prepareTransferInAuto();
        moveArmToInitTransferPosition();
        turn(TURRET_INIT_POSITION);
        sleep(500);
        resetAll();
    }

    /**
     * reset motor encoders
     */
    public void resetAll() {

        shutdown = true;
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setPower(0.0);
        vSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vSlideMotor.setPower(0.0);
    }

    public void resetZeroPosition() {

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setPower(0.0);
        vSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vSlideMotor.setPower(0.0);
    }

    public double getClawPosition() {
        return outtakeClawServo.getPosition();
    }

    public boolean isClawClosed() {
        return (outtakeClawServo.getPosition() > OUTTAKE_CLAW_CLOSE_POSITION_CHECK);
    }

    public double getArmPosition() {
        return outtakeArmServo.getPosition();
    }

    /**
     * When auto init, set up the servos to the right position and start the h-slider controller
     */
    public void setupAttachmentsForAutoRun() {
        // outtake arm down and claw in half open
        // use gamepad1 to close the claw to get the preload
        outtakeArmServo.setPosition(OUTTAKE_ARM_INIT_POSITION);
        //outtakeClawServo.setPosition(OUTTAKE_CLAW_HALF_OPEN_POSITION);

        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        turretMotor.setPositionPIDFCoefficients(TURRET_POSITION_PID);

        turretTargetPosition = 0;

        vSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // start a new thread which handles v-slider movements
        vSliderTargetPosition = 0;
        start();
    }

    /**
     * Initialize the servos to the right positions
     */
    public void setupAttachmentsForTeleOps() {
        // outtake arm down and claw in half open
        // use gamepad1 to close the claw to get the preload
        outtakeArmServo.setPosition(OUTTAKE_ARM_INIT_POSITION);
        outtakeClawServo.setPosition(OUTTAKE_CLAW_HALF_OPEN_POSITION);
        //outtakeClawServo.setPosition(OUTTAKE_CLAW_HALF_OPEN_POSITION);

        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        turretMotor.setPositionPIDFCoefficients(TURRET_POSITION_PID);
        turretTargetPosition = 0;

        vSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // start a new thread which handles v-slider movements
        vSliderTargetPosition = 0;
        start();
    }

    public void closeOuttakeClaw() {
        outtakeClawServo.setPosition(OUTTAKE_CLAW_CLOSE_POSITION);
    }

    public void halfOpenOuttakeClaw() {
        outtakeClawServo.setPosition(OUTTAKE_CLAW_HALF_OPEN_POSITION);
    }

    public void openOuttakeClaw() {
        outtakeClawServo.setPosition(OUTTAKE_CLAW_OPEN_POSITION);
    }

    /**
     * Toggle the claw from CLOSE->HALF_OPEN->OPEN->CLOSE
     */
    public void toggleOuttakeClaw() {
        double position = outtakeClawServo.getPosition();
        if(position == OUTTAKE_CLAW_CLOSE_POSITION ) {
            outtakeClawServo.setPosition(OUTTAKE_CLAW_HALF_OPEN_POSITION);
        } else if(position ==OUTTAKE_CLAW_HALF_OPEN_POSITION ) {
            outtakeClawServo.setPosition(OUTTAKE_CLAW_OPEN_POSITION);
        } else if(position == OUTTAKE_CLAW_OPEN_POSITION ) {
            outtakeClawServo.setPosition(OUTTAKE_CLAW_CLOSE_POSITION);
        } else {
            outtakeClawServo.setPosition(OUTTAKE_CLAW_CLOSE_POSITION);
        }
    }

    public int getVSliderPosition() {
        return vSlideMotor.getCurrentPosition();
    }

    public boolean isVSliderTargetPositionReached() {
        return isVSliderTargetReached.get();
    }

    public boolean isTurretTargetPositionReached() {
        return isTurretTargetReached.get();
    }

    public boolean isOuttakeTargetPositionReached() {
        return isTurretTargetReached.get() && isVSliderTargetReached.get();
    }

    public Servo getOuttakeClawServo() {
        return outtakeClawServo;
    }

    public int getVSliderTargetPosition() {
        return vSliderTargetPosition;
    }

    public int getTurretTargetPosition() {
        return turretTargetPosition;
    }

    // default is auto mode
    // for auto, we use custom controller
    // for teleops, we use RUN_TO_POSITION
    public void setTurretInTeleOps() {
        outtakeOpMode = 1;
    }
    public void setTurretInAuto() {
        outtakeOpMode = 0;
    }

    public void sendTelemetry(Telemetry telemetry) {
        telemetry.addData("outta ke claw position", outtakeClawServo.getPosition());
        telemetry.addData("outtake arm position", outtakeArmServo.getPosition());
        telemetry.addData("v-slider position", vSliderTargetPosition);
        telemetry.addData("turret target position", turretTargetPosition);
        telemetry.addData("v-slider current position", vSlideMotor.getCurrentPosition());
        telemetry.addData("turret target position", turretMotor.getCurrentPosition());
    }

    public void setMotorControllerDelayInMilli(double delay) {
        motorControllerDelayInMilli = delay;
    }

    public void setMotorControlMode(int mode) {
        motorControlMode = mode;
    }
}
