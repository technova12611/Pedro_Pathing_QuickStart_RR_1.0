package org.firstinspires.ftc.teamcode.subsystem;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.software.ActionUtil;
import org.firstinspires.ftc.teamcode.utils.hardware.HardwareCreator;
import org.firstinspires.ftc.teamcode.utils.hardware.MotorWithPID;
import org.firstinspires.ftc.teamcode.utils.control.PIDCoefficients;
import org.firstinspires.ftc.teamcode.utils.software.MovingArrayList;

import java.util.HashMap;
import java.util.Map;

@Config
public class Outtake {
    public static PIDCoefficients outtakePID = new PIDCoefficients(0.0065, 0, 0.0001);

    public static int OUTTAKE_SLIDE_MAX = 2160;
    public static int OUTTAKE_SLIDE_ABOVE_LEVEL_2 = 2150;
    public static int OUTTAKE_SLIDE_BELOW_LEVEL_2 = 1850;
    public static int OUTTAKE_SLIDE_ABOVE_LEVEL_1 = 1550;
    public static int OUTTAKE_SLIDE_BELOW_LEVEL_1 = 1100;

    public static int OUTTAKE_SLIDE_HIGH = OUTTAKE_SLIDE_ABOVE_LEVEL_2;
    public static int OUTTAKE_TELEOPS = OUTTAKE_SLIDE_BELOW_LEVEL_1;
    public static int OUTTAKE_SLIDE_MID = 1250;
    public static int OUTTAKE_SLIDE_CYCLES_ONE = 1050;
    public static int OUTTAKE_SLIDE_CYCLES_TWO = 1210;

    public static int OUTTAKE_SLIDE_FAR_LOW = 1070;
    public static int OUTTAKE_SLIDE_LOW = 850;

    public static int OUTTAKE_SLIDE_AFTER_DUMP_AUTO = 1100;
    public static int OUTTAKE_SLIDE_INIT = 0;

    public static int OUTTAKE_SLIDE_INCREMENT= 200;

    public static double LATCH_CLOSED = 0.55;
    public static double LATCH_SCORE_1 = 0.415;
    public static double LATCH_SCORE_2 = 0.47;

    public static double OUTTAKE_PIVOT_REVERSE_DUMP = 0.01;
    public static double OUTTAKE_PIVOT_INIT = 0.18;
    public static double OUTTAKE_PIVOT_SLIDING = 0.22;
    public static double OUTTAKE_PIVOT_DUMP_LOW = 0.38;
    public static double OUTTAKE_PIVOT_DUMP_MID = 0.39;

    public static double OUTTAKE_PIVOT_DUMP_HIGH = 0.56;

    public static double OUTTAKE_PIVOT_DUMP_CYCLE = 0.405;

    public static double OUTTAKE_PIVOT_DUMP_VERY_HIGH = 0.62;

    public static double SLIDE_PIVOT_INIT = 0.43;
    public static double SLIDE_PIVOT_SLIDING = 0.52;
    public static double SLIDE_PIVOT_DUMP = 0.238;

    public static double SLIDE_PIVOT_DUMP_1 = 0.248;
    public static double SLIDE_PIVOT_DUMP_VOLTAGE_MAX = 2.65;
    public static double SLIDE_PIVOT_DUMP_VOLTAGE_MIN = 2.58;

    public static double SLIDE_PIVOT_DUMP_VOLTAGE_SUPER_MAX = 2.70;
    public static double SLIDE_PIVOT_DUMP_VOLTAGE_SUPER_MIN = 2.63;

    public static double SLIDE_PIVOT_DUMP_2 = 0.268;

    public static double SLIDE_PIVOT_STRAFE = 0.285;

    public static double SLIDE_PIVOT_DUMP_HIGH = 0.10;

    public static double SLIDE_PIVOT_DUMP_VERY_HIGH = 0.0;

    public static double OUTTAKE_WIRE_DOWN = 0.72;

    public static double OUTTAKE_WIRE_SAFE_DOWN = 0.70;

    public static double OUTTAKE_WIRE_MIDDLE = 0.47;
    public static double OUTTAKE_WIRE_HIGH = 0.40;
    public static double OUTTAKE_WIRE_VERY_HIGH = 0.35;

    public static double OUTTAKE_WIRE_FOR_HANGING_DOWN = 0.73;
    public static double OUTTAKE_WIRE_FOR_HANGING_UP = 0.38;

    public static double OUTTAKE_FIXER_INIT = 0.77;
    public static double OUTTAKE_FIXER_LEVEL_1 = 0.09;
    public static double OUTTAKE_FIXER_LEVEL_2 = 0.18;
    public static double OUTTAKE_FIXER_LEVEL_3 = 0.26;
    public static double OUTTAKE_FIXER_LEVEL_4 = 0.31;

    public static double OUTTAKE_FIXER_LEVEL_1_5 = 0.14;
    public static double OUTTAKE_FIXER_LEVEL_2_5 = 0.22;
    public static double OUTTAKE_FIXER_LEVEL_3_5 = 0.28;

    final MotorWithPID slide;
    public boolean slidePIDEnabled = true;
    final Servo latch;
    final Servo slidePivot;
    final Servo outtakePivot;

    final Servo outtakeFixerServo;
    final Servo outtakeWireServo;

    final AnalogInput slidePivotVoltage;
    final AnalogInput outtakePivotVoltage;

    public boolean isAuto = true;

    private boolean scoreLevel3 = false;

    public boolean isHangingHookUp = false;

    public boolean backdropTouched = false;

    public FixerServoPosition fixerServoPosition = FixerServoPosition.LEVEL_0;
    private MovingArrayList slidePivotVoltages = new MovingArrayList(10);

    public Outtake(HardwareMap hardwareMap) {
        if (Memory.outtakeSlide != null) { // Preserve motor zero position
            this.slide = Memory.outtakeSlide;
            isAuto = false;
        } else {
            this.slide = new MotorWithPID(HardwareCreator.createMotor(hardwareMap, "outtake"), outtakePID);
            Memory.outtakeSlide = this.slide;
        }
        this.slide.setMaxPower(1.0);
        this.slide.getMotor().setDirection(DcMotorSimple.Direction.REVERSE);
        this.latch = HardwareCreator.createServo(hardwareMap, "outtakeLatch");
        this.slidePivot = HardwareCreator.createServo(hardwareMap, "outtakeSlidePivot");
        this.outtakePivot = HardwareCreator.createServo(hardwareMap, "outtakePivot");
        this.outtakeWireServo = HardwareCreator.createServo(hardwareMap, "outtakeWireServo");
        this.outtakeFixerServo = HardwareCreator.createServo(hardwareMap, "outtakeFixerServo");

        this.slidePivotVoltage = hardwareMap.get(AnalogInput.class, "slidePivotVoltage");
        this.outtakePivotVoltage = hardwareMap.get(AnalogInput.class, "outtakePivotVoltage");
    }

    public enum OuttakeLatchState {
        CLOSED,
        LATCH_1,
        LATCH_2
    }

    public OuttakeLatchState latchState;

    public class OuttakeLatchStateAction implements Action {
        OuttakeLatchState newState;

        public OuttakeLatchStateAction(OuttakeLatchState newState) {
            this.newState = newState;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            packet.addLine("Outtake latch state changed from " + latchState + " to " + newState);
            latchState = newState;
            return false;
        }
    }

    public void prepTeleop() {
        this.slide.getMotor().setPower(-0.2);
    }

    public void finishPrepTeleop() {
        this.slide.getMotor().setPower(0);
    }

    public void initialize() {
        this.slide.setCurrentPosition(0);
        this.slide.setTargetPosition(0);
        this.latch.setPosition(LATCH_CLOSED);
        this.outtakeWireServo.setPosition(OUTTAKE_WIRE_SAFE_DOWN);
        this.slidePivot.setPosition(SLIDE_PIVOT_INIT);
        this.outtakePivot.setPosition(OUTTAKE_PIVOT_INIT);
        this.outtakeFixerServo.setPosition(OUTTAKE_FIXER_INIT);
    }

    public void update() {
        slide.update();
        checkSlidePivotPosition();

//        if (!this.slide.isBusy()) {
//            if (this.slide.getCurrentPosition() < -20) {
//                Log.d("Outtake_Logger", "Slider motor current position: " + this.slide.getCurrentPosition());
//                resetSlideEncoder();
//            }
//        }
    }

    public void setupForSlidingInAuto() {
        this.slidePivot.setPosition(SLIDE_PIVOT_SLIDING);
        this.outtakePivot.setPosition(OUTTAKE_PIVOT_SLIDING);
    }

    public Action moveSliderBlocking(double increment) {
        OUTTAKE_TELEOPS = this.slide.getCurrentPosition();
        Log.d("Outtake_Slide", "Current position:" + OUTTAKE_TELEOPS );
        Log.d("Outtake_Slide", "slide is busy:" + this.slide.isBusy() );
        if(!this.slide.isBusy()) {
            int multiplier = increment > 0 ? 1 : -1;

            if(!scoreLevel3 && OUTTAKE_TELEOPS > OUTTAKE_SLIDE_MAX - 150 && increment > 0.30) {
                Log.d("Outtake_Slide", "Target position:" + OUTTAKE_TELEOPS + " | level 3 is enabled");
                scoreLevel3 = true;
                return this.prepareToScoreLevel3();
            } else if(OUTTAKE_TELEOPS < OUTTAKE_SLIDE_MAX -150){
                scoreLevel3 = false;
            }

            OUTTAKE_TELEOPS = this.slide.getCurrentPosition() + OUTTAKE_SLIDE_INCREMENT * multiplier;

            if(OUTTAKE_TELEOPS >= OUTTAKE_SLIDE_MAX) {
                OUTTAKE_TELEOPS = OUTTAKE_SLIDE_MAX;
            }
            Log.d("Outtake_Slide", "New position:" + OUTTAKE_TELEOPS + " | Current position: " + this.slide.getCurrentPosition() + " | increment: " + increment);

            return extendOuttakeTeleOps();
        }
        return new OuttakeLatchStateAction(OuttakeLatchState.CLOSED);
    }

    public Action resetSliderPosition() {
        return new SequentialAction(
                this.slide.setTargetPositionAction(-150),
                new SleepAction(0.5),
                this.slide.zeroMotorInternalsAction("OuttakeSlideMotor"));
    }

    public Action retractOuttake() {

        double sleepTime = 0.5;
        if(!isAuto) {
            sleepTime = 0.60;
        }

        return new SequentialAction(
                prepareToSlide(),
                new SleepAction(sleepTime),
 //               latchClosed(),
                new ParallelAction(
                    outtakeWireDown(),
                    this.slide.setTargetPositionAction(OUTTAKE_SLIDE_INIT, "outtakeSlide")
                )//,
//                new SleepAction(0.5),
//                prepareToTransfer()
        );
    }

    public Action latchScore1() {
        double slideServoVoltage = slidePivotVoltage.getVoltage();
        double outtakeServoVoltage = outtakePivotVoltage.getVoltage();
        double slidePivotPosition = slidePivot.getPosition();
        if(slidePivotPosition > SLIDE_PIVOT_DUMP_HIGH) {
            slidePivotPosition = SLIDE_PIVOT_DUMP;
            if (slideServoVoltage > SLIDE_PIVOT_DUMP_VOLTAGE_MAX ) {
                slidePivotPosition = SLIDE_PIVOT_DUMP_1;
            }
        }

        String servoPositions = "SlidePivot voltage: " + String.format("%.2f", slideServoVoltage) +
        " | OuttakePivot voltage: " + String.format("%.2f", outtakeServoVoltage);
        Log.d("SlidePivot_Dump_Logger", servoPositions);

        return new SequentialAction(
                new ActionUtil.ServoPositionAction(slidePivot, slidePivotPosition, "slidePivot"),
                new ActionUtil.ServoPositionAction(latch, LATCH_SCORE_1, "latch"),
                new Intake.UpdatePixelCountAction(-1),
                new OuttakeLatchStateAction(OuttakeLatchState.LATCH_1)
        );
    }

    public Action afterScore() {
        return new SequentialAction(
                new ActionUtil.ServoPositionAction(slidePivot, SLIDE_PIVOT_DUMP_2, "slidePivot"),
                this.slide.setTargetPositionAction(OUTTAKE_SLIDE_AFTER_DUMP_AUTO, "outtakeSlide")
        );
    }

    public Action strafeToAlign() {
        return new SequentialAction(
                new ActionUtil.ServoPositionAction(slidePivot, SLIDE_PIVOT_STRAFE, "slidePivot")
        );
    }

    public Action latchScore0() {
        return new SequentialAction(
                new ActionUtil.ServoPositionAction(latch, LATCH_SCORE_1, "latch"),
                new OuttakeLatchStateAction(OuttakeLatchState.LATCH_1)
        );
    }

    public Action latchScore2() {

        double slideServoVoltage = slidePivotVoltage.getVoltage();
        double outtakeServoVoltage = outtakePivotVoltage.getVoltage();
        double slidePivotPosition = slidePivot.getPosition();
        if(slidePivotPosition > SLIDE_PIVOT_DUMP_HIGH) {
            slidePivotPosition = SLIDE_PIVOT_DUMP;
            if (slideServoVoltage > SLIDE_PIVOT_DUMP_VOLTAGE_MAX ) {
                slidePivotPosition = SLIDE_PIVOT_DUMP_1;
            }
        }

        String servoPositions = "SlidePivot voltage: " + String.format("%.2f", slideServoVoltage) +
                " | OuttakePivot voltage: " + String.format("%.2f", outtakeServoVoltage);

        Log.d("SlidePivot_Dump_Logger", servoPositions);

        return new SequentialAction(
                new ActionUtil.ServoPositionAction(slidePivot, slidePivotPosition, "slidePivot"),
                new ActionUtil.ServoPositionAction(latch, LATCH_SCORE_2, "latch"),
                new Intake.UpdatePixelCountAction(-2),
                new OuttakeLatchStateAction(OuttakeLatchState.LATCH_2)
        );
    }

    public Action latchClosed() {
        return new SequentialAction(
                new ActionUtil.ServoPositionAction(latch, LATCH_CLOSED, "latch"),
                new OuttakeLatchStateAction(OuttakeLatchState.CLOSED)
        );
    }
    public Action reverseDump() {
        return new SequentialAction(
                new ActionUtil.ServoPositionAction(slidePivot, SLIDE_PIVOT_DUMP_VERY_HIGH, "slidePivot"),
            new ActionUtil.ServoPositionAction(outtakePivot, OUTTAKE_PIVOT_REVERSE_DUMP, "outtakePivot")
        );
    }

    public Action extendOuttakeLow() {
        return new SequentialAction(
                new ActionUtil.ServoPositionAction(outtakeWireServo, OUTTAKE_WIRE_MIDDLE, "outtakeWireServo"),
                this.slide.setTargetPositionAction(OUTTAKE_SLIDE_LOW, "outtakeSlide")
        );
    }

    public Action extendOuttakeCycleOne() {
        return new SequentialAction(
                new ActionUtil.ServoPositionAction(outtakeWireServo, OUTTAKE_WIRE_MIDDLE, "outtakeWireServo"),
                this.slide.setTargetPositionAction(OUTTAKE_SLIDE_CYCLES_ONE, "outtakeSlide")
        );
    }

    public Action extendOuttakeCycleTwo() {
        return new SequentialAction(
                new ActionUtil.ServoPositionAction(outtakeWireServo, OUTTAKE_WIRE_MIDDLE, "outtakeWireServo"),
                this.slide.setTargetPositionAction(OUTTAKE_SLIDE_CYCLES_TWO, "outtakeSlide")
        );
    }

    public Action extendOuttakeFarLow() {
        return new SequentialAction(
                new ActionUtil.ServoPositionAction(outtakeWireServo, OUTTAKE_WIRE_MIDDLE, "outtakeWireServo"),
                this.slide.setTargetPositionAction(OUTTAKE_SLIDE_FAR_LOW, "outtakeSlide")
        );
    }

    public Action extendOuttakeTeleOps() {
        double wirePosition = OUTTAKE_TELEOPS >= OUTTAKE_SLIDE_BELOW_LEVEL_2?OUTTAKE_WIRE_VERY_HIGH:OUTTAKE_WIRE_HIGH;
        return new SequentialAction(
                new ActionUtil.ServoPositionAction(outtakeWireServo, wirePosition, "outtakeWireServo"),
                this.slide.setTargetPositionAction(OUTTAKE_TELEOPS, "outtakeSlide")
        );
    }

    public Action prepareToSlide() {
        return new SequentialAction(
                new ActionUtil.ServoPositionAction(outtakePivot, OUTTAKE_PIVOT_SLIDING, "outtakePivot"),
                new ActionUtil.ServoPositionAction(slidePivot, SLIDE_PIVOT_SLIDING, "slidePivot")
        );
    }

    public Action prepareToTransfer() {
        return new SequentialAction(
                new ActionUtil.ServoPositionAction(latch, LATCH_CLOSED, "latch"),
                new ActionUtil.ServoPositionAction(outtakePivot, OUTTAKE_PIVOT_INIT, "outtakePivot"),
                new ActionUtil.ServoPositionAction(slidePivot, SLIDE_PIVOT_INIT, "slidePivot")
        );
    }

    public Action outtakeWireUp() {
        return new SequentialAction(
                new ActionUtil.ServoPositionAction(outtakeWireServo, OUTTAKE_WIRE_HIGH, "outtakeWire")
        );
    }

    public Action outtakeWireForHanging() {

        if(!isHangingHookUp) {
            isHangingHookUp = true;
            return new SequentialAction(
                    new ActionUtil.ServoPositionAction(outtakeWireServo, OUTTAKE_WIRE_FOR_HANGING_DOWN, "outtakeWire"),
                    new SleepAction(0.30),
                    new ActionUtil.ServoPositionAction(outtakeWireServo, OUTTAKE_WIRE_FOR_HANGING_UP, "outtakeWire")
            );
        } else {
            isHangingHookUp = false;
            return new ActionUtil.ServoPositionAction(outtakeWireServo, OUTTAKE_WIRE_SAFE_DOWN, "outtakeWire");
        }
    }

    public Action outtakeWireDown() {
        return new ActionUtil.ServoPositionAction(outtakeWireServo, OUTTAKE_WIRE_DOWN, "outtakeWire");
    }

    public Action prepareToScore() {

        if(scoreLevel3) {
            return prepareToScoreLevel3();
        }

        double outtakeDumpPosition = isAuto?OUTTAKE_PIVOT_DUMP_LOW:OUTTAKE_PIVOT_DUMP_MID;
        double slideDumpPosition = SLIDE_PIVOT_DUMP;

        return new SequentialAction(
                new ActionUtil.ServoPositionAction(latch, LATCH_CLOSED, "latch"),
                new ActionUtil.ServoPositionAction(outtakePivot, outtakeDumpPosition, "outtakePivot"),
                new ActionUtil.ServoPositionAction(slidePivot, slideDumpPosition, "slidePivot")
        );
    }

    public Action prepareToScoreCycle() {

        double outtakeDumpPosition = OUTTAKE_PIVOT_DUMP_CYCLE;
        double slideDumpPosition = SLIDE_PIVOT_DUMP;

        Log.d("Outtake_Pivot_Servo", "Outtake Dump Servo Position: " + String.format("%.2f", outtakeDumpPosition));
        Log.d("Slide_Pivot_Servo", "Slide Dump Servo Position: " + String.format("%.2f", slideDumpPosition));

        return new SequentialAction(
                new ActionUtil.ServoPositionAction(latch, LATCH_CLOSED, "latch"),
                new ActionUtil.ServoPositionAction(outtakePivot, outtakeDumpPosition, "outtakePivot"),
                new ActionUtil.ServoPositionAction(slidePivot, slideDumpPosition, "slidePivot")
        );
    }

    public Action prepareToScoreLevel3() {
        double outtakeDumpPosition = OUTTAKE_PIVOT_DUMP_VERY_HIGH;
        double slideDumpPosition = SLIDE_PIVOT_DUMP_VERY_HIGH;

        Log.d("prepareToScoreLevel3", "Level 3 is enabled " + String.format("(%.2f, %.2f)", outtakeDumpPosition, slideDumpPosition));

        return new SequentialAction(
                new ActionUtil.ServoPositionAction(outtakeWireServo, OUTTAKE_WIRE_VERY_HIGH, "outtakeWireServo"),
                new ActionUtil.ServoPositionAction(latch, LATCH_CLOSED, "latch"),
                new ActionUtil.ServoPositionAction(outtakePivot, outtakeDumpPosition, "outtakePivot"),
                new ActionUtil.ServoPositionAction(slidePivot, slideDumpPosition, "slidePivot")
        );
    }

    public String getServoPositions() {

//        double slideServoVoltage = slidePivotVoltage.getVoltage();
//        double slideServoPosition = slidePivot.getPosition();
//
//        slidePivotVoltages.add(slideServoVoltage);
//
//        if(backdropTouched &&
//            slidePivotVoltages.getMean() <= SLIDE_PIVOT_DUMP_VOLTAGE_MIN) {
//            backdropTouched= false;
//        } else if (slideServoPosition > SLIDE_PIVOT_DUMP_HIGH &&
//                slidePivotVoltages.getMean() >= SLIDE_PIVOT_DUMP_VOLTAGE_MAX) {
//            backdropTouched = true;
//        }

        return "SlidePivot: " + String.format("%.2f", slidePivot.getPosition()) +
                " | OuttakePivot: " + String.format("%.2f", this.outtakePivot.getPosition()) +
                " | Latch: " + String.format("%.2f", this.latch.getPosition()) +
                " | SlidePivot voltage: " + String.format("%.2f", slidePivotVoltages.getMean()) +
                " | backdropTouched: " + backdropTouched;
    }

    public boolean checkSlidePivotPosition() {
        double slideServoVoltage = slidePivotVoltage.getVoltage();
        double slideServoPosition = slidePivot.getPosition();

        slidePivotVoltages.add(slideServoVoltage);

        if(backdropTouched &&
                slidePivotVoltages.getMean() <= SLIDE_PIVOT_DUMP_VOLTAGE_MIN) {
            backdropTouched= false;
        } else if (slideServoPosition > SLIDE_PIVOT_DUMP_HIGH &&
                slidePivotVoltages.getMean() >= SLIDE_PIVOT_DUMP_VOLTAGE_MAX) {
            backdropTouched = true;
        }

        return backdropTouched;
    }

    public void resetSlideEncoder() {
        this.slide.zeroMotorInternals();
        this.slide.setTargetPosition(0);
    }

    public Action resetSlideEncoderAction() {
        return this.slide.zeroMotorInternalsAction("SlideMotor");
    }

    public boolean hasOuttakeReached() {
        return backdropTouched;
    }

    public int getMotorCurrentPosition() {
        return this.slide.getCurrentPosition();
    }

    public int getMotorTargetPosition() {
        return this.slide.getTargetPosition();
    }

    public boolean isMotorBusy() {
        return this.slide.isBusy();
    }

    private class SlideServoPositionAction implements Action {
        double targetVoltage;
        boolean blocking;

        double beginPosition;
        double beginVoltage;

        Servo slideServo;
        AnalogInput slidePivotVoltage;

        public SlideServoPositionAction(Servo slideServo, AnalogInput slidePivotVoltage, double targetVoltage, boolean blocking) {
            this.slideServo = slideServo;
            this.slidePivotVoltage = slidePivotVoltage;
            this.targetVoltage = targetVoltage;
            this.blocking = blocking;
            this.beginPosition = slideServo.getPosition();
            this.beginVoltage = slidePivotVoltage.getVoltage();
        }

        @Override
        public boolean run(TelemetryPacket packet) {

            double currentPosition = slideServo.getPosition();
            double currentVoltage = slidePivotVoltage.getVoltage();
            boolean targetReached = false;

            double targetPosition = currentPosition;
            if(targetVoltage > currentVoltage) {
                targetPosition = currentPosition+0.005;
            } else {
                targetPosition = currentPosition-0.005;
            }

            slideServo.setPosition(Range.clip(targetPosition, 0.0, 1.0));

            if( (targetVoltage > beginVoltage && currentVoltage < targetVoltage) ||
                    (targetVoltage < beginVoltage && currentVoltage > targetVoltage) ||
                Math.abs(currentVoltage - targetVoltage) < 0.02)
            {
                targetReached = true;
            } else {
                slideServo.setPosition(currentPosition+0.005);
            }

            if (blocking) {
                if(targetReached) {
                    return false;
                }
                return true;
            }
            return false;
        }
    }

    public boolean checkSlidePivotOverreached() {
        double slideServoVoltage = slidePivotVoltage.getVoltage();
        double slideServoPosition = slidePivot.getPosition();

        if(backdropTouched &&
                slideServoVoltage <= SLIDE_PIVOT_DUMP_VOLTAGE_SUPER_MIN) {
            return false;
        } else if (slideServoPosition > SLIDE_PIVOT_DUMP_HIGH &&
                slideServoVoltage >= SLIDE_PIVOT_DUMP_VOLTAGE_SUPER_MAX) {
            return true;
        }

        return false;
    }

    public void resetSlidePivotDumpVoltageLimit() {
        double servoVoltage = slidePivotVoltage.getVoltage();
        SLIDE_PIVOT_DUMP_VOLTAGE_MIN = servoVoltage + 0.02;
        SLIDE_PIVOT_DUMP_VOLTAGE_MAX = servoVoltage + 0.07;
        SLIDE_PIVOT_DUMP_VOLTAGE_SUPER_MIN = servoVoltage + 0.05;
        SLIDE_PIVOT_DUMP_VOLTAGE_SUPER_MAX = servoVoltage + 0.12;

        Log.d("SlidePivot_Logger", "SLIDE_PIVOT_DUMP_VOLTAGE_MIN=" + String.format("%3.2f", SLIDE_PIVOT_DUMP_VOLTAGE_MIN) +
                        " | SLIDE_PIVOT_DUMP_VOLTAGE_MAX=" + String.format("%3.2f", SLIDE_PIVOT_DUMP_VOLTAGE_MAX) +
                        " | SLIDE_PIVOT_DUMP_VOLTAGE_SUPER_MIN=" + String.format("%3.2f", SLIDE_PIVOT_DUMP_VOLTAGE_SUPER_MIN) +
                        " | SLIDE_PIVOT_DUMP_VOLTAGE_SUPER_MAX=" + String.format("%3.2f", SLIDE_PIVOT_DUMP_VOLTAGE_SUPER_MAX));
    }

    public Action resetSlidePivotServoDumpVoltageLimit() {
        return new SlideServoDumpVoltageLimitAction();
    }

    private class SlideServoDumpVoltageLimitAction implements Action {
        public SlideServoDumpVoltageLimitAction() {}

        @Override
        public boolean run(TelemetryPacket packet) {
            resetSlidePivotDumpVoltageLimit();
            return false;
        }
    }

    public Action moveUpOuttakeFixerServo() {
        int fixerPosition = fixerServoPosition.level;
        fixerPosition = fixerPosition + 5;

        fixerServoPosition = FixerServoPosition.getPosition(fixerPosition);

        if(fixerServoPosition == null) {
            fixerServoPosition = FixerServoPosition.LEVEL_0;
        }

        return new SequentialAction(
                new ActionUtil.ServoPositionAction(outtakeFixerServo,
                        fixerServoPosition.position,
                        "outtakeFixerServo")
        );
    }

    public Action moveDownOuttakeFixerServo() {

        int fixerPosition = fixerServoPosition.level;
        fixerPosition = fixerPosition -5;

        fixerServoPosition = FixerServoPosition.getPosition(fixerPosition);

        if(fixerServoPosition == null) {
            fixerServoPosition = FixerServoPosition.LEVEL_0;
        }

        return new SequentialAction(
                new ActionUtil.ServoPositionAction(outtakeFixerServo,
                        fixerServoPosition.position,
                        "outtakeFixerServo")
        );
    }

    public Action resetOuttakeFixerServo() {
        fixerServoPosition  = FixerServoPosition.LEVEL_0;

        return new SequentialAction(
                new ActionUtil.ServoPositionAction(outtakeFixerServo,
                        fixerServoPosition.position,
                        "outtakeFixerServo")
        );
    }

    public enum FixerServoPosition {
        LEVEL_0 (5, OUTTAKE_FIXER_INIT),
        LEVEL_1 (10, OUTTAKE_FIXER_LEVEL_1),
        LEVEL_2 (20, OUTTAKE_FIXER_LEVEL_2),
        LEVEL_3 (30, OUTTAKE_FIXER_LEVEL_3),
        LEVEL_4 (40, OUTTAKE_FIXER_LEVEL_4),
        LEVEL_1_5 (15, OUTTAKE_FIXER_LEVEL_1_5),
        LEVEL_2_5 (25, OUTTAKE_FIXER_LEVEL_2_5),
        LEVEL_3_5 (35, OUTTAKE_FIXER_LEVEL_3_5);

        private static final Map<Integer, FixerServoPosition> BY_NUMBER = new HashMap<>();

        static {
            for (FixerServoPosition e : values()) {
                BY_NUMBER.put(Integer.valueOf(e.level), e);
            }
        }

        public final int level;
        public final double position;

        private FixerServoPosition(int level, double position) {
            this.level = level;
            this.position = position;
        }

        public static FixerServoPosition getPosition(int level) {
            int temp = level;
            if(level > 40) {
                temp = 10;
            }

            if(level < 5) {
                temp = 5;
            }
            return BY_NUMBER.get(Integer.valueOf(level));
        }
    }

    public FixerServoPosition getFixerServoLevel() {
        return this.fixerServoPosition;
    }

}
