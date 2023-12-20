package org.firstinspires.ftc.teamcode.subsystem;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.software.ActionUtil;
import org.firstinspires.ftc.teamcode.utils.hardware.HardwareCreator;
import org.firstinspires.ftc.teamcode.utils.hardware.MotorWithPID;
import org.firstinspires.ftc.teamcode.utils.control.PIDCoefficients;

@Config
public class Outtake {
    public static PIDCoefficients outtakePID = new PIDCoefficients(0.01, 0, 0.0004);

    public static int OUTTAKE_SLIDE_MAX = 2200;
    public static int OUTTAKE_SLIDE_ABOVE_LEVEL_2 = 2150;
    public static int OUTTAKE_SLIDE_BELOW_LEVEL_2 = 1850;
    public static int OUTTAKE_SLIDE_ABOVE_LEVEL_1 = 1550;
    public static int OUTTAKE_SLIDE_BELOW_LEVEL_1 = 1250;

    public static int OUTTAKE_SLIDE_HIGH = OUTTAKE_SLIDE_ABOVE_LEVEL_2;
    public static int OUTTAKE_TELEOPS = OUTTAKE_SLIDE_BELOW_LEVEL_1;
    public static int OUTTAKE_SLIDE_MID = 1250;
    public static int OUTTAKE_SLIDE_CYCLES_ONE = 1000;
    public static int OUTTAKE_SLIDE_CYCLES_TWO = 1150;
    public static int OUTTAKE_SLIDE_LOW = 865;
    public static int OUTTAKE_SLIDE_INIT = 0;

    public static int OUTTAKE_SLIDE_INCREMENT= 300;

    public static double LATCH_CLOSED = 0.55;
    public static double LATCH_SCORE_1 = 0.415;
    public static double LATCH_SCORE_2 = 0.47;

    public static double OUTTAKE_PIVOT_REVERSE_DUMP = 0.01;
    public static double OUTTAKE_PIVOT_INIT = 0.175;
    public static double OUTTAKE_PIVOT_SLIDING = 0.23;
    public static double OUTTAKE_PIVOT_DUMP_LOW = 0.38;
    public static double OUTTAKE_PIVOT_DUMP_MID = 0.40;

    public static double OUTTAKE_PIVOT_DUMP_HIGH = 0.56;

    public static double OUTTAKE_PIVOT_DUMP_CYCLE = 0.39;

    public static double OUTTAKE_PIVOT_DUMP_VERY_HIGH = 0.62;

    public static double SLIDE_PIVOT_INIT = 0.438;
    public static double SLIDE_PIVOT_SLIDING = 0.52;
    public static double SLIDE_PIVOT_DUMP = 0.25;

    public static double SLIDE_PIVOT_DUMP_2 = 0.270;

    public static double SLIDE_PIVOT_DUMP_HIGH = 0.10;

    public static double SLIDE_PIVOT_DUMP_VERY_HIGH = 0.0;

    public static double OUTTAKE_WIRE_DOWN = 0.72;

    public static double OUTTAKE_WIRE_SAFE_DOWN = 0.70;

    public static double OUTTAKE_WIRE_MIDDLE = 0.50;
    public static double OUTTAKE_WIRE_HIGH = 0.45;
    public static double OUTTAKE_WIRE_VERY_HIGH = 0.37;

    public static double OUTTAKE_WIRE_FOR_HANGING_DOWN = 0.73;
    public static double OUTTAKE_WIRE_FOR_HANGING_UP = 0.38;

    final MotorWithPID slide;
    public boolean slidePIDEnabled = true;
    final Servo latch;
    final Servo slidePivot;
    final Servo outtakePivot;

    final Servo outtakeWireServo;

    final AnalogInput slidePivotVoltage;
    final AnalogInput outtakePivotVoltage;

    public boolean isAuto = true;

    private boolean scoreLevel3 = false;

    public boolean isHangingHookUp = false;

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
        this.slide.getMotor().setPower(-0.3);
    }

    public void finishPrepTeleop() {
        this.slide.getMotor().setPower(0);
    }

    public void initialize() {
        this.slide.setCurrentPosition(0);
        this.slide.setTargetPosition(0);
        this.slidePivot.setPosition(SLIDE_PIVOT_INIT);
        this.outtakePivot.setPosition(OUTTAKE_PIVOT_INIT);
        this.latch.setPosition(LATCH_CLOSED);
        this.outtakeWireServo.setPosition(OUTTAKE_WIRE_SAFE_DOWN);
    }

    public void update() {
        if (slidePIDEnabled) {
            slide.update();
        }
    }

    public Action moveSliderBlocking(double increment) {
        OUTTAKE_TELEOPS = this.slide.getCurrentPosition();
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

    public Action retractOuttake() {
        return new SequentialAction(
                prepareToSlide(),
                new SleepAction(0.3),
                latchClosed(),
                new ParallelAction(
                    outtakeWireDown(),
                    this.slide.setTargetPositionAction(OUTTAKE_SLIDE_INIT, "outtakeSlide")
                )
        );
    }

    public Action latchScore1() {
        return new SequentialAction(
                new ActionUtil.ServoPositionAction(latch, LATCH_SCORE_1, "latch"),
                new Intake.UpdatePixelCountAction(-1),
                new OuttakeLatchStateAction(OuttakeLatchState.LATCH_1)
        );
    }

    public Action afterScore() {
        return new SequentialAction(
                new ActionUtil.ServoPositionAction(slidePivot, SLIDE_PIVOT_DUMP_2, "slidePivot")
        );
    }

    public Action latchScore0() {
        return new SequentialAction(
                new ActionUtil.ServoPositionAction(latch, LATCH_SCORE_1, "latch"),
                new OuttakeLatchStateAction(OuttakeLatchState.LATCH_1)
        );
    }

    public Action latchScore2() {
        return new SequentialAction(
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

        Log.d("Outtake_Pivot_Servo", "Outtake Dump Servo Position: " + String.format("%.2f", outtakeDumpPosition));
        Log.d("Slide_Pivot_Servo", "Slide Dump Servo Position: " + String.format("%.2f", slideDumpPosition));

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
        return "SlidePivot: " + String.format("%.2f", this.slidePivot.getPosition()) +
                " | OuttakePivot: " + String.format("%.2f", this.outtakePivot.getPosition()) +
                " | Latch: " + String.format("%.2f", this.latch.getPosition());
    }

    public void resetSlideEncoder() {
        this.slide.zeroMotorInternals();
    }

}
