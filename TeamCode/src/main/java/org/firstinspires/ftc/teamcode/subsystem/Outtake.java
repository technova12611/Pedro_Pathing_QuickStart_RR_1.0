package org.firstinspires.ftc.teamcode.subsystem;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
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
    public static int OUTTAKE_TELEOP = 1200;

    public static int OUTTAKE_SLIDE_VERY_HIGH = 1800;

    public static int OUTTAKE_SLIDE_HIGH = 1650;
    public static int OUTTAKE_SLIDE_MID = 1150;
    public static int OUTTAKE_SLIDE_LOW = 750;
    public static int OUTTAKE_SLIDE_INIT = 0;

    public static int OUTTAKE_SLIDE_INCREMENT= 360;

    public static double LATCH_CLOSED = 0.55;
    public static double LATCH_SCORE_1 = 0.415;
    public static double LATCH_SCORE_2 = 0.48;

    public static double OUTTAKE_PIVOT_INIT = 0.18;
    public static double OUTTAKE_PIVOT_SLIDING = 0.23;
    public static double OUTTAKE_PIVOT_DUMP_LOW = 0.37;
    public static double OUTTAKE_PIVOT_DUMP_MID = 0.42;

    public static double OUTTAKE_PIVOT_DUMP_HIGH = 0.48;

    public static double SLIDE_PIVOT_INIT = 0.45;
    public static double SLIDE_PIVOT_SLIDING = 0.52;
    public static double SLIDE_PIVOT_DUMP = 0.25;
    public static double SLIDE_PIVOT_HIGH = 0.05;

    public static double OUTTAKE_WIRE_DOWN = 0.81;
    public static double OUTTAKE_WIRE_MIDDLE = 0.35;
    public static double OUTTAKE_WIRE_HIGH = 0.27;

    final MotorWithPID slide;
    public boolean slidePIDEnabled = true;
    final Servo latch;
    final Servo slidePivot;
    final Servo outtakePivot;

    final Servo outtakeWireServo;

    final AnalogInput slidePivotVoltage;
    final AnalogInput outtakePivotVoltage;

    public boolean isAuto = true;

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
        this.outtakeWireServo.setPosition(OUTTAKE_WIRE_DOWN);
    }

    public void update() {
        if (slidePIDEnabled) {
            slide.update();
        }
    }

    public Action moveSliderBlocking(double increment) {
        OUTTAKE_TELEOP = this.slide.getCurrentPosition();
        if(!this.slide.isBusy()) {
            int multiplier = increment > 0 ? 1 : -1;
            OUTTAKE_TELEOP = this.slide.getCurrentPosition() + OUTTAKE_SLIDE_INCREMENT * multiplier;
            Log.d("Outtake Slide", "New position:" + OUTTAKE_TELEOP + " | Current position: " + this.slide.getCurrentPosition());
            return this.slide.setTargetPositionAction(OUTTAKE_TELEOP);
        }
        return new OuttakeLatchStateAction(OuttakeLatchState.CLOSED);
    }

    public Action retractOuttake() {
        return new SequentialAction(
                prepareToSlide(),
                new ActionUtil.ServoPositionAction(outtakeWireServo, OUTTAKE_WIRE_DOWN, "outtakeWireServo"),
                this.slide.setTargetPositionAction(OUTTAKE_SLIDE_INIT)
        );
    }

    public Action latchScore1() {
        return new SequentialAction(
                new ActionUtil.ServoPositionAction(latch, LATCH_SCORE_1, "latch"),
                new Intake.UpdatePixelCountAction(-1),
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

    public Action extendOuttakeLow() {
        return new SequentialAction(
                new ActionUtil.ServoPositionAction(outtakeWireServo, OUTTAKE_WIRE_MIDDLE, "outtakeWireServo"),
                this.slide.setTargetPositionAction(OUTTAKE_SLIDE_LOW)
        );
    }

    public Action extendOuttakeMid() {
        return new SequentialAction(
                new ActionUtil.ServoPositionAction(outtakeWireServo, OUTTAKE_WIRE_MIDDLE, "outtakeWireServo"),
                this.slide.setTargetPositionAction(OUTTAKE_SLIDE_MID)
        );
    }

    public Action prepareToSlide() {
        return new SequentialAction(
                new ActionUtil.ServoPositionAction(latch, LATCH_CLOSED, "latch"),
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

    public Action prepareToScore() {
        double outtakeDumpPosition = OUTTAKE_PIVOT_DUMP_LOW;
        if(this.slide.getCurrentPosition() > OUTTAKE_SLIDE_LOW + 100 && this.slide.getCurrentPosition() < OUTTAKE_SLIDE_HIGH - 100) {
            outtakeDumpPosition = OUTTAKE_PIVOT_DUMP_MID;
        } else if (this.slide.getCurrentPosition() > OUTTAKE_SLIDE_MID + 100) {
            outtakeDumpPosition = OUTTAKE_PIVOT_DUMP_HIGH;
        }

        Log.d("Outtake_Servo", "Outtake Dump Servo Position: " + String.format("%.2f", outtakeDumpPosition));
        Log.d("Outtake_Slide", "Slide Target Position: " + this.slide.getTargetPosition());
        Log.d("Outtake_Slide", "Slide Current position: " + this.slide.getCurrentPosition());

        return new SequentialAction(
                new ActionUtil.ServoPositionAction(latch, LATCH_CLOSED, "latch"),
                new ActionUtil.ServoPositionAction(outtakePivot, outtakeDumpPosition, "outtakePivot"),
                new ActionUtil.ServoPositionAction(slidePivot, SLIDE_PIVOT_DUMP, "slidePivot")
        );
    }

    public String getServoPositions() {
        return "SlidePivot: " + String.format("%.2f", this.slidePivot.getPosition()) +
                "OuttakePivot: " + String.format("%.2f", this.outtakePivot.getPosition()) +
                "Latch: " + String.format("%.2f", this.latch.getPosition());
    }

}
