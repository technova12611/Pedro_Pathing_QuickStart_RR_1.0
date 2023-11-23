package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
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
    public static int OUTTAKE_TELEOP = 660;

    public static int OUTTAKE_SLIDE_HIGH = 1800;
    public static int OUTTAKE_SLIDE_MID = 1200;
    public static int OUTTAKE_SLIDE_LOW = 400;
    public static int OUTTAKE_SLIDE_INIT = 0;

    public static double LATCH_CLOSED = 0.545;
    public static double LATCH_SCORE_1 = 0.415;
    public static double LATCH_SCORE_2 = 0.8;

    public static double OUTTAKE_PIVOT_INIT = 0.17;
    public static double OUTTAKE_PIVOT_SLIDE = 0.23;
    public static double OUTTAKE_PIVOT_DUMP = 0.29;

    public static double SLIDE_PIVOT_INIT = 0.47;
    public static double SLIDE_PIVOT_DUMP = 0.3;
    public static double SLIDE_PIVOT_HIGH = 0.05;

    public static double OUTTAKE_WIRE_DOWN = 0.81;
    public static double OUTTAKE_WIRE_MIDDLE = 0.5;
    public static double OUTTAKE_WIRE_HIGH = 0.4;

    final MotorWithPID slide;
    public boolean slidePIDEnabled = true;
    final Servo latch;
    final Servo slidePivot;
    final Servo outtakePivot;

    final Servo outtakeWireServo;

    public Outtake(HardwareMap hardwareMap) {
        if (Memory.outtakeSlide != null) { // Preserve motor zero position
            this.slide = Memory.outtakeSlide;
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
    }

    public void prepTeleop() {
        this.slide.getMotor().setPower(-0.3);
    }

    public void finishPrepTeleop() {
        this.slide.getMotor().setPower(0);
    }

    public void initialize() {
        this.slide.setTargetPosition(0);
        this.slidePivot.setPosition(SLIDE_PIVOT_INIT);
        this.outtakePivot.setPosition(OUTTAKE_PIVOT_INIT);
        this.latch.setPosition(LATCH_CLOSED);
        this.outtakeWireServo.setPosition(OUTTAKE_WIRE_DOWN);
    }

    public void resetMotors() {
        this.slide.setCurrentPosition(0);
    }

    public void update() {
        if (slidePIDEnabled) {
            slide.update();
        }
    }

    public void setSlidePower(double power) {
        slide.getMotor().setPower(power);
    }
    public void lockPosition() {
        OUTTAKE_TELEOP = this.slide.getCurrentPosition();
        this.slide.setTargetPosition(OUTTAKE_TELEOP);
    }

    public Action extendOuttakeMidBlocking() {
        return this.slide.setTargetPositionActionBlocking(OUTTAKE_SLIDE_MID);
    }

    public Action extendOuttakeTeleopBlocking() {
        return this.slide.setTargetPositionActionBlocking(OUTTAKE_TELEOP);
    }
    public Action extendOuttakeLowBlocking() {
        return this.slide.setTargetPositionActionBlocking(OUTTAKE_SLIDE_LOW);
    }
    public Action retractOuttake() {
        return new SequentialAction(
                prepareSlide(),
                this.slide.setTargetPositionAction(OUTTAKE_SLIDE_INIT)
        );
    }

    public Action latchScore1() {
        return new ActionUtil.ServoPositionAction(latch, LATCH_SCORE_1);
    }

    public Action latchScoring2() {
        return new ActionUtil.ServoPositionAction(latch, LATCH_SCORE_2);
    }

    public Action latchClosed() {
        return new ActionUtil.ServoPositionAction(latch, LATCH_CLOSED);
    }

    public Action extendOuttakeLow() {
        return new SequentialAction(
                prepareSlide(),
                this.slide.setTargetPositionAction(OUTTAKE_SLIDE_LOW)
        );
    }

    public Action extendOuttakeMid() {
        return new SequentialAction(
                prepareSlide(),
                this.slide.setTargetPositionAction(OUTTAKE_SLIDE_MID)
        );
    }

    public Action extendOuttakeHigh() {
        return new SequentialAction(
                prepareSlide(),
                this.slide.setTargetPositionAction(OUTTAKE_SLIDE_HIGH)
        );
    }

    public Action prepareSlide() {
        return new SequentialAction(
                new ActionUtil.ServoPositionAction(latch, LATCH_CLOSED),
                new ActionUtil.ServoPositionAction(outtakePivot, OUTTAKE_PIVOT_SLIDE),
                new ActionUtil.ServoPositionAction(slidePivot, SLIDE_PIVOT_INIT),
                new SleepAction(0.2)
        );
    }

}
