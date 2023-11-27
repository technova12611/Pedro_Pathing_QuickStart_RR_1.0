package org.firstinspires.ftc.teamcode.subsystem;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.software.ActionUtil;
import org.firstinspires.ftc.teamcode.utils.hardware.HardwareCreator;
import org.firstinspires.ftc.teamcode.utils.hardware.MotorWithVelocityPID;
import org.firstinspires.ftc.teamcode.utils.control.PIDCoefficients;

@Config
public class Intake {
    public static int INTAKE_SPEED = 800;

    final MotorWithVelocityPID intakeMotor;
    public static PIDCoefficients intakeMotorPid = new PIDCoefficients(0.0007, 0, 0.1);

    final Servo stackIntakeLinkage;
    final Servo stackIntakeServoLeft;
    final Servo stackIntakeServoRight;

    final CRServo bottomRollerServo;

    public static double STACK_INTAKE_LEFT_INIT = 0.01;
    public static double STACK_INTAKE_LEFT_PRELOAD = 0.19;
    public static double STACK_INTAKE_LEFT_1ST_PIXEL = 0.64;
    public static double STACK_INTAKE_LEFT_2nd_PIXEL = 1.0;

    public static double STACK_INTAKE_RIGHT_INIT = 0.01;
    public static double STACK_INTAKE_RIGHT_PRELOAD = 0.19;
    public static double STACK_INTAKE_RIGHT_1ST_PIXEL = 0.64;
    public static double STACK_INTAKE_RIGHT_2nd_PIXEL = 1.0;

    public static double STACK_INTAKE_LINKAGE_INIT = 0.95;
    public static double STACK_INTAKE_LINKAGE_DOWN = 0.4925;
    public static double STACK_INTAKE_LINKAGE_UP = 0.89;

    final DigitalChannel beamBreaker1;

    final DigitalChannel beamBreaker2;

    public Intake(HardwareMap hardwareMap) {
        this.intakeMotor = new MotorWithVelocityPID(HardwareCreator.createMotor(hardwareMap, "intake"), intakeMotorPid);
        this.intakeMotor.setMaxPower(1.0);
        this.intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        stackIntakeServoLeft = HardwareCreator.createServo(hardwareMap, "stackIntakeServoLeft");
        stackIntakeServoRight = HardwareCreator.createServo(hardwareMap, "stackIntakeServoRight");
        stackIntakeLinkage = HardwareCreator.createServo(hardwareMap, "stackIntakeLinkage");
        bottomRollerServo = HardwareCreator.createCRServo(hardwareMap, "bottomRollerServo", HardwareCreator.ServoType.AXON);

        beamBreaker1 = hardwareMap.get(DigitalChannel.class, "beamBreaker1");
        beamBreaker2 = hardwareMap.get(DigitalChannel.class, "beamBreaker2");
    }

    public void initialize(boolean isAuto) {
        if( isAuto) {
            stackIntakeLinkage.setPosition(STACK_INTAKE_LINKAGE_INIT);
            stackIntakeServoLeft.setPosition(STACK_INTAKE_LEFT_PRELOAD);
            stackIntakeServoRight.setPosition(STACK_INTAKE_RIGHT_PRELOAD);
        }
        else {
            stackIntakeLinkage.setPosition(STACK_INTAKE_LINKAGE_UP);
            stackIntakeServoLeft.setPosition(STACK_INTAKE_LEFT_INIT);
            stackIntakeServoRight.setPosition(STACK_INTAKE_RIGHT_INIT);
        }

        intakeState = IntakeState.OFF;
        stackIntakeState = StackIntakeState.INIT;
    }

    public enum IntakeState {
        OFF,
        REVERSING,
        ON
    }

    public enum StackIntakeState {
        INIT,
        UP,
        DOWN
    }
    public IntakeState intakeState;

    public StackIntakeState stackIntakeState;

    public class IntakeStateAction implements Action {
        IntakeState newState;

        public IntakeStateAction(IntakeState newState) {
            this.newState = newState;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            intakeState = newState;
            return false;
        }
    }

    public class StackIntakeStateAction implements Action {
        StackIntakeState newState;

        public StackIntakeStateAction(StackIntakeState newState) {
            this.newState = newState;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            stackIntakeState = newState;
            return false;
        }
    }

    public Action intakeOn() {
        return new SequentialAction(
                //intakeMotor.setTargetVelocityAction(INTAKE_SPEED),
                new ActionUtil.DcMotorExPowerAction(intakeMotor.getMotor(), INTAKE_SPEED / 1000.0),
                new IntakeStateAction(IntakeState.ON),
                new ActionUtil.CRServoAction(bottomRollerServo, 1.0)
        );
    }

    public Action intakeReverse() {
        return new SequentialAction(
                //intakeMotor.setTargetVelocityAction(-INTAKE_SPEED),
                new ActionUtil.DcMotorExPowerAction(intakeMotor.getMotor(), -INTAKE_SPEED / 1000.0),
                new IntakeStateAction(IntakeState.REVERSING)
        );
    }

    public Action intakeReverseThenStop() {
        return new SequentialAction(
                intakeReverse(),
                new SleepAction(2.0),
                intakeOff()
        );
    }

    public Action intakeOff() {
        return new SequentialAction(
                intakeMotor.setTargetVelocityAction(0),
                new ActionUtil.DcMotorExPowerAction(intakeMotor.getMotor(), 0),
                new IntakeStateAction(IntakeState.OFF),
                new ActionUtil.CRServoAction(bottomRollerServo, 0.0)
        );
    }

    public Action prepareStackIntake() {
        return new SequentialAction(
                //intakeMotor.setTargetVelocityAction(0),
                new ActionUtil.ServoPositionAction(stackIntakeLinkage, STACK_INTAKE_LINKAGE_DOWN),
                new ActionUtil.ServoPositionAction(stackIntakeServoLeft, STACK_INTAKE_LEFT_INIT),
                new ActionUtil.ServoPositionAction(stackIntakeServoRight, STACK_INTAKE_RIGHT_INIT),
                new StackIntakeStateAction(StackIntakeState.DOWN)
        );
    }

    public Action prepareTeleOpsIntake() {
        return new SequentialAction(
                new ActionUtil.ServoPositionAction(stackIntakeLinkage, STACK_INTAKE_LINKAGE_UP),
                new ActionUtil.ServoPositionAction(stackIntakeServoLeft, STACK_INTAKE_LEFT_INIT),
                new ActionUtil.ServoPositionAction(stackIntakeServoRight, STACK_INTAKE_RIGHT_INIT),
                new StackIntakeStateAction(StackIntakeState.UP)
        );
    }

    public Action intakeStackedPixels() {
        return new SequentialAction(
                intakeOn(),
                new ActionUtil.ServoPositionAction(stackIntakeServoRight, STACK_INTAKE_RIGHT_1ST_PIXEL),
                new SleepAction(0.25),
                new ActionUtil.ServoPositionAction(stackIntakeServoLeft, STACK_INTAKE_LEFT_1ST_PIXEL),
                new SleepAction(0.5),
                new ActionUtil.ServoPositionAction(stackIntakeServoRight, STACK_INTAKE_RIGHT_2nd_PIXEL),
                new SleepAction(0.25),
                new ActionUtil.ServoPositionAction(stackIntakeServoLeft, STACK_INTAKE_LEFT_2nd_PIXEL),
                new SleepAction(0.5)
        );
    }

    public Action scorePurplePreload() {
        return new SequentialAction(
                new ActionUtil.ServoPositionAction(stackIntakeLinkage, STACK_INTAKE_LINKAGE_DOWN),
                new SleepAction(0.25),
                new ActionUtil.ServoPositionAction(stackIntakeServoLeft, STACK_INTAKE_LEFT_INIT),
                new ActionUtil.ServoPositionAction(stackIntakeServoRight, STACK_INTAKE_RIGHT_INIT),
                new SleepAction(0.25)
        );
    }

    public void update() {
        //intakeMotor.update();
    }
}
