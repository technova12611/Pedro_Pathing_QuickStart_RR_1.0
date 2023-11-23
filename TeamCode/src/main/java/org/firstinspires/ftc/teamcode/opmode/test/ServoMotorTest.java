package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.control.PIDCoefficients;
import org.firstinspires.ftc.teamcode.utils.hardware.HardwareCreator;
import org.firstinspires.ftc.teamcode.utils.hardware.MotorWithPID;
import org.firstinspires.ftc.teamcode.utils.hardware.MotorWithVelocityPID;

@Config
//@Disabled
@TeleOp(group = "Test")
public class ServoMotorTest extends LinearOpMode {

    public static PIDCoefficients outtakePID = new PIDCoefficients(0.01, 0, 0.0004);
    public static PIDCoefficients intakeMotorPid = new PIDCoefficients(0.0007, 0, 0.1);

    Servo stackIntakeServoLeft;
    Servo stackIntakeServoRight;
    Servo stackIntakeLinkage;
    CRServo bottomRollerServo;
    Servo outtakeLatch;
    Servo outtakePivot;
    Servo slidePivot;
    Servo hangServoRight;
    Servo hangServoLeft;
    Servo droneLatch;
    Servo tilt;
    Servo outtakeWireServo;

    MotorWithPID slide;
    MotorWithVelocityPID intakeMotor;

    public static double STACK_INTAKE_LEFT_VALUE = 0.0;
    public static double STACK_INTAKE_RIGHT_VALUE = 0.0;
    public static double STACK_INTAKE_LINKAGE_VALUE = 0.0;
    public static double BOTTOM_ROLLER_CRSERVO_VALUE = 0.0;
    public static double OUTTAKE_LATCH_VALUE = 0.0;
    public static double OUTTAKE_PIVOT_VALUE = 0.0;
    public static double SLIDE_PIVOT_VALUE = 0.0;
    public static double HANG_SERVO_RIGHT_VALUE = 0.0;
    public static double HANG_SERVO_LEFT_VALUE = 0.0;
    public static double DRONE_LATCH_VALUE = 0.0;
    public static double DRONE_TILT_VALUE = 0.0;
    public static double OUTTAKE_WIRE_SERVO_VALUE = 0.0;

    public static int OUTTAKE_MOTOR_POSITION = 0;

    public static int INTAKE_MOTOR_VELOCITY = 0;

    public static double INTAKE_MOTOR_POWER = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        this.bottomRollerServo = HardwareCreator.createCRServo(hardwareMap, "bottomRollerServo");

        this.stackIntakeServoLeft = HardwareCreator.createServo(hardwareMap, "stackIntakeServoLeft");
        this.stackIntakeServoRight = HardwareCreator.createServo(hardwareMap, "stackIntakeServoRight");
        this.stackIntakeLinkage = HardwareCreator.createServo(hardwareMap, "stackIntakeLinkage");
        this.outtakeLatch = HardwareCreator.createServo(hardwareMap, "outtakeLatch");
        this.slidePivot = HardwareCreator.createServo(hardwareMap, "outtakeSlidePivot");
        this.outtakePivot = HardwareCreator.createServo(hardwareMap, "outtakePivot");
        this.hangServoRight = HardwareCreator.createServo(hardwareMap, "hangServoRight");
        this.hangServoLeft = HardwareCreator.createServo(hardwareMap, "hangServoLeft");
        this.droneLatch = HardwareCreator.createServo(hardwareMap, "droneLatch");
        this.tilt = HardwareCreator.createServo(hardwareMap, "tilt");
        this.outtakeWireServo = HardwareCreator.createServo(hardwareMap, "outtakeWireServo");

        this.slide = new MotorWithPID(HardwareCreator.createMotor(hardwareMap, "outtake"), outtakePID);
        this.intakeMotor = new MotorWithVelocityPID(HardwareCreator.createMotor(hardwareMap, "intake"), intakeMotorPid);
        this.intakeMotor.setMaxPower(1.0);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        while (!isStopRequested()) {

            // stack intake left servo test
            //
            if (STACK_INTAKE_LEFT_VALUE != 0.0) {
                ((PwmControl) this.stackIntakeServoLeft).setPwmEnable();
                stackIntakeServoLeft.setPosition(STACK_INTAKE_LEFT_VALUE);
            } else {
                ((PwmControl) this.stackIntakeServoLeft).setPwmDisable();
            }

            // stack intake right servo test
            //
            if (STACK_INTAKE_RIGHT_VALUE != 0.0) {
                ((PwmControl) this.stackIntakeServoRight).setPwmEnable();
                stackIntakeServoRight.setPosition(STACK_INTAKE_RIGHT_VALUE);
            } else {
                ((PwmControl) this.stackIntakeServoRight).setPwmDisable();
            }

            // stack intake linkage servo test
            //
            if (STACK_INTAKE_LINKAGE_VALUE != 0.0) {
                ((PwmControl) this.stackIntakeLinkage).setPwmEnable();
                stackIntakeLinkage.setPosition(STACK_INTAKE_LINKAGE_VALUE);
            } else {
                ((PwmControl) this.stackIntakeLinkage).setPwmDisable();
            }

            // outtake latch servo test
            if (OUTTAKE_LATCH_VALUE != 0.0) {
                ((PwmControl) this.outtakeLatch).setPwmEnable();
                outtakeLatch.setPosition(OUTTAKE_LATCH_VALUE);
            } else {
                ((PwmControl) this.outtakeLatch).setPwmDisable();
            }

            // outtake slide pivot servo test
            //
            if (SLIDE_PIVOT_VALUE != 0.0) {
                ((PwmControl) this.slidePivot).setPwmEnable();
                slidePivot.setPosition(SLIDE_PIVOT_VALUE);
            } else {
                ((PwmControl) this.slidePivot).setPwmDisable();
            }

            // outtake Pivot servo test
            //
            if (OUTTAKE_PIVOT_VALUE != 0.0) {
                ((PwmControl) this.outtakePivot).setPwmEnable();
                outtakePivot.setPosition(OUTTAKE_PIVOT_VALUE);
            } else {
                ((PwmControl) this.outtakePivot).setPwmDisable();
            }

            // hang Servo Right servo test
            //
            if (HANG_SERVO_RIGHT_VALUE != 0.0) {
                ((PwmControl) this.hangServoRight).setPwmEnable();
                hangServoRight.setPosition(HANG_SERVO_RIGHT_VALUE);
            } else {
                ((PwmControl) this.hangServoRight).setPwmDisable();
            }

            // stack intake left servo test
            //
            if (HANG_SERVO_LEFT_VALUE != 0.0) {
                ((PwmControl) this.hangServoLeft).setPwmEnable();
                hangServoLeft.setPosition(HANG_SERVO_LEFT_VALUE);
            } else {
                ((PwmControl) this.hangServoLeft).setPwmDisable();
            }

            // drone latch servo test
            //
            if (DRONE_LATCH_VALUE != 0.0) {
                ((PwmControl) this.droneLatch).setPwmEnable();
                droneLatch.setPosition(DRONE_LATCH_VALUE);
            } else {
                ((PwmControl) this.droneLatch).setPwmDisable();
            }

            // drone tilt servo test
            //
            if (DRONE_TILT_VALUE != 0.0) {
                ((PwmControl) this.tilt).setPwmEnable();
                tilt.setPosition(DRONE_TILT_VALUE);
            } else {
                ((PwmControl) this.tilt).setPwmDisable();
            }

            // outtake wire servo test
            //
            if (OUTTAKE_WIRE_SERVO_VALUE != 0.0) {
                ((PwmControl) this.outtakeWireServo).setPwmEnable();
                outtakeWireServo.setPosition(OUTTAKE_WIRE_SERVO_VALUE);
            } else {
                ((PwmControl) this.outtakeWireServo).setPwmDisable();
                telemetry.addData("set outtakeWireServo to ", "null");
            }

            this.slide.setTargetPosition(OUTTAKE_MOTOR_POSITION);

            if(INTAKE_MOTOR_VELOCITY != 0) {
                this.intakeMotor.setTargetVelocity(INTAKE_MOTOR_VELOCITY);
            } else if (INTAKE_MOTOR_POWER != 0.0) {
                //this.intakeMotor.set
            }

            bottomRollerServo.setPower(BOTTOM_ROLLER_CRSERVO_VALUE);

            telemetry.addData("stackIntakeServoLeft: ", "%3.3f", stackIntakeServoLeft.getPosition());
            telemetry.addData("stackIntakeServoRight: ", "%3.3f", stackIntakeServoRight.getPosition());
            telemetry.addData("stackIntakeLinkage: ", "%3.3f", stackIntakeLinkage.getPosition());
            telemetry.addData("outtakeLatch: ", "%3.3f", outtakeLatch.getPosition());
            telemetry.addData("outtakePivot: ", "%3.3f", outtakePivot.getPosition());
            telemetry.addData("slidePivot: ", "%3.3f", slidePivot.getPosition());
            telemetry.addData("hangServoRight: ", "%3.3f", hangServoRight.getPosition());
            telemetry.addData("hangServoLeft: ", "%3.3f", hangServoLeft.getPosition());
            telemetry.addData("droneLatch: ", "%3.3f", droneLatch.getPosition());
            telemetry.addData("tilt: ", "%3.3f", tilt.getPosition());
            telemetry.addData("outtakeWireServo: ", "%3.3f", outtakeWireServo.getPosition());

            telemetry.addData("bottomRollerServo: ", "%3.3f", bottomRollerServo.getPower());

            telemetry.update();

        }
    }
}
