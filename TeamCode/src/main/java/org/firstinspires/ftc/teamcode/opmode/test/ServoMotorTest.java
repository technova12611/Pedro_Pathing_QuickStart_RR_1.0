package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.subsystem.Hang;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;
import org.firstinspires.ftc.teamcode.utils.control.PIDCoefficients;
import org.firstinspires.ftc.teamcode.utils.hardware.GamePadController;
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
    Servo droneTilt;
    Servo outtakeWireServo;

    MotorWithPID slide;
    MotorWithVelocityPID intakeMotor;
    DcMotorEx parOdometry1;
    DcMotorEx perpOdometry2;

    AnalogInput slidePivotVoltage;
    AnalogInput outtakePivotVoltage;

    public static double STACK_INTAKE_LEFT_VALUE = 0.05;
    public static double STACK_INTAKE_RIGHT_VALUE = 0.05;
    public static double STACK_INTAKE_LINKAGE_VALUE = Intake.STACK_INTAKE_LINKAGE_UP;
    public static double BOTTOM_ROLLER_CRSERVO_VALUE = 0.0;
    public static double OUTTAKE_LATCH_VALUE = Outtake.LATCH_CLOSED;
    public static double OUTTAKE_PIVOT_VALUE = Outtake.OUTTAKE_PIVOT_INIT;
    public static double SLIDE_PIVOT_VALUE = Outtake.SLIDE_PIVOT_INIT;
    public static double HANG_SERVO_RIGHT_VALUE = Hang.HANG_DOWN_RIGHT;
    public static double HANG_SERVO_LEFT_VALUE = Hang.HANG_DOWN_LEFT;
    public static double DRONE_LATCH_VALUE = 0.4;
    public static double DRONE_TILT_VALUE = 0.3;
    public static double OUTTAKE_WIRE_SERVO_VALUE = Outtake.OUTTAKE_WIRE_MIDDLE;

    public static int OUTTAKE_MOTOR_POSITION = 0;

    public static int OUTTAKE_MOTOR_INTERNAL_PIDF_POSITION = 0;

    public static int OUTTAKE_MOTOR_POWER = 0;
    public static int INTAKE_MOTOR_VELOCITY = 0;

    public static double INTAKE_MOTOR_POWER = 0;

    double prevIntakeMotorPower = 0.0;

    public IMU imu;

    public static int outtakeMode = 0;

    public static int stackIntakeMode = 0;

    public static int scoringMode = 0;

    DigitalChannel beamBreaker1;

    DigitalChannel beamBreaker2;

    boolean prevBeamBreakerState = true;
    boolean curBeamBreakerState = true;

    public static int totalPixelCount = 0;
    int pixelsCount = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        GamePadController g1 = new GamePadController(gamepad1);
        this.bottomRollerServo = HardwareCreator.createCRServo(hardwareMap, "bottomRollerServo", HardwareCreator.ServoType.GOBILDA);

        this.stackIntakeServoLeft = HardwareCreator.createServo(hardwareMap, "stackIntakeServoLeft");
        this.stackIntakeServoRight = HardwareCreator.createServo(hardwareMap, "stackIntakeServoRight");
        this.stackIntakeLinkage = HardwareCreator.createServo(hardwareMap, "stackIntakeLinkage");
        this.outtakeLatch = HardwareCreator.createServo(hardwareMap, "outtakeLatch");
        this.slidePivot = HardwareCreator.createServo(hardwareMap, "outtakeSlidePivot");
        this.outtakePivot = HardwareCreator.createServo(hardwareMap, "outtakePivot");
        this.hangServoRight = HardwareCreator.createServo(hardwareMap, "hangServoRight");
        this.hangServoLeft = HardwareCreator.createServo(hardwareMap, "hangServoLeft");
        this.droneLatch = HardwareCreator.createServo(hardwareMap, "droneLatch");
        this.droneTilt = HardwareCreator.createServo(hardwareMap, "droneTilt");
        this.outtakeWireServo = HardwareCreator.createServo(hardwareMap, "outtakeWireServo");

//        this.slide = new MotorWithPID(HardwareCreator.createMotor(hardwareMap, "outtake"), outtakePID);

        this.slide = new MotorWithPID(HardwareCreator.createMotor(hardwareMap, "outtake"), outtakePID);
        this.slide.setDirection(DcMotorSimple.Direction.REVERSE);
        this.slide.setMaxPower(0.95);
        this.slide.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.intakeMotor = new MotorWithVelocityPID(HardwareCreator.createMotor(hardwareMap, "intake"), intakeMotorPid);
        this.intakeMotor.setMaxPower(0.95);
        this.intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        this.parOdometry1 = HardwareCreator.createMotor(hardwareMap, "par");
        this.parOdometry1.setDirection(DcMotorSimple.Direction.REVERSE);
        this.perpOdometry2 =HardwareCreator.createMotor(hardwareMap, "perp");
        this.perpOdometry2.setDirection(DcMotorSimple.Direction.REVERSE);

        this.parOdometry1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.perpOdometry2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.slidePivotVoltage = hardwareMap.get(AnalogInput.class, "slidePivotVoltage");
        this.outtakePivotVoltage = hardwareMap.get(AnalogInput.class, "outtakePivotVoltage");

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        beamBreaker1 = hardwareMap.get(DigitalChannel.class, "beamBreaker1");
        beamBreaker2 = hardwareMap.get(DigitalChannel.class, "beamBreaker2");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (!isStopRequested()) {

            g1.update();

            if(g1.xOnce()) {
                outtakeMode = 2;
            } else if(g1.yOnce()) {
                outtakeMode = 5;
            } else if (g1.dpadUpOnce()) {
                outtakeMode = 3;
            } else if(g1.dpadDownOnce()) {
                outtakeMode = 6;
            } else if(g1.aOnce()) {
                outtakeMode = 1;
            } else if(g1.bOnce()) {
                outtakeMode = 0;
            }

            if(stackIntakeMode == 1) {
                bottomRollerServo.setPower(0.6);
                this.intakeMotor.getMotor().setPower(0.85);
                if(stackIntakeLinkage.getPosition() > Intake.STACK_INTAKE_LINKAGE_DOWN + 0.1) {
                    stackIntakeLinkage.setPosition(Intake.STACK_INTAKE_LINKAGE_DOWN);
                    Thread.sleep(250);
                }
                if(stackIntakeServoRight.getPosition() < Intake.STACK_INTAKE_RIGHT_1ST_PIXEL - 0.25) {
                    stackIntakeServoRight.setPosition(Intake.STACK_INTAKE_RIGHT_1ST_PIXEL);
                    Thread.sleep(200);
                }
                if(stackIntakeServoLeft.getPosition() < Intake.STACK_INTAKE_LEFT_1ST_PIXEL - 0.25) {
                    stackIntakeServoLeft.setPosition(Intake.STACK_INTAKE_LEFT_1ST_PIXEL);
                    Thread.sleep(500);
                    stackIntakeMode = 2;
                }
            }
            else if(stackIntakeMode == 2) {
                bottomRollerServo.setPower(0.6);
                this.intakeMotor.getMotor().setPower(0.85);
                if(stackIntakeLinkage.getPosition() > Intake.STACK_INTAKE_LINKAGE_DOWN + 0.1) {
                    stackIntakeLinkage.setPosition(Intake.STACK_INTAKE_LINKAGE_DOWN);
                    Thread.sleep(250);
                }

                if (stackIntakeServoRight.getPosition() < Intake.STACK_INTAKE_RIGHT_2nd_PIXEL - 0.1) {
                    stackIntakeServoRight.setPosition(Intake.STACK_INTAKE_RIGHT_2nd_PIXEL);
                    Thread.sleep(200);
                }

                if (stackIntakeServoLeft.getPosition() < Intake.STACK_INTAKE_LEFT_2nd_PIXEL - 0.1) {
                    stackIntakeServoLeft.setPosition(Intake.STACK_INTAKE_LEFT_2nd_PIXEL);
                    Thread.sleep(500);
                }
            }
            else {

                // stack intake left servo test
                //
                stackIntakeServoLeft.setPosition(STACK_INTAKE_LEFT_VALUE);

                // stack intake right servo test
                //

                stackIntakeServoRight.setPosition(STACK_INTAKE_RIGHT_VALUE);

                // stack intake linkage servo test
                //
                stackIntakeLinkage.setPosition(STACK_INTAKE_LINKAGE_VALUE);
            }

            this.slide.update();

            // init
            if(outtakeMode == 1) {
                outtakeLatch.setPosition(Outtake.LATCH_CLOSED);
                slidePivot.setPosition(Outtake.SLIDE_PIVOT_INIT);
                outtakePivot.setPosition(Outtake.OUTTAKE_PIVOT_INIT);
            }
            // prepare sliding
            else if(outtakeMode == 2 ) {

                INTAKE_MOTOR_POWER = 0.0;
                BOTTOM_ROLLER_CRSERVO_VALUE = 0.0;

                outtakeLatch.setPosition(Outtake.LATCH_CLOSED);
                slidePivot.setPosition(Outtake.SLIDE_PIVOT_SLIDING);
                outtakePivot.setPosition(Outtake.OUTTAKE_PIVOT_SLIDING);

                Thread.sleep(500);

            }
            // sliding up
            else if(outtakeMode == 3 ) {

                if(slidePivot.getPosition() < Outtake.SLIDE_PIVOT_SLIDING) {
                    slidePivot.setPosition(Outtake.SLIDE_PIVOT_SLIDING);
                    outtakePivot.setPosition(Outtake.OUTTAKE_PIVOT_SLIDING);
                    Thread.sleep(200);
                }

                this.slide.getMotor().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                if(OUTTAKE_MOTOR_POSITION != 0) {
                    this.slide.setTargetPosition(OUTTAKE_MOTOR_POSITION);
                }
                else {
                    this.slide.setTargetPosition(Outtake.OUTTAKE_SLIDE_MID);
                }

//                if(!this.slide.isBusy()) {
//                    outtakeMode = (scoringMode==0?4:5);
//                }
            }
            // scoring 1
            else if(outtakeMode == 4) {
                slidePivot.setPosition(Outtake.SLIDE_PIVOT_DUMP);
                outtakePivot.setPosition(Outtake.OUTTAKE_PIVOT_DUMP);
                Thread.sleep(250);
                outtakeLatch.setPosition(Outtake.LATCH_SCORE_1);

                Thread.sleep(500);
                outtakeLatch.setPosition(Outtake.LATCH_SCORE_2);
                Thread.sleep(500);

                pixelsCount=0;

                outtakeMode = 2;

            }
            // scoring 2
            else if(outtakeMode == 5 ) {
                slidePivot.setPosition(Outtake.SLIDE_PIVOT_DUMP);
                outtakePivot.setPosition(Outtake.OUTTAKE_PIVOT_DUMP);
                Thread.sleep(250);
                outtakeLatch.setPosition(Outtake.LATCH_SCORE_2);

                Thread.sleep(500);

                pixelsCount=0;

                outtakeMode = 2;

            }
            // sliding down
            else if(outtakeMode == 6 ) {

                outtakeLatch.setPosition(Outtake.LATCH_CLOSED);
                slidePivot.setPosition(Outtake.SLIDE_PIVOT_SLIDING);
                outtakePivot.setPosition(Outtake.OUTTAKE_PIVOT_SLIDING);

                Thread.sleep(250);

                this.slide.getMotor().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                if(OUTTAKE_MOTOR_POSITION > 0) {
                    this.slide.setTargetPosition(OUTTAKE_MOTOR_POSITION);
                } else {
                    this.slide.setTargetPosition(0);
                }

            }
            else {
                // outtake latch servo test
                outtakeLatch.setPosition(OUTTAKE_LATCH_VALUE);

                // outtake slide pivot servo test
                //
                slidePivot.setPosition(SLIDE_PIVOT_VALUE);

                // outtake Pivot servo test
                //
                outtakePivot.setPosition(OUTTAKE_PIVOT_VALUE);

                if(OUTTAKE_MOTOR_INTERNAL_PIDF_POSITION != 0) {
                    this.slide.getMotor().setTargetPosition(OUTTAKE_MOTOR_INTERNAL_PIDF_POSITION);
                    this.slide.getMotor().setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    this.slide.getMotor().setPower(0.8);
                    this.slide.setMaxPower(0.8);
                } else if(OUTTAKE_MOTOR_POSITION != 0) {
                    this.slide.getMotor().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    this.slide.setTargetPosition(OUTTAKE_MOTOR_POSITION);
                    this.slide.update();
                }
            }

            // hang Servo Right servo test
            //
            hangServoRight.setPosition(HANG_SERVO_RIGHT_VALUE);

            // stack intake left servo test
            //
            hangServoLeft.setPosition(HANG_SERVO_LEFT_VALUE);

            // drone latch servo test
            //
            droneLatch.setPosition(DRONE_LATCH_VALUE);

            // drone tilt servo test
            //
            droneTilt.setPosition(DRONE_TILT_VALUE);

            // outtake wire servo test
            //
            outtakeWireServo.setPosition(OUTTAKE_WIRE_SERVO_VALUE);

            if (INTAKE_MOTOR_VELOCITY != 0) {
                this.intakeMotor.setTargetVelocity(INTAKE_MOTOR_VELOCITY);
                this.intakeMotor.update();
            } else if (INTAKE_MOTOR_POWER != 0.0) {
                this.intakeMotor.getMotor().setPower(INTAKE_MOTOR_POWER);
                prevIntakeMotorPower = INTAKE_MOTOR_POWER;
            } else {
                this.intakeMotor.getMotor().setPower(0.0);
                this.intakeMotor.setTargetVelocity(0);
            }

            curBeamBreakerState = beamBreaker1.getState();
            if(curBeamBreakerState && !prevBeamBreakerState) {
                pixelsCount++;
                totalPixelCount++;
            }

            if(curBeamBreakerState != prevBeamBreakerState) {
                prevBeamBreakerState = curBeamBreakerState;
            }

            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.addData("Yaw (Z) velocity", "%.2f Deg/Sec", angularVelocity.zRotationRate);


            bottomRollerServo.setPower(BOTTOM_ROLLER_CRSERVO_VALUE);

            telemetry.addData("outtake mode: ", outtakeMode);
            telemetry.addData("stackIntakeServoLeft: ", "%3.2f", stackIntakeServoLeft.getPosition());
            telemetry.addData("stackIntakeServoRight: ", "%3.2f", stackIntakeServoRight.getPosition());
            telemetry.addData("stackIntakeLinkage: ", "%3.2f", stackIntakeLinkage.getPosition());
            telemetry.addData("outtakeLatch: ", "%3.2f", outtakeLatch.getPosition());
            telemetry.addData("outtakePivot: ", "%3.2f", outtakePivot.getPosition());
            telemetry.addData("slidePivot: ", "%3.2f", slidePivot.getPosition());
            telemetry.addData("hangServoRight: ", "%3.2f", hangServoRight.getPosition());
            telemetry.addData("hangServoLeft: ", "%3.2f", hangServoLeft.getPosition());
            telemetry.addData("droneLatch: ", "%3.2f", droneLatch.getPosition());
            telemetry.addData("tilt: ", "%3.2f", droneTilt.getPosition());
            telemetry.addData("outtakeWireServo: ", "%3.2f", outtakeWireServo.getPosition());

            telemetry.addData("bottomRollerServo: ", "%3.2f", bottomRollerServo.getPower());

            telemetry.addData("outtake motor position: ", slide.getCurrentPosition());
            telemetry.addData("outtake motor internal motor position: ", slide.getMotor().getCurrentPosition());
            telemetry.addData("intake motor velocity: ", "%3.2f", intakeMotor.getVelocity());
            telemetry.addData("outtake motor mode: ", slide.getRunMode());
            telemetry.addData("Beam breaker state: ", curBeamBreakerState);
            telemetry.addData("pixelCount: ", pixelsCount);
            telemetry.addData("totalCount: ", totalPixelCount);
            telemetry.addData("Par encoder: ", parOdometry1.getCurrentPosition());
            telemetry.addData("Perp encoder value: ", perpOdometry2.getCurrentPosition());

            telemetry.addData("slidePivotVoltage: ", "%3.2f", slidePivotVoltage.getVoltage());
            telemetry.addData("outtakePivotVoltage: ", "%3.2f", outtakePivotVoltage.getVoltage());

            telemetry.update();

        }
    }
}
