package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.subsystem.Drone;
import org.firstinspires.ftc.teamcode.subsystem.Hang;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;
import org.firstinspires.ftc.teamcode.utils.control.PIDCoefficients;
import org.firstinspires.ftc.teamcode.utils.hardware.GamePadController;
import org.firstinspires.ftc.teamcode.utils.hardware.HardwareCreator;
import org.firstinspires.ftc.teamcode.utils.hardware.MotorWithPID;
import org.firstinspires.ftc.teamcode.utils.hardware.MotorWithVelocityPID;

@Config
@TeleOp(group = "Test")
public class ServoMotorTest extends LinearOpMode {
    public static PIDCoefficients outtakePID = new PIDCoefficients(0.005, 0, 0.0001);
    Servo stackIntakeServoLeft;
    Servo stackIntakeServoRight;
    Servo stackIntakeLinkage;
    CRServo bottomRollerServo;
    Servo outtakeLatch;
    Servo outtakePivot;
    Servo slidePivot;
    Servo droneLatch;
    Servo droneTilt;
    Servo outtakeWireServo;

    Servo outtakeFixerServo;

    MotorWithPID slide;
    DcMotorEx intakeMotor;
    DcMotorEx parOdometry1;

    DcMotorEx parOdometry2;

    DcMotorEx perpOdometry2;

    DcMotorEx hangMotor;

    AnalogInput slidePivotVoltage;
    AnalogInput outtakePivotVoltage;

    public static double STACK_INTAKE_LEFT_VALUE = Intake.STACK_INTAKE_LEFT_INIT;
    public static double STACK_INTAKE_RIGHT_VALUE = Intake.STACK_INTAKE_RIGHT_INIT;
    public static double STACK_INTAKE_LINKAGE_VALUE = Intake.STACK_INTAKE_LINKAGE_DOWN;
    public static double BOTTOM_ROLLER_CRSERVO_VALUE = 0.0;
    public static double OUTTAKE_LATCH_VALUE = Outtake.LATCH_CLOSED;
    public static double OUTTAKE_PIVOT_VALUE = Outtake.OUTTAKE_PIVOT_INIT;
    public static double SLIDE_PIVOT_VALUE = Outtake.SLIDE_PIVOT_INIT;
    public static double DRONE_LATCH_VALUE = Drone.LATCH_CLOSED;
    public static double DRONE_TILT_VALUE = Drone.TILT_INIT;
    public static double OUTTAKE_WIRE_SERVO_VALUE = Outtake.OUTTAKE_WIRE_MIDDLE;

    public static int OUTTAKE_MOTOR_POSITION = 0;

    public static int OUTTAKE_MOTOR_INTERNAL_PIDF_POSITION = 0;

    public static int INTAKE_MOTOR_VELOCITY = 0;

    public static double INTAKE_MOTOR_POWER = 0;

    public static double OUTTAKE_FIXER_SERVO_POSITION = Outtake.OUTTAKE_FIXER_INIT;

    public IMU imu;

    public static int outtakeMode = 0;

    public static int stackIntakeMode = 0;

    public static int scoringMode = 0;

    DigitalChannel beamBreaker1;

    DigitalChannel beamBreaker2;

    Rev2mDistanceSensor stackDistance;
    Rev2mDistanceSensor stackDistance2;
    Rev2mDistanceSensor backdropDistance;

    boolean prevBeamBreakerState = true;
    boolean curBeamBreakerState = true;

    public static int totalPixelCount = 0;
    int pixelsCount = 0;

    public static int HANG_MOTOR_POSITION = 0;
    public static double HANG_MOTOR_POWER = 0.0;

    OverflowEncoder par, par1;
    OverflowEncoder perp;

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
        this.droneLatch = HardwareCreator.createServo(hardwareMap, "droneLatch");
        this.droneTilt = HardwareCreator.createServo(hardwareMap, "droneTilt");
        this.outtakeWireServo = HardwareCreator.createServo(hardwareMap, "outtakeWireServo");
        this.outtakeFixerServo = HardwareCreator.createServo(hardwareMap, "outtakeFixerServo");

//        this.slide = new MotorWithPID(HardwareCreator.createMotor(hardwareMap, "outtake"), outtakePID);

        this.slide = new MotorWithPID(HardwareCreator.createMotor(hardwareMap, "outtake"), outtakePID);
        this.slide.setDirection(DcMotorSimple.Direction.REVERSE);
        this.slide.setMaxPower(0.95);
        this.slide.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.intakeMotor = HardwareCreator.createMotor(hardwareMap, "intake_for_perp");
        this.intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        this.parOdometry1 = HardwareCreator.createMotor(hardwareMap, "par");
        this.parOdometry1.setDirection(DcMotorSimple.Direction.REVERSE);
        this.perpOdometry2 =HardwareCreator.createMotor(hardwareMap, "intake_for_perp");
        //this.perpOdometry2.setDirection(DcMotorSimple.Direction.REVERSE);

        this.parOdometry2 = HardwareCreator.createMotor(hardwareMap, "rightBack");

        this.parOdometry1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.parOdometry2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.perpOdometry2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.par = new OverflowEncoder(new RawEncoder(HardwareCreator.createMotor(hardwareMap, "par")));
        this.par.setDirection(DcMotorSimple.Direction.REVERSE);

        this.par1 = new OverflowEncoder(new RawEncoder(HardwareCreator.createMotor(hardwareMap, "rightBack")));

        this.perp = new OverflowEncoder(new RawEncoder(HardwareCreator.createMotor(hardwareMap, "intake_for_perp")));

        this.hangMotor = HardwareCreator.createMotor(hardwareMap, "hangMotor");
        this.hangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.hangMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.hangMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.slidePivotVoltage = hardwareMap.get(AnalogInput.class, "slidePivotVoltage");
        this.outtakePivotVoltage = hardwareMap.get(AnalogInput.class, "outtakePivotVoltage");

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.DOWN));
        imu.initialize(parameters);
        imu.resetYaw();

        beamBreaker1 = hardwareMap.get(DigitalChannel.class, "beamBreaker1");
        beamBreaker2 = hardwareMap.get(DigitalChannel.class, "beamBreaker2");

        stackDistance = (Rev2mDistanceSensor) hardwareMap.get(DistanceSensor.class, "stackDistance");
        stackDistance2 = (Rev2mDistanceSensor) hardwareMap.get(DistanceSensor.class, "stackDistance2");
        backdropDistance = (Rev2mDistanceSensor) hardwareMap.get(DistanceSensor.class, "backdropDistance");

        this.stackIntakeLinkage.setPosition(Intake.STACK_INTAKE_LINKAGE_DOWN);

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
                this.intakeMotor.setPower(0.85);
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
                this.intakeMotor.setPower(0.85);
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
                    Thread.sleep(400);
                }

                this.slide.getMotor().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                if(OUTTAKE_MOTOR_POSITION != 0) {
                    this.slide.setTargetPosition(OUTTAKE_MOTOR_POSITION);
                }
                else {
                    this.slide.setTargetPosition(Outtake.OUTTAKE_SLIDE_FAR_LOW);
                }

            }
            // scoring 1
            else if(outtakeMode == 4) {
                slidePivot.setPosition(Outtake.SLIDE_PIVOT_DUMP);
                outtakePivot.setPosition(Outtake.OUTTAKE_PIVOT_DUMP_CYCLE);

                this.slide.getMotor().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                if(OUTTAKE_MOTOR_POSITION != 0) {
                    this.slide.setTargetPosition(OUTTAKE_MOTOR_POSITION);
                }

//                Thread.sleep(250);
//                outtakeLatch.setPosition(Outtake.LATCH_SCORE_1);
//
//                Thread.sleep(500);
//                outtakeLatch.setPosition(Outtake.LATCH_SCORE_2);
//                Thread.sleep(500);
//
//                pixelsCount=0;
//
//                outtakeMode = 2;

            }
            // scoring 2
            else if(outtakeMode == 5 ) {
                slidePivot.setPosition(Outtake.SLIDE_PIVOT_DUMP);
                outtakePivot.setPosition(Outtake.OUTTAKE_PIVOT_DUMP_CYCLE);
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
            else if(outtakeMode == 7 ) {

                slidePivot.setPosition(Outtake.SLIDE_PIVOT_SLIDING);
                outtakePivot.setPosition(Outtake.OUTTAKE_PIVOT_SLIDING);
                Thread.sleep(600);

                this.slide.getMotor().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                if(OUTTAKE_MOTOR_POSITION != 0) {
                    this.slide.setTargetPosition(OUTTAKE_MOTOR_POSITION);
                }
                else {
                    this.slide.setTargetPosition(Outtake.OUTTAKE_SLIDE_FAR_LOW);
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

            // drone latch servo test
            //
            droneLatch.setPosition(DRONE_LATCH_VALUE);

            // drone tilt servo test
            //
            droneTilt.setPosition(DRONE_TILT_VALUE);

            // outtake wire servo test
            //
            outtakeWireServo.setPosition(OUTTAKE_WIRE_SERVO_VALUE);

            outtakeFixerServo.setPosition(OUTTAKE_FIXER_SERVO_POSITION);

            this.intakeMotor.setPower(INTAKE_MOTOR_POWER);

            curBeamBreakerState = beamBreaker1.getState();
            if(curBeamBreakerState && !prevBeamBreakerState) {
                pixelsCount++;
                totalPixelCount++;
            }

            if(curBeamBreakerState != prevBeamBreakerState) {
                prevBeamBreakerState = curBeamBreakerState;
            }

            if(HANG_MOTOR_POSITION != 0) {
                this.hangMotor.setTargetPosition(HANG_MOTOR_POSITION);
                this.hangMotor.setPower(Range.clip(HANG_MOTOR_POWER,-1.0, 1.0));
                this.hangMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            telemetry.addData("** Stack distance sensor left: ", String.format("%3.2f",stackDistance.getDistance(DistanceUnit.INCH)));
            telemetry.addData("** Stack distance sensor right: ", String.format("%3.2f",stackDistance2.getDistance(DistanceUnit.INCH)));
            telemetry.addData("**    Backdrop distance sensor: ", String.format("%3.2f", backdropDistance.getDistance(DistanceUnit.INCH)));

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
            telemetry.addData("droneLatch: ", "%3.2f", droneLatch.getPosition());
            telemetry.addData("tilt: ", "%3.2f", droneTilt.getPosition());
            telemetry.addData("outtakeWireServo: ", "%3.2f", outtakeWireServo.getPosition());
            telemetry.addData("outtakeFixerServo: ", "%3.2f", outtakeFixerServo.getPosition());

            telemetry.addData("bottomRollerServo: ", "%3.2f", bottomRollerServo.getPower());

            telemetry.addData("outtake motor position: ", slide.getCurrentPosition());
            telemetry.addData("outtake motor internal motor position: ", slide.getMotor().getCurrentPosition());
            telemetry.addData("intake motor power: ", "%3.2f", intakeMotor.getPower());
            telemetry.addData("outtake motor mode: ", slide.getRunMode());
            telemetry.addData("Beam breaker state: ", curBeamBreakerState);
            telemetry.addData("pixelCount: ", pixelsCount);
            telemetry.addData("totalCount: ", totalPixelCount);
            telemetry.addData("Par0 encoder: ", parOdometry1.getCurrentPosition());
            telemetry.addData("Par1 encoder: ", parOdometry2.getCurrentPosition());
            telemetry.addData("Perp encoder value: ", perpOdometry2.getCurrentPosition());

            telemetry.addData("slidePivotVoltage: ", "%3.2f", slidePivotVoltage.getVoltage());
            telemetry.addData("outtakePivotVoltage: ", "%3.2f", outtakePivotVoltage.getVoltage());

            telemetry.addData("hangMotor current Position: ", hangMotor.getCurrentPosition());
            telemetry.addData("hangMotor target Position: ", hangMotor.getTargetPosition());

            telemetry.addData("par0 overflow encoder value: ", par.encoder.getPositionAndVelocity().position);
            telemetry.addData("perp overflow encoder value: ", perp.encoder.getPositionAndVelocity().position);
            telemetry.addData("par1 overflow encoder value: ", par1.encoder.getPositionAndVelocity().position);

            telemetry.addData("** Stack distance sensor left: ", String.format("%3.2f",stackDistance.getDistance(DistanceUnit.INCH)));
            telemetry.addData("** Stack distance sensor right: ", String.format("%3.2f",stackDistance2.getDistance(DistanceUnit.INCH)));
            telemetry.addData("**    Backdrop distance sensor: ", String.format("%3.2f", backdropDistance.getDistance(DistanceUnit.INCH)));

            telemetry.update();

        }
    }
}
