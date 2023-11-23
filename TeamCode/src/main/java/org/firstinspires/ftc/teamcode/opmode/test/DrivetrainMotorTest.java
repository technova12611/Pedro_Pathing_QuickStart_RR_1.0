package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.utils.hardware.HardwareCreator;

@Config
//@Disabled
@TeleOp(group = "Test")
public class DrivetrainMotorTest extends LinearOpMode {
    public DcMotorEx leftFront, leftBack, rightBack, rightFront;
    public static double runForTime = 5.0; // 5 seconds
    public static double motorPower = 0.8; // 5 seconds

    @Override
    public void runOpMode() throws InterruptedException {

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        leftFront = HardwareCreator.createMotor(hardwareMap, "leftFront");
        leftBack = HardwareCreator.createMotor(hardwareMap, "leftBack");
        rightBack = HardwareCreator.createMotor(hardwareMap, "rightBack");
        rightFront = HardwareCreator.createMotor(hardwareMap, "rightFront");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        while (opModeInInit()) {
            telemetry.addData("Testing 4 motors and compare the encode value: ", "%3.2f", runForTime);
            telemetry.update();
            idle();
        }

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightBack.setPower(motorPower);
        rightFront.setPower(motorPower);
        leftFront.setPower(motorPower);
        leftBack.setPower(motorPower);

        long startTime = System.currentTimeMillis();

        while(!isStopRequested() && (System.currentTimeMillis() - startTime) < runForTime * 1000) {
            telemetry.addData("right_back", rightBack.getCurrentPosition());
            telemetry.addData("right_front", rightFront.getCurrentPosition());
            telemetry.addData("left_front", leftFront.getCurrentPosition());
            telemetry.addData("left_back", leftBack.getCurrentPosition());

            telemetry.addData("ElapsedTime: ", System.currentTimeMillis() - startTime);

            telemetry.update();
            idle();
        }
    }

}
