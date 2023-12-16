package org.firstinspires.ftc.teamcode.subsystem;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.utils.software.ActionUtil;
import org.firstinspires.ftc.teamcode.utils.hardware.HardwareCreator;

@Config
public class Hang {
    public static int HANG_POSITION_MAX = 5000;
    public static int HANG_POSITION = 1250;
    public static int HANG_INREMENTAL_CHANGE_POSITION = 300;

    public static int DROPDOWN_AFTER_HANG_POSITION = 500;

    public static double HOOK_BLOCKER_BLOCK = 0.81;
    public static double  HOOK_BLOCKER_UNBLOCK = 0.6;

    final DcMotorEx hangMotor;
    final Servo hangHookBlocker;

    public Hang(HardwareMap hardwareMap) {

        this.hangMotor = HardwareCreator.createMotor(hardwareMap, "hangMotor");
        this.hangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.hangMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.hangHookBlocker = HardwareCreator.createServo(hardwareMap, "hangHookBlocker");
    }

    public void initialize() {

        this.hangMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.hangHookBlocker.setPosition(HOOK_BLOCKER_BLOCK);
    }

    public Action hang() {
        return new ActionUtil.DcMotorExRTPAction(hangMotor, HANG_POSITION, 0.95);
    }

    public Action hangSlowly() {
        int position = this.hangMotor.getCurrentPosition() + HANG_INREMENTAL_CHANGE_POSITION;
        return new ActionUtil.DcMotorExRTPAction(hangMotor, Range.clip(position, 0, HANG_POSITION_MAX), 0.9);
    }

    public String getCurrentPosition () {
        return "Hang motor position: " +  this.hangMotor.getCurrentPosition();
    }

    public Action unblockHook() {
        return new ActionUtil.ServoPositionAction(hangHookBlocker, HOOK_BLOCKER_UNBLOCK, "hangHookBlocker");
    }

    public void dropdownFromHang() {
        if (hangMotor.getCurrentPosition() > (DROPDOWN_AFTER_HANG_POSITION + 200)) {
            hangMotor.setTargetPosition(DROPDOWN_AFTER_HANG_POSITION);
            hangMotor.setPower(0.5);
            hangMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public boolean isMotorBusy() {
        return this.hangMotor.isBusy();
    }

}
