package org.firstinspires.ftc.teamcode.subsystem;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.software.ActionUtil;
import org.firstinspires.ftc.teamcode.utils.hardware.HardwareCreator;

@Config

public class Hang {

    public static int HANG_HOOK_UP_POSITION = -800;

    public static int HANG_HOOK_DOWN_POSITION = 0;

    public static int HANG_POSITION = 700;


    final DcMotorEx hangMotor;

    public Hang(HardwareMap hardwareMap) {

        this.hangMotor = HardwareCreator.createMotor(hardwareMap, "hangMotor");
        this.hangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.hangMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void initialize() {
        this.hangMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public Action hookUp() {
        return new ActionUtil.DcMotorExRTPAction(hangMotor, HANG_HOOK_UP_POSITION, 0.3);
    }

    public Action hookDown() {
        return new ActionUtil.DcMotorExRTPAction(hangMotor, HANG_HOOK_DOWN_POSITION, 0.3);
    }

    public Action hang() {
        return new ActionUtil.DcMotorExRTPAction(hangMotor, HANG_POSITION, 0.8);
    }

    public String getCurrentPosition () {
        String message = "Hang motor position: " +  this.hangMotor.getCurrentPosition();
        //Log.d("HangMotor", message);
        return message;
    }

}
