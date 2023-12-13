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
    public static int HANG_POSITION = 2500;
    public static int HANG_INREMENTAL_CHANGE_POSITION = 300;

    final DcMotorEx hangMotor;

    public Hang(HardwareMap hardwareMap) {

        this.hangMotor = HardwareCreator.createMotor(hardwareMap, "hangMotor");
        this.hangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.hangMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void initialize() {
        this.hangMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public Action hang() {
        return new ActionUtil.DcMotorExRTPAction(hangMotor, HANG_POSITION, 0.95);
    }

    public Action hangSlowly() {
        int position = this.hangMotor.getCurrentPosition() + HANG_INREMENTAL_CHANGE_POSITION;
        return new ActionUtil.DcMotorExRTPAction(hangMotor, Range.clip(position, 0, HANG_POSITION_MAX), 0.9);
    }

    public String getCurrentPosition () {
        String message = "Hang motor position: " +  this.hangMotor.getCurrentPosition();
        //Log.d("HangMotor", message);
        return message;
    }

}
