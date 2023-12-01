package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.software.ActionUtil;
import org.firstinspires.ftc.teamcode.utils.hardware.HardwareCreator;

@Config

public class Hang {
    public static double HANG_UP_RIGHT = 0.58;
    public static double HANG_DOWN_RIGHT = 0.24;
    public static double HANG_UP_LEFT = 0.6;
    public static double HANG_DOWN_LEFT = 0.26;


    final Servo hangServoRight;
    final Servo hangServoLeft;

    public Hang(HardwareMap hardwareMap) {
        this.hangServoRight = HardwareCreator.createServo(hardwareMap, "hangServoRight");
        this.hangServoLeft = HardwareCreator.createServo(hardwareMap, "hangServoLeft");
    }

    public void initialize() {
        hangServoRight.setPosition(HANG_DOWN_RIGHT);
        hangServoLeft.setPosition(HANG_DOWN_LEFT);
    }

    public Action armsUp() {
        return new ParallelAction(
                new ActionUtil.ServoPositionAction(hangServoRight, HANG_UP_RIGHT),
                new ActionUtil.ServoPositionAction(hangServoLeft, HANG_UP_LEFT)
        );
    }

    public Action armsDown() {
        return new ParallelAction(
                new ActionUtil.ServoPositionAction(hangServoRight, HANG_DOWN_RIGHT),
                new ActionUtil.ServoPositionAction(hangServoLeft, HANG_DOWN_RIGHT)
        );
    }
}
