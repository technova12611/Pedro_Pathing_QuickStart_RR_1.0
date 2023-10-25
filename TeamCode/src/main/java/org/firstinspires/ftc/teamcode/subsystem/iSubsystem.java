package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.hardware.HardwareCreator;

public abstract class iSubsystem {
    public iSubsystem() {
    }

    public void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            // ignore
        }
    }

    public DcMotorEx initMotor(HardwareMap hardwareMap, String motorName) {
        DcMotorEx motor = HardwareCreator.createMotor(hardwareMap, motorName);
        motor.setPower(0);
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        return motor;
    }
}
