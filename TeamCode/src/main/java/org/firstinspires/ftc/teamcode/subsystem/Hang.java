package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.software.ActionUtil;
import org.firstinspires.ftc.teamcode.utils.hardware.HardwareCreator;
import org.firstinspires.ftc.teamcode.utils.hardware.MotorWithPID;
import org.firstinspires.ftc.teamcode.utils.control.PIDCoefficients;

@Config
public class Hang {
    public static PIDCoefficients hangPID = new PIDCoefficients(0.0025, 0, 0.0004);
    public static int HANG_EXTENDED = 9300;
    final MotorWithPID hang;

    public Hang(HardwareMap hardwareMap) {
        this.hang = new MotorWithPID(HardwareCreator.createMotor(hardwareMap, "hang"), hangPID);
        this.hang.setMaxPower(1.0);
    }

    public Action extendHang() {
        return this.hang.setTargetPositionAction(HANG_EXTENDED);
    }
    public Action retractHang() {
        return this.hang.setTargetPositionAction(0);
    }
    public void update() {
        this.hang.update();
    }
}
