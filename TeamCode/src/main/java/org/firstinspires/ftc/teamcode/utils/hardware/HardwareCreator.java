package org.firstinspires.ftc.teamcode.utils.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.utils.hardware.fake.CRServoFake;
import org.firstinspires.ftc.teamcode.utils.hardware.fake.DcMotorFake;
import org.firstinspires.ftc.teamcode.utils.hardware.fake.ServoFake;

@Config
public class HardwareCreator {
    public static boolean SIMULATE_HARDWARE = false;

    public static DcMotorEx createMotor(HardwareMap hardwareMap, String deviceName) {
        if (SIMULATE_HARDWARE) return new DcMotorFake();
        try {
            return hardwareMap.get(DcMotorEx.class, deviceName);
        } catch (IllegalArgumentException e) { // Could not find device
            RobotLog.addGlobalWarningMessage("Failed to find DcMotorEx '%s'", deviceName);
            return new DcMotorFake();
        }
    }

    public static Servo createServo(HardwareMap hardwareMap, String deviceName) {
        if (SIMULATE_HARDWARE) return new ServoFake();
        try {
            return hardwareMap.get(ServoImplEx.class, deviceName);
        } catch (IllegalArgumentException e) {
            RobotLog.addGlobalWarningMessage("Failed to find Servo '%s'", deviceName);
            return new ServoFake();
        }
    }

    public static CRServo createCRServo(HardwareMap hardwareMap, String deviceName) {
        if (SIMULATE_HARDWARE) return new CRServoFake();
        try {
            return hardwareMap.get(CRServo.class, deviceName);
        } catch (IllegalArgumentException e) {
            RobotLog.addGlobalWarningMessage("Failed to find CRServo '%s'", deviceName);
            return new CRServoFake();
        }
    }

    public static void changeGoBildaServoRange(Servo servo, boolean expanded) {
        PwmControl.PwmRange range = expanded? new PwmControl.PwmRange(500, 2500) : new PwmControl.PwmRange(600, 2400);

        if (servo instanceof ServoFake) {
            ((ServoFake) servo).setPwmRange(range);
        } else if (servo instanceof ServoImplEx) {
            ((ServoImplEx) servo).setPwmRange(range);
        }
    }
}
