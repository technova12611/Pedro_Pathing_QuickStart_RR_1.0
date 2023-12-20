package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.hardware.HardwareCreator;
import org.firstinspires.ftc.teamcode.utils.software.ActionUtil;

@Config
public class Drone {
    public static double LATCH_SCORED = 0.55;
    public static double LATCH_CLOSED = 0.32;

    public static double LATCH_LOADING = 0.33;

    // Drone Tilt is Axon Mini Servo
    public static double TILT_INIT = 0.3;
    public static double TILT_LAUNCH = 0.57;

    final Servo latch;
    final Servo tilt;

    public Drone(HardwareMap hardwareMap) {
        this.latch = HardwareCreator.createServo(hardwareMap, "droneLatch");
        this.tilt = HardwareCreator.createServo(hardwareMap, "droneTilt");
    }

    public void initialize() {
        latch.setPosition(LATCH_CLOSED);
        tilt.setPosition(TILT_INIT);
    }

    public Action scoreDrone() {
        return new SequentialAction(
                new ActionUtil.ServoPositionAction(tilt, TILT_LAUNCH),
                new SleepAction(0.25),
                new ActionUtil.ServoPositionAction(latch, LATCH_SCORED),
                new SleepAction(0.75),
                new ActionUtil.ServoPositionAction(tilt, TILT_INIT),
                new ActionUtil.ServoPositionAction(latch, LATCH_CLOSED)
        );
    }
}
