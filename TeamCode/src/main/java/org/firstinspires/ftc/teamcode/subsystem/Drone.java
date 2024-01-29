package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.hardware.HardwareCreator;
import org.firstinspires.ftc.teamcode.utils.software.ActionUtil;

@Config
public class Drone {
    public static double LATCH_SCORED = 0.45;
    public static double LATCH_CLOSED = 0.66;

    // Drone Tilt is Axon Mini Servo
    public static double TILT_INIT = 0.30;
    public static double TILT_LAUNCH = 0.6;

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
                new ActionUtil.ServoPositionAction(tilt, TILT_LAUNCH, "drone_tilt"),
                new SleepAction(0.5),
                new ActionUtil.ServoPositionAction(latch, LATCH_SCORED, "drone_latch"),
                new SleepAction(0.5),
                new ActionUtil.ServoPositionAction(tilt, TILT_INIT, "drone_tilt"),
                new ActionUtil.ServoPositionAction(latch, LATCH_CLOSED, "drone_latch")
        );
    }

    public Action initDrone() {
        return new ParallelAction(
                new ActionUtil.ServoPositionAction(tilt, TILT_INIT, "drone_tilt"),
                new ActionUtil.ServoPositionAction(latch, LATCH_CLOSED, "drone_latch")
        );
    }
}
