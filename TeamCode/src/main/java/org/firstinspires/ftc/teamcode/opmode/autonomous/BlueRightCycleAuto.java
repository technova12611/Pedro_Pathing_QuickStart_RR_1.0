package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pipeline.AlliancePosition;

@Config
@Autonomous(name = "BLUE Far Cycle Auto (2+3)", group = "BLUE Auto", preselectTeleOp = "Manual Drive")
public class BlueRightCycleAuto extends RedLeftAuto {
    // 0 = left, 1 = middle, 2 = right

    @Override
    protected void onInit() {
        super.onInit();

        this.cycleStart = new Pose2d[]{
                new Pose2d(BlueLeftAuto.blue_spike[0].position.x, 11.0, Math.toRadians(-180)),
                new Pose2d(BlueLeftAuto.blue_spike[1].position.x, 11.0, Math.toRadians(-180)),
                new Pose2d(BlueLeftAuto.blue_spike[2].position.x, 11.0, Math.toRadians(-180))
        };

        this.stackAlignment = new Pose2d(-48.0, 11.0, Math.toRadians(-180.85));
        this.stackIntake1 = new Pose2d(-56.8, 11.5, Math.toRadians(-180));
        this.safeTrussPassStop = new Pose2d(-49.0, 11.0, Math.toRadians(-180));
        this.backdropAlignmentCycle = new Pose2d(38.0, 11.0, Math.toRadians(-180));
    }

    @Override
    protected AlliancePosition getAlliance() {
        return AlliancePosition.BLUE;
    }

    @Override
    protected void printDescription() {
        telemetry.addData("Description", "BLUE Far (Left-Side) Cycle Auto");
    }

    protected boolean doCycle() {
        return true;
    }
}

