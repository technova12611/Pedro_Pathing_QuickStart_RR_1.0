package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.pipeline.AlliancePosition;

@Config
@Autonomous(name = "RED Far (LEFT) Cycle Auto (2+3)", group = "RED Auto FAR", preselectTeleOp = "Manual Drive")
public class RedLeftCycleAuto extends RedLeftAuto {
    // 0 = left, 1 = middle, 2 = right

    @Override
    protected void onInit() {
        super.onInit();

        this.cycleStart = new Pose2d[]{
                new Pose2d(RedRightAuto.red_spike[2].position.x, -11.0, Math.toRadians(180)),
                new Pose2d(RedRightAuto.red_spike[2].position.x, -11.0, Math.toRadians(180)),
                new Pose2d(RedRightAuto.red_spike[2].position.x, -11.0, Math.toRadians(180))
        };

        this.stackAlignment = new Pose2d(-46.0, -11.0, Math.toRadians(180.85));
        this.stackIntake1 = new Pose2d(-56.5, -12.0, Math.toRadians(180));
        this.safeTrussPassStop = new Pose2d(-51.0, -11.0, Math.toRadians(180));

        this.backdropAlignmentCycle = new Pose2d[]{
                new Pose2d(38.0, -11.0, Math.toRadians(180)),
                new Pose2d(38.0, -11.0, Math.toRadians(180)),
                new Pose2d(38.0, -11.0, Math.toRadians(180))
        };
    }

    @Override
    protected AlliancePosition getAlliance() {
        return AlliancePosition.RED;
    }

    @Override
    protected void printDescription() {
        telemetry.addData("Description", "RED Far (Left-Side) Cycle Auto");
    }

    protected boolean doCycle() {
        return true;
    }

}
