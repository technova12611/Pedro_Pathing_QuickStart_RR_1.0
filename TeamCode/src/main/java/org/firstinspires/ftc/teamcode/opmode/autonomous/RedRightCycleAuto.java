package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pipeline.AlliancePosition;

@Config
@Autonomous(name = "RED Near (RIGHT) Cycle Auto (2+4)", group = "RED Auto", preselectTeleOp = "Manual Drive")
public class RedRightCycleAuto extends NearCycleAutoBase {

    @Override
    protected void onInit() {
        super.onInit();
        // 0 = left, 1 = middle, 2 = right
        this.start = RedRightAuto.red_start;
        this.backdrop = RedRightAuto.red_backdrop;
        this.spike = RedRightAuto.red_spike;

        this.cycleStart = new Pose2d[]{
                new Pose2d(spike[2].position.x, -11.0, Math.toRadians(180)),
                new Pose2d(spike[2].position.x, -11.0, Math.toRadians(180)),
                new Pose2d(spike[2].position.x, -11.0, Math.toRadians(180))
        };

        this.stackAlignment = new Pose2d(-48.0, -11.0, Math.toRadians(180.85));
        this.stackIntake1 = new Pose2d(-55.8, -12.0, Math.toRadians(180));
        this.stackIntake2 = new Pose2d(-56.5, -11.0, Math.toRadians(180));
        this.safeTrussPassStop = new Pose2d(-49.0, -11.0, Math.toRadians(180));
        this.backdropAlignment = new Pose2d(38.0, -11.0, Math.toRadians(180));

        cycleScore = new Pose2d[] {
                new Pose2d(49.2, -35.0, Math.toRadians(180)),
                new Pose2d(48.8, -31.0, Math.toRadians(180)),
                new Pose2d(48.8, -31.0, Math.toRadians(180))
        };

        this.parking = new Pose2d(44.1, -20.0, Math.toRadians(180));
    }

    protected AlliancePosition getAlliance() {
        return AlliancePosition.RED;
    }

    @Override
    protected void printDescription() {
        telemetry.addData("Description", "RED Near (Right-Side) Cycle Auto");
    }
}
