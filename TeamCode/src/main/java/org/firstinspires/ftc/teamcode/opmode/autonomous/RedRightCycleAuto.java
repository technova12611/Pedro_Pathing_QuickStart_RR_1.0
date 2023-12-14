package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.pipeline.AlliancePosition;
import org.firstinspires.ftc.teamcode.pipeline.FieldPosition;

@Config
@Autonomous(name = "RED Right Cycle Auto (2+4)", group = "RED Auto", preselectTeleOp = "Manual Drive")
public class RedRightCycleAuto extends NearCycleAutoBase {

    @Override
    protected void onInit() {
        // 0 = left, 1 = middle, 2 = right
        this.start = RedRightAuto.red_start;
        this.backdrop = RedRightAuto.red_backdrop;
        this.spike = RedRightAuto.red_spike;

        this.cycleStart = new Pose2d[]{
                new Pose2d(spike[0].position.x, -12.0, Math.toRadians(180)),
                new Pose2d(spike[1].position.x, -12.0, Math.toRadians(180)),
                new Pose2d(spike[2].position.x, -12.0, Math.toRadians(180))
        };

        this.stackAlignment = new Pose2d(-50.0, -11.0, Math.toRadians(180));
        this.stackIntake = new Pose2d(-54.75, -11.75, Math.toRadians(180));
        this.safeTrussPassStop = new Pose2d(-49.0, -11.0, Math.toRadians(180));
        this.backdropAlignment = new Pose2d(45.0, -11.0, Math.toRadians(180));

        cycleScore = new Pose2d[] {
                new Pose2d(49.0, -39.0, Math.toRadians(180)),
                new Pose2d(49.0, -32.0, Math.toRadians(180)),
                new Pose2d(49.0, -32.0, Math.toRadians(180))
        };

        this.parking = new Pose2d(45.0, -20.0, Math.toRadians(180));
    }

    protected AlliancePosition getAlliance() {
        return AlliancePosition.RED;
    }

    @Override
    protected void printDescription() {
        telemetry.addData("Description", "RED Right Cycle Auto");
    }
}
