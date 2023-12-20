package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.pipeline.AlliancePosition;
import org.firstinspires.ftc.teamcode.pipeline.FieldPosition;

@Config
@Autonomous(name = "RED Far Auto (2+1)", group = "RED Auto", preselectTeleOp = "Manual Drive")
public class RedLeftAuto extends FarAutoBase {
    // 0 = left, 1 = middle, 2 = right

    @Override
    protected void onInit() {
        this.start = new Pose2d(-40.0, -62.0, Math.toRadians(90));
        this.spike = new Pose2d[] {
                new Pose2d(-48.0, -45.0, Math.toRadians(90)),
                new Pose2d(-43.5, -36.25, Math.toRadians(90)),
                new Pose2d(-36.7, -36.25, Math.toRadians(25))
        };
        this.backdrop = new Pose2d[] {
                new Pose2d(47.8, -29, Math.toRadians(180)),
                new Pose2d(47.8, -36, Math.toRadians(180)),
                new Pose2d(47.8, -42, Math.toRadians(180))
        };

        this.parking = new Pose2d(45.0, -20.0, Math.toRadians(180));

        this.backOffFromSpike = new Pose2d[] {
                new Pose2d(-36, -48.5, Math.toRadians(90)),
                new Pose2d(-40, -43.5, Math.toRadians(90)),
                new Pose2d(-40, -45, Math.toRadians(90))
        };

        this.stackIntakeAlignment = new Pose2d[] {
                new Pose2d(-36, -12.05, Math.toRadians(90)),
                new Pose2d(-50, -36.75, Math.toRadians(180)),
                new Pose2d(-50, -12.05, Math.toRadians(180))
        };

        this.stackIntake = new Pose2d[] {
                new Pose2d(-58.75, -12.25, Math.toRadians(180)),
                new Pose2d(-58.5, -36.75, Math.toRadians(180)),
                new Pose2d(-58.75, -12.25, Math.toRadians(180))
        };
        this.crossFieldAlignment = new Pose2d[] {
                new Pose2d(-48, -11.75, Math.toRadians(180)),
                new Pose2d(-40, -36.0, Math.toRadians(180)),
                new Pose2d(-48, -11.75, Math.toRadians(180))
        };

        this.backdropAlignment = new Pose2d[] {
                new Pose2d(42.0, -11.25, Math.toRadians(180)),
                new Pose2d(42.0, -36.0, Math.toRadians(180)),
                new Pose2d(42.0, -11.25, Math.toRadians(180))
        };

        cycleScore = new Pose2d[] {
                new Pose2d(47.5, -39.0, Math.toRadians(180)),
                new Pose2d(47.5, -31.75, Math.toRadians(180)),
                new Pose2d(47.5, -31.75, Math.toRadians(180))
        };

        this.moveUp1 =  new Pose2d(-40.0, -54.0, Math.toRadians(90));

        this.stackIntakeAlignment2 = new Pose2d(-50, -12.75, Math.toRadians(180));
    }

    @Override
    protected AlliancePosition getAlliance() {
        return AlliancePosition.RED;
    }

    @Override
    protected void printDescription() {
        telemetry.addData("Description", "RED Left Auto");
    }

}
