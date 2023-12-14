package org.firstinspires.ftc.teamcode.opmode.autonomous;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.pipeline.AlliancePosition;
import org.firstinspires.ftc.teamcode.pipeline.FieldPosition;
import org.firstinspires.ftc.teamcode.roadrunner.PoseMessage;

@Config
@Autonomous(name = "Blue LEFT Cycle Auto (2+4)", group = "BLUE Auto", preselectTeleOp = "Manual Drive")
public class BlueLeftCycleAuto extends NearCycleAutoBase {

    @Override
    protected void onInit() {
        this.start = BlueLeftAuto.blue_start;

        this.backdrop = BlueLeftAuto.blue_backdrop;
        this.spike = BlueLeftAuto.blue_spike;

        this.cycleStart = new Pose2d[]{
                new Pose2d(spike[0].position.x, 12.0, Math.toRadians(-180)),
                new Pose2d(spike[1].position.x, 12.0, Math.toRadians(-180)),
                new Pose2d(spike[2].position.x, 12.0, Math.toRadians(-180))
        };

        this.stackAlignment = new Pose2d(-50.0, 11.0, Math.toRadians(-180));
        this.stackIntake = new Pose2d(-54.25, 11.25, Math.toRadians(-180));
        this.safeTrussPassStop = new Pose2d(-49.0, 11.0, Math.toRadians(-180));

        this.backdropAlignment = new Pose2d(45.0, 11.0, Math.toRadians(-180));

        this.cycleScore = new Pose2d[]{
                new Pose2d(48.0, 33.0, Math.toRadians(-180)),
                new Pose2d(48.0, 31.0, Math.toRadians(-180)),
                new Pose2d(48.0, 38.0, Math.toRadians(-180))
        };

        this.parking = new Pose2d(45.0, 20.0, Math.toRadians(-180));
    }

    protected AlliancePosition getAlliance() {
        return AlliancePosition.BLUE;
    }

    @Override
    protected void printDescription() {
        telemetry.addData("Description", "BLUE Left Cycle Auto");
    }

}
