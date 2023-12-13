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
@Autonomous(name = "RED Right Cycle Auto", group = "RED Auto", preselectTeleOp = "Manual Drive")
public class RedRightCycleAuto extends NearCycleAutoBase {

    @Override
    protected void onInit() {
        this.start = new Pose2d(16.0, -62.0, Math.toRadians(90));
    }

    protected AlliancePosition getAlliance() {
        return AlliancePosition.RED;
    }

    @Override
    protected void printDescription() {
        telemetry.addData("Description", "RED Right Cycle Auto");
    }

}
