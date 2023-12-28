package org.firstinspires.ftc.teamcode.roadrunner.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.TwoDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.roadrunner.messages.PoseMessage;
import org.firstinspires.ftc.teamcode.roadrunner.ThreeDeadWheelLocalizer;

public final class StraightTest extends LinearOpMode {
    public static double distance = 36.0;
    public static int mode = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        TwoDeadWheelLocalizer localizer = (TwoDeadWheelLocalizer)drive.localizer;

        int begin = localizer.par.getPositionAndVelocity().position;
        telemetry.addData("par0 encoder begin value: ", begin);
        //telemetry.addData("par1 encoder begin value: ", localizer.par1.getPositionAndVelocity().position);
        telemetry.addData("perp encoder begin value: ", localizer.perp.getPositionAndVelocity().position);
        telemetry.update();
        waitForStart();

        Actions.runBlocking(
            drive.actionBuilder(drive.pose)
                    .lineToX(distance)
                    .build());

        if(mode == 1) {
            Actions.runBlocking(
                    new SequentialAction(
                            new SleepAction(1.0),
            drive.actionBuilder(drive.pose)
                    .lineToX(0)
                    .build()));
        }

        telemetry.addData("end of run pose estimate: ", new PoseMessage(drive.pose));
        telemetry.update();

        while (!isStopRequested()) {
            drive.updatePoseEstimate();
            telemetry.addData("Pose estimate: ", new PoseMessage(drive.pose));
            int end = localizer.par.getPositionAndVelocity().position;
            telemetry.addData("par0 encoder end value: ", end);
            telemetry.addData("par0 encoder delta value: ", end-begin);
//            telemetry.addData("par1 encoder end value: ", localizer.par1.getPositionAndVelocity().position);
            telemetry.addData("perp encoder end value: ", localizer.perp.getPositionAndVelocity().position);
            telemetry.addData("IMU value: ", String.format("%3.2f",drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)));
            telemetry.update();
        }
    }
}
