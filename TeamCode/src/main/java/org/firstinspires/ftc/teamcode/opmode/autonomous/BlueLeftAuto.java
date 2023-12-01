package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pipeline.AlliancePosition;

@Config
@Autonomous(name = "Blue Left Auto", group = "Auto", preselectTeleOp = "Manual Drive")
public class BlueLeftAuto extends AutoBase {
   public static Pose2d[] spike = {
           new Pose2d(28.5, 24.5, Math.toRadians(-180)),
           new Pose2d(12.5, 26.5, Math.toRadians(-180)),
           new Pose2d(34.5, 26.5, Math.toRadians(-180))
   };
   public static Pose2d[] backdrop =  {
           new Pose2d(48, 28, Math.toRadians(-180)),
           new Pose2d(48, 36, Math.toRadians(-180)),
           new Pose2d(48, 45, Math.toRadians(-180))
   };
   // 0 = left, 1 = middle, 2 = right
   public static Pose2d start = new Pose2d(16.0, 62.0, Math.toRadians(-90));
   public static Pose2d parking = new Pose2d(53.0, 60.0, Math.toRadians(-180));

   protected AlliancePosition getAlliance() {
      return AlliancePosition.BLUE;
   }

   @Override
   protected Pose2d getStartPose() {
      return start;
   }

   @Override
   protected void printDescription() {
      telemetry.addData("Description", "Red Right Auto");
   }

   @Override
   protected void onRun() {

      sched.addAction(
              new SequentialAction(
                      // to score yellow pixel on the backdrop
                      drive.actionBuilder(drive.pose)
                              .setTangent(0)
                              .splineTo(backdrop[SPIKE].position, -Math.PI/2)
                              .build(),
                      outtake.extendOuttakeLow(),
                      outtake.prepareToScore(),
                      outtake.latchScore1(),
                      new SleepAction(0.75),
                      new ParallelAction(
                              outtake.retractOuttake(),
                              intake.stackIntakeLinkageDown(),

                              // to score the purple pixel on the spike
                              drive.actionBuilder(backdrop[SPIKE])
                                      .strafeTo(spike[SPIKE].position)
                                      .build()
                      ),

                      intake.scorePurplePreload(),
                      new SleepAction(0.5),

                      // to park and prepare for teleops
                      intake.prepareTeleOpsIntake(),
                      outtake.prepareToTransfer(),

                      drive.actionBuilder(spike[SPIKE])
                              .setReversed(true)
                              .strafeTo(parking.position)
                              .build()
              )
      );
   }
}
