package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pipeline.AlliancePosition;
import org.firstinspires.ftc.teamcode.pipeline.FieldPosition;

@Config
@Autonomous(name = "Blue Right Auto", group = "BLUE Auto", preselectTeleOp = "Manual Drive")
public class BlueRightAuto extends AutoBase {
   public static Pose2d[] spike = {
           new Pose2d(35.5, 25.5, Math.toRadians(-180)),
           new Pose2d(28.5, 23.5, Math.toRadians(-180)),
           new Pose2d(14.1, 33.5, Math.toRadians(-180))
   };

   public static Pose2d[] backdrop =  {
           new Pose2d(49.2, 41.5, Math.toRadians(-180)),
           new Pose2d(49.2, 36, Math.toRadians(-180)),
           new Pose2d(49.2, 30.5, Math.toRadians(-180))
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
      telemetry.addData("Description", "BLUE Right Auto");
   }

   @Override
   protected void onRun() {

      sched.addAction(
              new SequentialAction(
                      // to score yellow pixel on the backdrop
                      new ParallelAction(
                              drive.actionBuilder(drive.pose)
                                      .setTangent(0)
                                      .splineTo(backdrop[SPIKE].position, -Math.PI/2)
                                      .build(),

                              outtake.prepareToSlide(),
                              new SequentialAction(
                                      new SleepAction(1.5),
                                      outtake.extendOuttakeLow()
                              )
                      ),

                      outtake.prepareToScore(),
                      new SleepAction(0.25),
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
                      new ParallelAction(
                         intake.prepareTeleOpsIntake(),
                         outtake.prepareToTransfer(),

                         drive.actionBuilder(spike[SPIKE])
                                 .setReversed(true)
                                 .strafeTo(parking.position)
                                 .build()
                      )
              )
      );
   }

   @Override
   public FieldPosition getFieldPosition() {
      return FieldPosition.FAR;
   }
}