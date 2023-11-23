package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.utils.software.AutoConstants;

@Config
@Autonomous(name = "Red Right Auto", group = "Auto", preselectTeleOp = "Manual Drive")
public class RedRightAuto extends AutoBase {
   public static Pose2d[] spike = {
           new Pose2d(24, -40, Math.toRadians(-90)),
           new Pose2d(10, -38, Math.toRadians(-90)),
           new Pose2d(8, -37, Math.toRadians(-45))
   };
   public static Pose2d[] spikeBackedOut =  {
           new Pose2d(22, -48, Math.toRadians(-90)),
           new Pose2d(10, -46, Math.toRadians(-90)),
           new Pose2d(16, -45, Math.toRadians(-45))
   };
   // 0 = right, 1 = middle, 2 = left
   public static Pose2d start = new Pose2d(12, -61, Math.toRadians(-90));
   public static Pose2d parking = new Pose2d(56, -58, Math.toRadians(180));

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
      deliverSpike();
      scorePreload();
      park();
   }

   private void deliverSpike() {
      if (SPIKE != 2) {
         sched.addAction(
                 drive.actionBuilder(getStartPose())
                    .strafeTo(spike[SPIKE].position)
                    .build()
         );
      } else {
         sched.addAction(
                 drive.actionBuilder(getStartPose())
                         .lineToY((spike[SPIKE].position.y + getStartPose().position.y)/2)
                         .strafeToLinearHeading(spike[SPIKE].position, spike[SPIKE].heading)
                         .build()
         );
      }
      sched.addAction(
                  new SleepAction(0.5)
      );
   }

   private void scorePreload() {
      sched.addAction(
              new SequentialAction(
                      drive.actionBuilder(spike[SPIKE])
                              .strafeToLinearHeading(spikeBackedOut[SPIKE].position, spikeBackedOut[SPIKE].heading)
                              .strafeToLinearHeading(AutoConstants.redScoring[SPIKE].position, AutoConstants.redScoring[SPIKE].heading)
                              .build(),
//                      outtake.wristScoring(),
                      outtake.extendOuttakeLowBlocking(),
                      drive.actionBuilder(AutoConstants.redScoring[SPIKE])
                              .strafeToLinearHeading(AutoConstants.redScoring[SPIKE].position.plus(new Vector2d(12, 0)), AutoConstants.redScoring[SPIKE].heading) // Correct for any turning that occured during the previous move
                              .build(),
//                      outtake.latchScoring(),
                      new SleepAction(0.5),
                      outtake.extendOuttakeMidBlocking()
              )
      );
   }

   private void park() {
      sched.addAction(
              drive.actionBuilder(new Pose2d(AutoConstants.redScoring[SPIKE].position.plus(new Vector2d(10, 0)),
                              AutoConstants.redScoring[SPIKE].heading))
                      .strafeToLinearHeading(AutoConstants.redScoring[SPIKE].position, AutoConstants.redScoring[SPIKE].heading)
                      .afterDisp(10, new SequentialAction(
//                              outtake.wristStored(),
//                              new SleepAction(0.5),
                              outtake.retractOuttake()
//                              outtake.latchClosed(),
//                              new SleepAction(0.5)
                      ))
                      .strafeToLinearHeading(new Vector2d(AutoConstants.redScoring[SPIKE].position.x, parking.position.y), parking.heading)
                      .strafeToLinearHeading(parking.position, parking.heading)
                      .build()
      );
   }
}
