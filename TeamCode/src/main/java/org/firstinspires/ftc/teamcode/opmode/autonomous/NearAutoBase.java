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
public abstract class NearAutoBase extends AutoBase {
   public Pose2d[] spike;
   public Pose2d[] backdrop;
   // 0 = left, 1 = middle, 2 = right
   public Pose2d start;
   public Pose2d parking;

   protected AlliancePosition getAlliance() {
      return AlliancePosition.RED;
   }

   @Override
   protected Pose2d getStartPose() {
      return start;
   }

   @Override
   protected void onRun() {

      sched.addAction(
              new SequentialAction(
                      // to score yellow pixel on the backdrop
                      new ParallelAction(
                      drive.actionBuilder(drive.pose)
                              .strafeToLinearHeading(backdrop[SPIKE].position, backdrop[SPIKE].heading)
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
                      intake.stackIntakeLinkageDown(),
                      new SleepAction(1.0),
                      new ParallelAction(
                          new SequentialAction(
                              new SleepAction(0.25),
                              outtake.retractOuttake()),

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
       return FieldPosition.NEAR;
   }

}
