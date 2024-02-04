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
import org.firstinspires.ftc.teamcode.subsystem.Outtake;
import org.firstinspires.ftc.teamcode.utils.software.ActionUtil;

@Config
public abstract class NearAutoBase extends AutoBase {
   public Pose2d[] spike;
   public Pose2d[] backdrop;
   // 0 = left, 1 = middle, 2 = right
   public Pose2d start;
   public Pose2d start_forward;
   public Pose2d parking;
   public Pose2d parking_2;

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
                      new MecanumDrive.DrivePoseLoggingAction(drive, "starting_position"),
                      outtake.prepareToSlide(),
                      // to score yellow pixel on the backdrop
                      new ParallelAction(
                      drive.actionBuilder(drive.pose)
                              .strafeToLinearHeading(backdrop[SPIKE].position,
                                      backdrop[SPIKE].heading, drive.slowVelConstraint,drive.slowAccelConstraint)
                              .build(),
                          new SequentialAction(
                                  new SleepAction(1.25),
                              outtake.extendOuttakeLow()
                          )
                      ),
                      new MecanumDrive.DrivePoseLoggingAction(drive, "backdrop_position"),
                      outtake.prepareToScoreCycle(),
                      new SleepAction(0.5),
                      getBackdropDistanceAdjustmentAction(),
                      outtake.latchScore1(),
                      intake.stackIntakeLinkageDown(),
                      new SleepAction(1.0),
                      outtake.afterScore(),
                      new SleepAction(0.50),
                      new ActionUtil.RunnableAction(() -> {
                          pidDriveActivated = false;
                          straightDistance = 0.0;
                          return false;
                      })
              ));

      // score purple pixel
       sched.addAction(
               new SequentialAction(
                  new ParallelAction(
                      new SequentialAction(
                              outtake.prepareToSlide(),
                          new SleepAction(0.25),
                          outtake.retractOuttake()),

                          // to score the purple pixel on the spike
                          drive.actionBuilder(backdrop[SPIKE])
                                  .strafeToLinearHeading(spike[SPIKE].position, spike[SPIKE].heading, drive.slowVelConstraint,drive.slowAccelConstraint)
                                  .build()
                  ),
                  new MecanumDrive.DrivePoseLoggingAction(drive, "spike_position"),

                  intake.scorePurplePreload(),
                  new SleepAction(0.5)
               )
       );

       // to park and prepare for teleops
       sched.addAction(
           new SequentialAction(
              new ParallelAction(
                      intake.prepareTeleOpsIntake(),
                      outtake.prepareToTransfer(),
                      drive.actionBuilder(spike[SPIKE])
                              .setReversed(true)
                              .strafeToLinearHeading(parking.position, parking.heading)
                              .strafeToLinearHeading(parking_2.position, parking_2.heading)
                              .build()
              ),

              new SleepAction(1.0),
              new MecanumDrive.DrivePoseLoggingAction(drive, "parking_position")
          )
      );
   }

   @Override
   public FieldPosition getFieldPosition() {
       return FieldPosition.NEAR;
   }

}
