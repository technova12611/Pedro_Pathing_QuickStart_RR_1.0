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
@Autonomous(name = "Blue RIGHT Auto (2+1)", group = "BLUE Auto", preselectTeleOp = "Manual Drive")
public class BlueRightAuto extends FarAutoBase {
   @Override
   protected void onInit() {
      this.start = new Pose2d(-40.0, 62.0, Math.toRadians(-90));
      this.spike = new Pose2d[] {
              new Pose2d(-38.5, 38.5, Math.toRadians(-25)),
              new Pose2d(-42.5, 35.5, Math.toRadians(-90)),
              new Pose2d(-48.0, 45.0, Math.toRadians(-90))
      };
      this.backdrop = new Pose2d[] {
              new Pose2d(46.2, 42, Math.toRadians(-180)),
              new Pose2d(46.2, 36, Math.toRadians(-180)),
              new Pose2d(46.2, 29, Math.toRadians(-180))
      };

      this.parking = new Pose2d(45.0, 20.0, Math.toRadians(-180));

      this.cycleScore = new Pose2d[] {
              new Pose2d(46.2, 32.0, Math.toRadians(-180)),
              new Pose2d(46.2, 32.0, Math.toRadians(-180)),
              new Pose2d(46.2, 39.0, Math.toRadians(-180)),
      };

      this.backOffFromSpike = new Pose2d[] {
              new Pose2d(-40, 45, Math.toRadians(-90)),
              new Pose2d(-40, 43, Math.toRadians(-90)),
              new Pose2d(-36, 46, Math.toRadians(-90))
      };

      this.stackIntakeAlignment = new Pose2d[] {
              new Pose2d(-50, 11.0, Math.toRadians(-180)),
              new Pose2d(-50, 34.5, Math.toRadians(-180)),
              new Pose2d(-36, 11.0, Math.toRadians(-90))
      };

      this.stackIntake = new Pose2d[] {
              new Pose2d(-58.5, 11.0, Math.toRadians(-180)),
              new Pose2d(-58.5, 34.5, Math.toRadians(-180)),
              new Pose2d(-58.5, 11.0, Math.toRadians(180))
      };
      this.crossFieldAlignment = new Pose2d[] {
              new Pose2d(-48, 11, Math.toRadians(-180)),
              new Pose2d(-40, 35, Math.toRadians(-180)),
              new Pose2d(-48, 11, Math.toRadians(-180))
      };

      this.backdropAlignment = new Pose2d[] {
              new Pose2d(45.0, 10.5, Math.toRadians(-180)),
                      new Pose2d(45.0, 35.0, Math.toRadians(-180)),
                              new Pose2d(45.0, 10.5, Math.toRadians(-180))};

      this.moveUp1 =  new Pose2d(-40.0, 54.0, Math.toRadians(-90));

      this.stackIntakeAlignment2 = new Pose2d(-50, 11.0, Math.toRadians(-180));
   }

   @Override
   protected AlliancePosition getAlliance() {
      return AlliancePosition.BLUE;
   }

   @Override
   protected void printDescription() {
      telemetry.addData("Description", "Blue RIGHT Auto");
   }
}
