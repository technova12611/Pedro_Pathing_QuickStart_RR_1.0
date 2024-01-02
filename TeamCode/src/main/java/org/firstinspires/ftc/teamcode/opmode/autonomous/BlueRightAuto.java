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
@Autonomous(name = "Blue Far Auto (2+1)", group = "BLUE Auto", preselectTeleOp = "Manual Drive")
public class BlueRightAuto extends FarAutoBase {
   @Override
   protected void onInit() {
      this.start = new Pose2d(-38.5, 62.0, Math.toRadians(-90));
      this.spike = new Pose2d[] {
              new Pose2d(-35.0, 38.5, Math.toRadians(-25)),
              new Pose2d(-41.0, 36.7, Math.toRadians(-90)),
              new Pose2d(-46.5, 45.0, Math.toRadians(-90))
      };

      this.backdrop = new Pose2d[] {
              new Pose2d(47.5, 38.5, Math.toRadians(-180)),
              new Pose2d(47.5, 33.5, Math.toRadians(-180)),
              new Pose2d(47.5, 27.8, Math.toRadians(-180))
      };

      this.parking = new Pose2d(42.0, 20.0, Math.toRadians(-180));

      this.cycleScore = new Pose2d[] {
              new Pose2d(47.3, 30.0, Math.toRadians(-180)),
              new Pose2d(47.3, 27.0, Math.toRadians(-180)),
              new Pose2d(47.3, 36.0, Math.toRadians(-180)),
      };

      this.backOffFromSpike = new Pose2d[] {
              new Pose2d(-38.5, 45, Math.toRadians(-90)),
              new Pose2d(-38.5, 43, Math.toRadians(-90)),
              new Pose2d(-35.2, 46, Math.toRadians(-90))
      };

      this.stackIntakeAlignment = new Pose2d[] {
              new Pose2d(-48.5, 11.5, Math.toRadians(-180)),
              new Pose2d(-48.5, 35.5, Math.toRadians(-180)),
              new Pose2d(-35.2, 11.5, Math.toRadians(-90))
      };

      this.stackIntake = new Pose2d[] {
              new Pose2d(-57.7, 11.25, Math.toRadians(-180)),
              new Pose2d(-57.5, 35.25, Math.toRadians(-180)),
              new Pose2d(-57.7, 11.25, Math.toRadians(180))
      };
      this.crossFieldAlignment = new Pose2d[] {
              new Pose2d(-46.5, 11.5, Math.toRadians(-180)),
              new Pose2d(-38.5, 35.7, Math.toRadians(-180)),
              new Pose2d(-46.5, 11.5, Math.toRadians(-180))
      };

      this.backdropAlignment = new Pose2d[] {
              new Pose2d(42.0, 11.5, Math.toRadians(-180)),
                      new Pose2d(42.0, 35.5, Math.toRadians(-180)),
                              new Pose2d(42.0, 11.5, Math.toRadians(-180))};

      this.moveUp1 =  new Pose2d(-38.5, 54.0, Math.toRadians(-90));

      this.stackIntakeAlignment2 = new Pose2d(-48.5, 11.0, Math.toRadians(-180));
   }

   @Override
   protected AlliancePosition getAlliance() {
      return AlliancePosition.BLUE;
   }

   @Override
   protected void printDescription() {
      telemetry.addData("Description", "Blue Far (Right-Side) Auto");
   }
}
