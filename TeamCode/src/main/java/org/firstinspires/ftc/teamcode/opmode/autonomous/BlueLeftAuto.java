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
@Autonomous(name = "Blue Near (LEFT) Auto (2+0)", group = "BLUE Auto", preselectTeleOp = "Manual Drive")
public class BlueLeftAuto extends NearAutoBase {
   public static Pose2d[] blue_spike = {
           new Pose2d(36.0, 26.5, Math.toRadians(-180)),
           new Pose2d(28.5, 24.5, Math.toRadians(-180)),
           new Pose2d(13.5, 33.0, Math.toRadians(-180))
   };

   public static Pose2d[] blue_backdrop =  {
           new Pose2d(48.0, 42.0, Math.toRadians(-180)),
           new Pose2d(48.0, 36.0, Math.toRadians(-180)),
           new Pose2d(48.0, 29.5, Math.toRadians(-180))
   };
   // 0 = left, 1 = middle, 2 = right
   public static Pose2d blue_start = new Pose2d(14.5, 62.0, Math.toRadians(-90));
   public static Pose2d blue_parking = new Pose2d(51.0, 61.5, Math.toRadians(-180));

   public static Pose2d blue_parking_2 = new Pose2d(57.0, 61.5, Math.toRadians(-180));

   @Override
   protected void onInit() {
      this.start = blue_start;
      this.spike = blue_spike;
      this.backdrop =  blue_backdrop;
      this.parking = blue_parking;
      this.parking_2 = blue_parking_2;
   }
   protected AlliancePosition getAlliance() {
      return AlliancePosition.BLUE;
   }

   @Override
   protected void printDescription() {
      telemetry.addData("Description", "BLUE Near (Left-Side) Auto");
   }
}
