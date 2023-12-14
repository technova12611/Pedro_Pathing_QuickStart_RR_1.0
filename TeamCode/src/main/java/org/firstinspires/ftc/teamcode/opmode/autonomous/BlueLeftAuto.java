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
@Autonomous(name = "Blue LEFT Auto (2+0)", group = "BLUE Auto", preselectTeleOp = "Manual Drive")
public class BlueLeftAuto extends NearAutoBase {
   public static Pose2d[] blue_spike = {
           new Pose2d(35.5, 25.5, Math.toRadians(-180)),
           new Pose2d(29.25, 23.5, Math.toRadians(-180)),
           new Pose2d(14.1, 34.25, Math.toRadians(-180))
   };

   public static Pose2d[] blue_backdrop =  {
           new Pose2d(48.75, 42.0, Math.toRadians(-180)),
           new Pose2d(48.75, 36, Math.toRadians(-180)),
           new Pose2d(48.75, 28.5, Math.toRadians(-180))
   };
   // 0 = left, 1 = middle, 2 = right
   public static Pose2d blue_start = new Pose2d(16.0, 62.5, Math.toRadians(-90));
   public static Pose2d blue_parking = new Pose2d(53.0, 60.0, Math.toRadians(-180));

   @Override
   protected void onInit() {
      this.start = blue_start;
      this.spike = blue_spike;
      this.backdrop =  blue_backdrop;
      this.parking = blue_parking;
   }
   protected AlliancePosition getAlliance() {
      return AlliancePosition.BLUE;
   }

   @Override
   protected void printDescription() {
      telemetry.addData("Description", "BLUE Left Auto");
   }

}
