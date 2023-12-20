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
@Autonomous(name = "RED Near Auto (2+0)", group = "RED Auto", preselectTeleOp = "Manual Drive")
public class RedRightAuto extends NearAutoBase {

   // 0 = left, 1 = middle, 2 = right
   public static Pose2d[] red_spike = {
           new Pose2d(14.8, -34.5, Math.toRadians(180)),
           new Pose2d(30.2, -26.7, Math.toRadians(180)),
           new Pose2d(36.5, -27.5, Math.toRadians(180))
   };
   public static Pose2d[] red_backdrop =  {
           new Pose2d(49.4, -32.25, Math.toRadians(180)), // 29.0 for left
           new Pose2d(49.4, -38.0, Math.toRadians(180)),  // 35.5 for left
           new Pose2d(49.4, -44.0, Math.toRadians(180))  // 42 for left
   };
   // 0 = left, 1 = middle, 2 = right
   public static Pose2d red_start = new Pose2d(16.0, -62.0, Math.toRadians(90));
   public static Pose2d red_parking = new Pose2d(53.0, -61.0, Math.toRadians(180));

   @Override
   protected void onInit() {
      this.spike = red_spike;
      this.backdrop =  red_backdrop;
      this.start = red_start;
      this.parking = red_parking;
   }

   @Override
   protected AlliancePosition getAlliance() {
      return AlliancePosition.RED;
   }

   @Override
   protected void printDescription() {
      telemetry.addData("Description", "RED Right Auto");
   }

}
