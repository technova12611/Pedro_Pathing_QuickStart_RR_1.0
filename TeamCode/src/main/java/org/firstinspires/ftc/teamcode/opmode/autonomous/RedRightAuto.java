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
@Autonomous(name = "RED Right Auto (2+0)", group = "RED Auto", preselectTeleOp = "Manual Drive")
public class RedRightAuto extends NearAutoBase {

   // 0 = left, 1 = middle, 2 = right
   public static Pose2d[] red_spike = {
           new Pose2d(14.5, -34.5, Math.toRadians(180)),
           new Pose2d(28.5, -25.75, Math.toRadians(180)),
           new Pose2d(36.0, -27.5, Math.toRadians(180))
   };
   public static Pose2d[] red_backdrop =  {
           new Pose2d(49.25, -29, Math.toRadians(180)),
           new Pose2d(49.25, -36, Math.toRadians(180)),
           new Pose2d(49.25, -42, Math.toRadians(180))
   };
   // 0 = left, 1 = middle, 2 = right
   public static Pose2d red_start = new Pose2d(16.0, -62.0, Math.toRadians(90));
   public static Pose2d red_parking = new Pose2d(53.0, -60.0, Math.toRadians(180));

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
