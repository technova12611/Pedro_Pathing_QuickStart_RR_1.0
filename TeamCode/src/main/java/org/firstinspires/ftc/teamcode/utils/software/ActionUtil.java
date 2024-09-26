package org.firstinspires.ftc.teamcode.utils.software;

import android.util.Log;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.Callable;

public class ActionUtil {
   public static class DcMotorExPowerAction implements Action {
      double power;
      DcMotorEx motor;

      public DcMotorExPowerAction(DcMotorEx motor, double power) {
         this.power = power;
         this.motor = motor;
      }

      @Override
      public boolean run(TelemetryPacket packet) {
         motor.setPower(power);
         return false;
      }
   }

   public static class DcMotorExRTPAction implements Action {
      int position;
      double power;
      DcMotorEx motor;

      public DcMotorExRTPAction(DcMotorEx motor, int position, double power) {
         this.position = position;
         this.motor = motor;
         this.power = power;
      }

      @Override
      public boolean run(TelemetryPacket packet) {
         Log.d("ActionUtil.Motor_RTP", "target: " + position + String.format("| power: %.2f", power));
         motor.setTargetPosition(position);
         motor.setPower(power);
         motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         return false;
      }
   }

   public static class ServoPositionAction implements Action {
      double position;
      Servo servo;
      String servoName = "unknown";

      public ServoPositionAction(Servo servo, double position, String servoName) {
         this.servo = servo;
         this.position = position;
         this.servoName = servoName;
      }

      public ServoPositionAction(Servo servo, double position) {
         this.servo = servo;
         this.position = position;
      }

      @Override
      public boolean run(TelemetryPacket packet) {
         servo.setPosition(position);
         String message = "Servo " + this.servoName + " set to position: " + position;
//         packet.addLine(message);
         Log.d("ActionUtil_servoAction_debug", message);
         return false;
      }
   }

   public static class CRServoAction implements Action {
      double power;
      CRServo servo;

      public CRServoAction(CRServo servo, double power) {
         this.servo = servo;
         this.power = power;
      }

      @Override
      public boolean run(TelemetryPacket packet) {
         servo.setPower(power);
         return false;
      }
   }

   public static class RunnableAction implements Action {
      Callable<Boolean> action;

      public RunnableAction(Callable<Boolean> action) {
         this.action = action;
      }

      @Override
      public boolean run(TelemetryPacket packet) {
         try {
            return action.call();
         } catch (Exception e) {
            throw new RuntimeException(e);
         }
      }
   }

   public static class DcMotorRWEAction implements Action {
      double duration;
      double power;
      DcMotorEx motor;
      ElapsedTime timer = null;

      public DcMotorRWEAction(DcMotorEx motor, double power, double duration) {
         this.duration = duration;
         this.motor = motor;
         this.power = power;
      }

      @Override
      public boolean run(TelemetryPacket packet) {
         if(this.timer == null) {
            this.timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
            Log.d("ActionUtil.Motor_RWE_logger", "Starting:: duration: " + String.format("|%.3f", duration) + String.format("| power: %.2f", power)
            + " | current timer: " + this.timer.milliseconds());
            motor.setPower(power);

            return true;
         }

         if(this.timer.milliseconds() >= duration) {
            Log.d("ActionUtil.Motor_RWE_logger", "Ending:: duration: " + String.format("|%.3f", duration) + String.format("| power: %.2f", power));
            motor.setPower(0.0);

            return false;
         }

         return true;
      }
   }
}
