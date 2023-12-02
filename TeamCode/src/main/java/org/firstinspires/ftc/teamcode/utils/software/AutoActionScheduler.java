package org.firstinspires.ftc.teamcode.utils.software;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.LinkedList;
import java.util.Queue;

public class AutoActionScheduler {
   final Queue<Action> actions = new LinkedList<>();
   final FtcDashboard dash = FtcDashboard.getInstance();
   final Canvas canvas = new Canvas();
   final Runnable pidUpdate;

   public long autoRunElapsedTime = 0;

   int actionOrder = 0;

   public AutoActionScheduler(Runnable pidUpdate) {
      this.pidUpdate = pidUpdate;
   }

   public void addAction(Action action) {
      actions.add(action);
   }

   public void run() {

      long startTime = System.currentTimeMillis();

      Log.d("AutoActionScheduler:","Action scheduler started ... | " + startTime);
      while (actions.peek() != null && !Thread.currentThread().isInterrupted()) {
         TelemetryPacket packet = new TelemetryPacket();
         packet.fieldOverlay().getOperations().addAll(canvas.getOperations());

         pidUpdate.run();

         Action a = actions.peek();
         a.preview(canvas);

         boolean running =  a.run(packet);
         dash.sendTelemetryPacket(packet);

         if (!running) {
            actions.remove();
            Log.d("AutoActionScheduler:", "Action " + (++actionOrder) + " finished at " + (System.currentTimeMillis()-startTime + "(ms)"));
         }
      }

      autoRunElapsedTime = System.currentTimeMillis() - startTime;

      Log.d("AutoActionScheduler:","Action scheduler completed at " + (System.currentTimeMillis()-startTime) + " (ms)");
   }

   public boolean isEmpty() {
      return actions.isEmpty();
   }
}
