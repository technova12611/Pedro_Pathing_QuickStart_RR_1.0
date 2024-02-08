package org.firstinspires.ftc.teamcode.utils.software;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.opmode.autonomous.AutoBase;
import org.firstinspires.ftc.teamcode.opmode.autonomous.BackdropPositionCallback;
import org.firstinspires.ftc.teamcode.opmode.autonomous.PreloadPositionDetectionCallback;
import org.firstinspires.ftc.teamcode.opmode.autonomous.StackPositionCallback;

import java.util.LinkedList;
import java.util.Queue;

public class AutoActionScheduler {
   final Queue<Action> actions = new LinkedList<>();
   final FtcDashboard dash = FtcDashboard.getInstance();
   final Canvas canvas = new Canvas();
   final Runnable pidUpdate;

   public long autoRunElapsedTime = 0;

   int actionOrder = 0;

   private StackPositionCallback stackCallback;

   private BackdropPositionCallback backdropCallback;

   private PreloadPositionDetectionCallback preloadPositionCallback;

   public AutoActionScheduler(Runnable pidUpdate) {
      this.pidUpdate = pidUpdate;
   }

   public void addAction(Action action) {
      actions.add(action);
   }

   public void setStackAlignmentCallback(StackPositionCallback stackCallback) {
      this.stackCallback = stackCallback;
   }

   public void setBackdropAlignmentCallback(BackdropPositionCallback backdropCallback) {
      this.backdropCallback = backdropCallback;
   }

   public void setPreloadPositionCallback(PreloadPositionDetectionCallback preloadPositionCallback) {
      this.preloadPositionCallback = preloadPositionCallback;
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
            if(a instanceof AutoBase.StackIntakePositionAction) {
               if(stackCallback != null) {
                  ((LinkedList) actions).addFirst(stackCallback.driveToStack());
                  Log.d("AutoActionScheduler:", "** Added a new StackDriveAction " + " completed at " + (System.currentTimeMillis()-startTime + "(ms)"));
               }
            }

            if(a instanceof AutoBase.PreloadPositionDetectionAction) {
               if(preloadPositionCallback != null) {
                  ((LinkedList) actions).addFirst(preloadPositionCallback.strafeToBackdrop());
                  Log.d("AutoActionScheduler:", "** Added a new PreloadPositionDetectionAction " + " completed at " + (System.currentTimeMillis()-startTime + "(ms)"));
               }
            }
            Log.d("AutoActionScheduler:", "Action: " + a + " - " + (++actionOrder) + " finished at " + (System.currentTimeMillis()-startTime + "(ms)"));
         }
      }

      autoRunElapsedTime = System.currentTimeMillis() - startTime;

      Log.d("AutoActionScheduler:","Action scheduler completed at " + (System.currentTimeMillis()-startTime) + " (ms)");
   }

   public boolean isEmpty() {
      return actions.isEmpty();
   }

   public void reset() {
      while (actions.peek() != null && !Thread.currentThread().isInterrupted()) {
         actions.remove();
      }
   }

   public int size() {
      return actions.size();
   }
}
