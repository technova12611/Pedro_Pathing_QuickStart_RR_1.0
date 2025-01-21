package org.firstinspires.ftc.teamcode.opmode;

import android.util.Log;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.util.PoseMessage;
import org.firstinspires.ftc.teamcode.utils.hardware.GamePadController;
import org.firstinspires.ftc.teamcode.utils.software.AutoActionScheduler;


@Config
public abstract class AutoBase extends LinearOpMode {

    protected Robot robot;
    protected Follower follower;

    protected AutoActionScheduler sched;

    protected int sampledCollected = 0;

    ElapsedTime loopTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    final public void update() {
        telemetry.addData("Time left", 30 - getRuntime());
        robot.update();
    }
    //
    final public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing... Please wait");
        telemetry.update();

        // Init subsystems
        GamePadController g1 = new GamePadController(gamepad1);

        this.robot = new Robot(hardwareMap,true, true);
        this.robot.initialize();

        //follower = robot.driveTrain.follower;
        follower.setStartingPose(getStartPose());

        this.sched = new AutoActionScheduler(this::update, hardwareMap);

        // initialize front and back vision portals
        loopTimer.reset();

        long previousLoopTime = System.currentTimeMillis();
        while (opModeInInit()) {
            g1.update();

            telemetry.addLine(printDescription());
            telemetry.update();
        }

        resetRuntime();

        // run the auto path, all the actions are queued
        //-------------------------------
        onRun();

        // end of the auto run
        // keep position and settings in memory for TeleOps
        //--------------------------------------------------
        //setEndingPose();
        Log.d("Auto_logger", String.format("!!! Auto program ended at %.3f", getRuntime()));
    }


    // the following needs to be implemented by the real auto program
    // mainly the path and other scoring actions

    /**
     * define the different starting pose for each locations
     * RED Right, RED Left, BLUE Right, BLUE Left. All have different Starting Posees.
     *
     */
    protected abstract Pose getStartPose();

    /**
     * print the user friendly message to alert driver the program is running
     */
    protected abstract String printDescription();

    /**
     * Build the auto path and queue up actions to execute
     */
    protected abstract void onRun();

    protected abstract void setEndingPose();

    // these are needed to know where the robot located
    //   protected abstract Globals.Alliance getAlliance();
    public class WaitSampleCollectedInPathAction implements Action {

        boolean firstTime = true;
        ElapsedTime timer = null;

        double previousLogTime = 0.0;

        public WaitSampleCollectedInPathAction() {
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if(firstTime) {
                firstTime = false;
                timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
                Log.d("WaitSampleCollectedInPath_logger", "started!");
            }
            assert follower != null;
            boolean shouldContinue = follower.isBusy(); //(!robot.intake.hasSampleInBucket() && follower.isBusy());

            // we got the sample collected in the path, break path immediately
            if(!shouldContinue) {

                if (follower.isBusy()) {
                    Log.d("WaitSampleCollectedInPath_logger", "collected sample in the path, breaking path !!! time taken: "
                            + String.format("%3.1f", timer.milliseconds()));
                    follower.breakFollowing();
                }
            }

            if((System.currentTimeMillis() - previousLogTime) > 100.0) {
                Log.d("WaitSampleCollectedInPath_logger", "Current Pose: " + new PoseMessage(follower.getPose()));
                previousLogTime = System.currentTimeMillis();
            }

            return shouldContinue;
        }
    }

}

