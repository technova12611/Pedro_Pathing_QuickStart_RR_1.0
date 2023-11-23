package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.hardware.HardwareCreator;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class IRBeamBreaker {
    final DigitalChannel beamBreaker1;
    final DigitalChannel beamBreaker2;

    public static int counter;
    public static int totalCounter;

    public IRBeamBreaker(HardwareMap hardwareMap) {
        beamBreaker1 = hardwareMap.get(DigitalChannel.class, "beamBreaker1");
        beamBreaker2 = hardwareMap.get(DigitalChannel.class, "beamBreaker2");

        beamBreaker1.setMode(DigitalChannel.Mode.INPUT);
        beamBreaker2.setMode(DigitalChannel.Mode.INPUT);

        counter=0;
        totalCounter = 0;
    }

    public class IRBeamBreakerAction implements Action {

        boolean prevState, curState;
        Intake intake;

        public IRBeamBreakerAction(Intake intake) {
            prevState = beamBreaker1.getState();
            curState = prevState;
            this.intake = intake;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            curState = beamBreaker1.getState();

            if(!curState && prevState) {
                totalCounter++;
                counter++;
                packet.addLine("Picked up one more pixel, current holding: " + counter + " | total picked up: " + totalCounter);
            }

            if(curState != prevState) {
                prevState =  curState;
            }

            if(counter++ == 2 && intake.intakeState == Intake.IntakeState.ON) {
                packet.addLine("Picked up 2 already, reversing intake for 2 seconds, then stop intake.");
                ExecutorService executor1 = Executors.newSingleThreadExecutor();
                executor1.submit(() -> {
                    intake.intakeReverseThenStop();
                });

                packet.addLine("Picked up one more pixel, current holding: " + counter + " | total picked up: " + totalCounter);
            }
            return true;
        }
    }
}
