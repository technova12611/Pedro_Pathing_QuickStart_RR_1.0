package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.utils.hardware.GamePadController;
import org.firstinspires.ftc.teamcode.utils.hardware.HardwareCreator;

import java.util.ArrayList;
import java.util.Collections;

@Config
//@Disabled
@TeleOp(group = "Test")
public class IRBeamBreakerTest extends LinearOpMode {

    public void runOpMode() throws InterruptedException {

        DigitalChannel beamBreaker1 = hardwareMap.get(DigitalChannel.class, "beamBreaker1");
        DigitalChannel beamBreaker2 = hardwareMap.get(DigitalChannel.class, "beamBreaker2");

        beamBreaker1.setMode(DigitalChannel.Mode.INPUT);
        beamBreaker2.setMode(DigitalChannel.Mode.INPUT);

        // Communicate to dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        boolean prevState = beamBreaker1.getState();
        boolean state = prevState;
        int counter = 0;
        int totalCounter = 0;
        while (opModeInInit()) {
            // Control servo selection
            state = beamBreaker1.getState();
            if(state && !prevState) {
                counter++;
            }

            prevState = state;

            telemetry.addData("Beam Breaker State: ", beamBreaker1.getState());
            telemetry.addData("Beam_Breaker_State: ", state?1:0);
            telemetry.addData("counter ", counter);
            telemetry.update();
            idle();
        }
    }
}
