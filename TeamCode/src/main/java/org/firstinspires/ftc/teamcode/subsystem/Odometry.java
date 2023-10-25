package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.hardware.Encoder;

import java.util.Arrays;
import java.util.List;

/**
 * The odometry system (dead tracking wheels) of the robot
 *
 *    /--------------\
 *    |              |
 *    |              |
 *    |      ||      |
 *    |      ||      |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class Odometry  {

    private Encoder trackingWheelAnterior, trackingWheelLateral;
    private List<Pose2d> trackingWheelPositions;

    public static double TICKS_PER_REV = 8192; // From https://www.revrobotics.com/rev-11-1271/
    public static double WHEEL_RADIUS = (35/2.0)/25.4; // in (diameter mm -> radius mm -> radius in)
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_MULTIPLIER = 1.014; // Y  | change to 1.025
    public static double ANTERIOR_MULTIPLIER = 1.015; // X | change to 1.037

    public Odometry(DcMotorEx anterior, DcMotorEx lateral) {
        trackingWheelAnterior = new Encoder(anterior); // Front Back (X)
        trackingWheelLateral = new Encoder(lateral); // Left Right (Y)

        trackingWheelAnterior.setDirection(Encoder.Direction.FORWARD);
        trackingWheelLateral.setDirection(Encoder.Direction.FORWARD);

        // TODO: Double check tracking wheel positions
        trackingWheelPositions = Arrays.asList(
                new Pose2d(4.5, 2.0, 0), // Anterior (forward back)
                new Pose2d(2.80, -1.0, Math.toRadians(-90.0)) // Lateral (left right)
        );
    }

    public void sendTelemetry(Telemetry telemetry) {
        telemetry.addData("Odometry", "X: %.3fin | Y: %.3fin",
                encoderTicksToInches(getAnteriorTicks()),
                encoderTicksToInches(getLateralTicks()));
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public List<Double> getWheelPositions() {
//        RobotLog.d("     Odometry-getWheelPositions: Current Thread- " + Thread.currentThread().getId() + " :: " + Thread.currentThread().getName() );
        List<Double> myList = Arrays.asList(
                encoderTicksToInches(getAnteriorTicks()),
                encoderTicksToInches(getLateralTicks())
        );

//        RobotLog.d("     Odometry-getWheelPositions end !!!");

        return myList;
    }

    public List<Double> getWheelVelocities() {
//        RobotLog.d("     Odometry-getWheelVelocities: Current Thread- " + Thread.currentThread().getId() + " :: " + Thread.currentThread().getName() );
        List<Double> myList =  Arrays.asList(
                encoderTicksToInches(trackingWheelAnterior.getCorrectedVelocity())*1.0,//1.015,
                encoderTicksToInches(trackingWheelLateral.getCorrectedVelocity()*1.0) //1.015)
        );
//        RobotLog.d("     Odometry-getWheelVelocities end !!!");
        return myList;
    }

    private double getAnteriorTicks() {
        return trackingWheelAnterior.getCurrentPosition()*ANTERIOR_MULTIPLIER;
    }

    private double getLateralTicks() {
        return trackingWheelLateral.getCurrentPosition()*LATERAL_MULTIPLIER;
    }

    public List<Pose2d> getTrackingWheelPositions() {
        return trackingWheelPositions;
    }
}
