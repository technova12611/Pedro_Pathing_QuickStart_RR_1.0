package org.firstinspires.ftc.teamcode.utils.control;

/**
 * Proportional, integral, and derivative (PID) gains used by [PIDFController].
 *
 * @param kP proportional gain
 * @param kI integral gain
 * @param kD derivative gain
 */
public class PIDCoefficients {
    public double kP = 0.0;
    public double kD = 0.0;
    public double kI = 0.0;

    public PIDCoefficients(double kP, double kD, double kI) {
        this.kP = kP;
        this.kD = kD;
        this.kI = kI;
    }
}