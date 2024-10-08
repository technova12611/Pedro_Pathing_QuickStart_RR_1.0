package org.firstinspires.ftc.teamcode.utils.hardware;


import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.utils.control.PIDCoefficients;
import org.firstinspires.ftc.teamcode.utils.control.PIDFController;

import kotlin.jvm.functions.Function2;

public class MotorWithVelocityPID {
    private DcMotorEx motor;
    private PIDFController pidfController;
    private PIDCoefficients pid;
    private int targetVelocity = 0;
    private int internalOffset = 0;
    private double maxPower = 0;

    public MotorWithVelocityPID(DcMotorEx motor, PIDCoefficients pid) {
        this(motor, pid, (x, v) -> 0.0);
    }

    public MotorWithVelocityPID(DcMotorEx motor, PIDCoefficients pid, Function2<Double, Double, Double> f) {
        this.motor = motor;
        this.pid = pid;
        this.pidfController = new PIDFController(pid, 0, 0, 0);

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public DcMotorEx getMotor() {
        return motor;
    }

    /**
     * Updates the power sent to the motor according to the pidf controller.
     */
    public void update() {
        double newPower = Range.clip(this.pidfController.update(motor.getVelocity()), -maxPower, maxPower);
        motor.setPower(newPower);
    }

    /**
     * Updates the power sent to the motor according to the pidf controller,
     * but scale pid output with a current and target voltage
     */
    public void update(double currentVoltage, double targetVoltage) {
        double newPower = Range.clip(this.pidfController.update(motor.getVelocity()) * targetVoltage / currentVoltage, -maxPower, maxPower);
        motor.setPower(newPower);
    }

    /**
     * Update the PID values in the controller.
     * Note that it is not possible to replace f after instantiation
     * @param newPID the new pid values to use
     */
    public void setPIDCoefficients(PIDCoefficients newPID) {
        this.pid.kP = newPID.kP;
        this.pid.kI = newPID.kI;
        this.pid.kD = newPID.kD;
    }

    public void setTargetVelocity(int velocity) {
        this.targetVelocity = velocity;
        this.pidfController.setTargetPosition(velocity - internalOffset); // TODO: Verify sign
    }

    private class TargetVelocityAction implements Action {
        int velocity;

        public TargetVelocityAction(int velocity, boolean blocking) {
            this.velocity = velocity;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            setTargetVelocity(velocity);
            return false;
        }
    }

    /**
     * Creates an action that will call setTargetVelocity with the provided velocity
     *
     * @param velocity the desired encoder target velocity
     */
    public Action setTargetVelocityAction(int velocity) {
        return new TargetVelocityAction(velocity, false);
    }

    public void zeroMotorInternals() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetIntegralGain() {
        this.pidfController.reset();
    }

    /**
     * Returns the current target encoder velocity for this motor.
     * @return the current target encoder velocity for this motor.
     */
    public int getTargetVelocity() {
        return targetVelocity;
    }

    /**
     * Returns the current velocity of the motor, in ticks per second
     * @return the current velocity of the motor
     */
    public double getVelocity() {
        return motor.getVelocity();
    }

    /**
     * Sets the maximum power level that can be sent to the motor
     * @param maxPower the maximum level of the motor, a value in the interval [0.0, 1.0]
     */
    public void setMaxPower(double maxPower) {
        this.maxPower = Math.abs(maxPower);
    }

    /**
     * Returns the maximum power level that can be sent to the motor
     * @return the maximum level of the motor, a value in the interval [0.0, 1.0]
     */
    public double getMaxPower() {
        return maxPower;
    }

    /**
     * Returns the current power level sent to the motor.
     * @return the current level of the motor, a value in the interval [-1.0, 1.0]
     */
    public double getPower() {
        return motor.getPower();
    }

    /**
     * Sets the motor power to zero
     */
    public void stopMotor() {
        motor.setPower(0);
    }

    /**
     * Sets the logical direction in which this motor operates.
     * @param direction the direction to set for this motor
     */
    public void setDirection(DcMotorSimple.Direction direction) {
        motor.setDirection(direction);
    }

    public DcMotor.RunMode getMode() {
        return motor.getMode();
    }
}
