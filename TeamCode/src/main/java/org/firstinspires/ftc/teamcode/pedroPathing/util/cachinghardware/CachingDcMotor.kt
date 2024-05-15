package org.firstinspires.ftc.teamcode.pedroPathing.util.cachinghardware

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior
import com.qualcomm.robotcore.hardware.DcMotorController
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType
import kotlin.math.abs

open class CachingDcMotor : CachingDcMotorSimple, DcMotor {
    val dcMotor: DcMotor
    private var cachedTargetPosition: Double

    /**
     * Default constructor for the cached motor, sets the threshold to 0.02
     *
     * @param motor the motor to encapsulate in the caching control
     */
    constructor(motor: DcMotor) : super(motor) {
        cachedTargetPosition = 0.0
        dcMotor = motor
    }

    /**
     * Allows an initial setting of a custom changeThreshold
     *
     * @param motor the motor to encapsulate in the caching control
     * @param changeThreshold the threshold at which the cache should write new values to the motor
     */
    constructor(motor: DcMotor, changeThreshold: Double) : super(motor, changeThreshold) {
        cachedTargetPosition = 0.0
        dcMotor = motor
    }

    /**
     * Returns the assigned type for this motor. If no particular motor type has been
     * configured, then [MotorConfigurationType.getUnspecifiedMotorType] will be returned.
     * Note that the motor type for a given motor is initially assigned in the robot
     * configuration user interface, though it may subsequently be modified using methods herein.
     *
     * @return the assigned type for this motor
     */
    override fun getMotorType(): MotorConfigurationType {
        return dcMotor.motorType
    }

    /**
     * Sets the assigned type of this motor. Usage of this method is very rare.
     *
     * @param motorType the new assigned type for this motor
     * @see .getMotorType
     */
    override fun setMotorType(motorType: MotorConfigurationType) {
        dcMotor.motorType = motorType
    }

    /**
     * Returns the underlying motor controller on which this motor is situated.
     *
     * @return the underlying motor controller on which this motor is situated.
     * @see .getPortNumber
     */
    override fun getController(): DcMotorController {
        return dcMotor.controller
    }

    /**
     * Returns the port number on the underlying motor controller on which this motor is situated.
     *
     * @return the port number on the underlying motor controller on which this motor is situated.
     * @see .getController
     */
    override fun getPortNumber(): Int {
        return dcMotor.portNumber
    }

    /**
     * Sets the behavior of the motor when a power level of zero is applied.
     *
     * @param zeroPowerBehavior the new behavior of the motor when a power level of zero is applied.
     * @see ZeroPowerBehavior
     *
     * @see .setPower
     */
    override fun setZeroPowerBehavior(zeroPowerBehavior: ZeroPowerBehavior) {
        dcMotor.zeroPowerBehavior = zeroPowerBehavior
    }

    /**
     * Returns the current behavior of the motor were a power level of zero to be applied.
     *
     * @return the current behavior of the motor were a power level of zero to be applied.
     */
    override fun getZeroPowerBehavior(): ZeroPowerBehavior {
        return dcMotor.zeroPowerBehavior
    }

    /**
     * Sets the zero power behavior of the motor to [FLOAT][ZeroPowerBehavior.FLOAT], then
     * applies zero power to that motor.
     *
     *
     * Note that the change of the zero power behavior to [FLOAT][ZeroPowerBehavior.FLOAT]
     * remains in effect even following the return of this method. <STRONG>This is a breaking
     * change</STRONG> in behavior from previous releases of the SDK. Consider, for example, the
     * following code sequence:
     *
     * <pre>
     * motor.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE); // method not available in previous releases
     * motor.setPowerFloat();
     * motor.setPower(0.0);
    </pre> *
     *
     *
     * Starting from this release, this sequence of code will leave the motor floating. Previously,
     * the motor would have been left braked.
     *
     * @see .setPower
     * @see .getPowerFloat
     * @see .setZeroPowerBehavior
     */
    @Deprecated("""This method is deprecated in favor of direct use of
	  {@link #setZeroPowerBehavior(ZeroPowerBehavior) setZeroPowerBehavior()} and
	  {@link #setPower(double) setPower()}.""", ReplaceWith("dcMotor.setPowerFloat()"))
    override fun setPowerFloat() {
        dcMotor.setPowerFloat()
    }

    /**
     * Returns whether the motor is currently in a float power level.
     *
     * @return whether the motor is currently in a float power level.
     * @see .setPowerFloat
     */
    @Deprecated("")
    override fun getPowerFloat(): Boolean {
        return dcMotor.powerFloat
    }

    /**
     * Sets the desired encoder target position to which the motor should advance or retreat
     * and then actively hold thereat. This behavior is similar to the operation of a servo.
     * The maximum speed at which this advance or retreat occurs is governed by the power level
     * currently set on the motor. While the motor is advancing or retreating to the desired
     * taget position, [.isBusy] will return true.
     *
     *
     * Note that adjustment to a target position is only effective when the motor is in
     * [RUN_TO_POSITION][RunMode.RUN_TO_POSITION]
     * RunMode. Note further that, clearly, the motor must be equipped with an encoder in order
     * for this mode to function properly.
     *
     * @param position the desired encoder target position
     * @see .getCurrentPosition
     * @see .setMode
     * @see RunMode.RUN_TO_POSITION
     *
     * @see .getTargetPosition
     * @see .isBusy
     */
    override fun setTargetPosition(position: Int) {
        if (abs(cachedTargetPosition - position) >= cachingTolerance) {
            dcMotor.targetPosition = position
            cachedTargetPosition = position.toDouble()
        }
    }

    /**
     * Returns the current target encoder position for this motor.
     *
     * @return the current target encoder position for this motor.
     * @see .setTargetPosition
     */
    override fun getTargetPosition(): Int {
        return dcMotor.targetPosition
    }

    /**
     * Returns true if the motor is currently advancing or retreating to a target position.
     *
     * @return true if the motor is currently advancing or retreating to a target position.
     * @see .setTargetPosition
     */
    override fun isBusy(): Boolean {
        return dcMotor.isBusy
    }

    /**
     * Returns the current reading of the encoder for this motor. The units for this reading,
     * that is, the number of ticks per revolution, are specific to the motor/encoder in question,
     * and thus are not specified here.
     *
     * @return the current reading of the encoder for this motor
     * @see .getTargetPosition
     * @see RunMode.STOP_AND_RESET_ENCODER
     */
    override fun getCurrentPosition(): Int {
        return dcMotor.currentPosition
    }

    /**
     * Sets the current run mode for this motor
     *
     * @param mode the new current run mode for this motor
     * @see RunMode
     *
     * @see .getMode
     */
    override fun setMode(mode: RunMode) {
        dcMotor.mode = mode
    }

    /**
     * Returns the current run mode for this motor
     *
     * @return the current run mode for this motor
     * @see RunMode
     *
     * @see .setMode
     */
    override fun getMode(): RunMode {
        return dcMotor.mode
    }
}