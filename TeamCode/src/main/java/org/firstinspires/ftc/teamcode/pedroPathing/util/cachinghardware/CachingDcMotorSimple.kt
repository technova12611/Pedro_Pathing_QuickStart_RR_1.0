package org.firstinspires.ftc.teamcode.pedroPathing.util.cachinghardware

import android.util.Log
import com.qualcomm.robotcore.hardware.DcMotorSimple
import kotlin.math.abs

open class CachingDcMotorSimple
/**
 * Default constructor for the cached simple motor, sets the threshold to 0.02
 *
 * @param dcMotorSimple the simple motor to encapsulate in the caching control
 * @param cachingTolerance the new change threshold at which the motor will be written to.
 */ @JvmOverloads constructor(val dcMotorSimple: DcMotorSimple,
                              var cachingTolerance: Double = 0.005) : CachingHardwareDevice(dcMotorSimple), DcMotorSimple {
    private var cachedPower = 0.0
    /**
     * returns the current changeThreshold value
     *
     * @return the current changeThreshold value, defaults to 0.02
     */
    /**
     * Allows an initial setting of a custom changeThreshold
     *
     * @param dcMotorSimple the simple motor to encapsulate in the caching control
     * @param cachingTolerance the threshold at which the cache should write new values to the motorSimple
     */
    /**
     * Sets the logical direction in which this motor operates.
     *
     * @param direction the direction to set for this motor
     * @see .getDirection
     */
    override fun setDirection(direction: DcMotorSimple.Direction) {
        dcMotorSimple.direction = direction
    }

    /**
     * Returns the current logical direction in which this motor is set as operating.
     *
     * @return the current logical direction in which this motor is set as operating.
     * @see .setDirection
     */
    override fun getDirection(): DcMotorSimple.Direction {
        return dcMotorSimple.direction
    }

    /**
     * Checks if the change in power output exceeds the set change threshold, if so, does a hardware write
     * @see com.qualcomm.robotcore.hardware.DcMotor.setPower
     * @param power the new power level of the motor, a value in the interval [-1.0, 1.0]
     */
    override fun setPower(power: Double) {
        // will accept the input if it is targeting 0, or full power in any direction, or if it has changed a sufficient amount
        if (abs(power - cachedPower) >= cachingTolerance || (power == 0.0 && cachedPower != 0.0) || (power >= 1.0 && !(cachedPower >= 1.0)) || (power <= -1.0 && !(cachedPower <= -1.0))) {
            cachedPower = power
            dcMotorSimple.power = power
            //Log.d("DcMotor_logger", "setPower: " + String.format("%3.3f", power))
        }
    }

    /**
     * Checks if the change in power output exceeds the set change threshold, if so, does a hardware write
     * @see setPower
     * @see com.qualcomm.robotcore.hardware.DcMotor.setPower
     * @param power the new power level of the motor, a value in the interval [-1.0, 1.0]
     * @return if a hardware write to update the output to the motor was executed
     */
    fun setPowerResult(power: Double): Boolean {
        // will accept the input if it is targeting 0, or full power in any direction, or if it has changed a sufficient amount
        if (abs(power - cachedPower) >= cachingTolerance || (power == 0.0 && cachedPower != 0.0) || (power >= 1.0 && !(cachedPower >= 1.0)) || (power <= -1.0 && !(cachedPower <= -1.0))) {
            cachedPower = power
            dcMotorSimple.power = power

            //Log.d("DcMotor_logger", "setPowerResult: " + String.format("%3.3f", power))
            return true
        }
        return false
    }

    /**
     * ignores the caching tolerance
     */
    fun setPowerRaw(power: Double): Boolean {
        val cachingTolerance = this.cachingTolerance
        this.cachingTolerance = 0.0
        val res = setPowerResult(power)
        this.cachingTolerance = cachingTolerance
        return res
    }

    /**
     * Returns the current configured power level of the motor.
     *
     * @return the current level of the motor, a value in the interval [0.0, 1.0]
     * @see .setPower
     */
    override fun getPower(): Double {
        return dcMotorSimple.power
    }
}