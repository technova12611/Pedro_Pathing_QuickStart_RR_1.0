package org.firstinspires.ftc.teamcode.pedroPathing.util.cachinghardware

import android.util.Log
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoController
import kotlin.math.abs

/**
 * Sets the difference between the previously written value and the new value for position before the caching control will write the new value.
 *
 * @param cachingTolerance the new change threshold at which the motor will be written to.
 */
open class CachingServo @JvmOverloads constructor(val servo: Servo,
                                                  var cachingTolerance: Double = 0.001) : CachingHardwareDevice(servo), Servo {
    private var cachedPosition: Double
    /**
     * returns the current changeThreshold value
     *
     * @return the current changeThreshold value, defaults to 0.01
     */
    /**
     * Allows an initial setting of a custom changeThreshold
     *
     * @param servo the servo to encapsulate in the caching control
     * @param cachingTolerance the threshold at which the cache should write new values to the servo
     */
    /**
     * Default constructor for the cached servo, sets the threshold to 0.01
     *
     * @param servo the servo to encapsulate in the caching control
     */
    init {
        cachedPosition = Double.NaN
    }

    /**
     * Returns the underlying servo controller on which this servo is situated.
     *
     * @return the underlying servo controller on which this servo is situated.
     * @see .getPortNumber
     */
    override fun getController(): ServoController {
        return servo.controller
    }

    /**
     * Returns the port number on the underlying servo controller on which this motor is situated.
     *
     * @return the port number on the underlying servo controller on which this motor is situated.
     * @see .getController
     */
    override fun getPortNumber(): Int {
        return servo.portNumber
    }

    /**
     * Sets the logical direction in which this servo operates.
     *
     * @param direction the direction to set for this servo
     * @see .getDirection
     * @see Direction
     */
    override fun setDirection(direction: Servo.Direction) {
        servo.direction = direction
    }

    /**
     * Returns the current logical direction in which this servo is set as operating.
     *
     * @return the current logical direction in which this servo is set as operating.
     * @see .setDirection
     */
    override fun getDirection(): Servo.Direction {
        return servo.direction
    }

    /**
     * Sets the current position of the servo, expressed as a fraction of its available
     * range. If PWM power is enabled for the servo, the servo will attempt to move to
     * the indicated position.
     *
     * @param position the position to which the servo should move, a value in the range [0.0, 1.0]
     * @see ServoController.pwmEnable
     * @see .getPosition
     */
    override fun setPosition(position: Double) {
        //will accept inputs of both 0.0 and 1.0 so that the controller can always hit the extremes.
        if (cachedPosition.isNaN() || abs(cachedPosition - position) >= cachingTolerance || (position <= 0.0 && cachedPosition > 0.0) || (position >= 1.0 && cachedPosition < 1.0)) {
            servo.position = position
            cachedPosition = position

            Log.d("Servo_logger", "setPosition: " + String.format("%3.3f", position))
        }
    }

    /**
     * Returns the position to which the servo was last commanded to move. Note that this method
     * does NOT read a position from the servo through any electrical means, as no such electrical
     * mechanism is, generally, available.
     *
     * @return the position to which the servo was last commanded to move, or Double.NaN
     * if no such position is known
     * @see .setPosition
     * @see Double.NaN
     *
     * @see Double.isNaN
     */
    override fun getPosition(): Double {
        Log.d("Servo_logger", "getPosition: " + String.format("%3.3f", cachedPosition))
        return servo.position
    }

    /**
     * Scales the available movement range of the servo to be a subset of its maximum range. Subsequent
     * positioning calls will operate within that subset range. This is useful if your servo has
     * only a limited useful range of movement due to the physical hardware that it is manipulating
     * (as is often the case) but you don't want to have to manually scale and adjust the input
     * to [setPosition()][.setPosition] each time.
     *
     *
     * For example, if scaleRange(0.2, 0.8) is set; then servo positions will be
     * scaled to fit in that range:<br></br>
     * setPosition(0.0) scales to 0.2<br></br>
     * setPosition(1.0) scales to 0.8<br></br>
     * setPosition(0.5) scales to 0.5<br></br>
     * setPosition(0.25) scales to 0.35<br></br>
     * setPosition(0.75) scales to 0.65<br></br>
     *
     *
     *
     * Note the parameters passed here are relative to the underlying full range of motion of
     * the servo, not its currently scaled range, if any. Thus, scaleRange(0.0, 1.0) will reset
     * the servo to its full range of movement.
     *
     * @param min the lower limit of the servo movement range, a value in the interval [0.0, 1.0]
     * @param max the upper limit of the servo movement range, a value in the interval [0.0, 1.0]
     * @see .setPosition
     */
    override fun scaleRange(min: Double, max: Double) {
        servo.scaleRange(min, max)
    }
}