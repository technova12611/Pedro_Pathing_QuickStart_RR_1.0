package org.firstinspires.ftc.teamcode.pedroPathing.util.cachinghardware

import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.ServoController

class CachingCRServo
/**
 * Default constructor for the cached continuous rotation servo, sets the threshold to 0.02
 *
 * @param CRServo the continuous rotation servo to encapsulate in the caching control
 */ @JvmOverloads constructor(val crServo: CRServo, changeThreshold: Double = 0.005) : CachingDcMotorSimple(crServo, changeThreshold), CRServo {
    /**
     * Allows an initial setting of a custom changeThreshold
     *
     * @param crServo the continuous rotation servo to encapsulate in the caching control
     * @param changeThreshold the threshold at which the cache should write new values to the continuous rotation servo
     */
    /**
     * Returns the underlying servo controller on which this servo is situated.
     *
     * @return the underlying servo controller on which this servo is situated.
     * @see .getPortNumber
     */
    override fun getController(): ServoController {
        return crServo.controller
    }

    /**
     * Returns the port number on the underlying servo controller on which this motor is situated.
     *
     * @return the port number on the underlying servo controller on which this motor is situated.
     * @see .getController
     */
    override fun getPortNumber(): Int {
        return crServo.portNumber
    }
}