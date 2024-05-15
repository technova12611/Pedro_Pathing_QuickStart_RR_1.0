package org.firstinspires.ftc.teamcode.pedroPathing.util.cachinghardware

import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.PIDCoefficients
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit

open class CachingDcMotorEX: CachingDcMotor, DcMotorEx {
    val motorEx: DcMotorEx

    /**
     * Default constructor for the cached motorEx, sets the threshold to 0.005
     *
     * @param motorEx the motor to encapsulate in the caching control
     */
    constructor(motorEx: DcMotorEx) : super(motorEx) {
        this.motorEx = motorEx
    }

    /**
     * Allows an initial setting of a custom changeThreshold
     *
     * @param motorEx the motor to encapsulate in the caching control
     * @param changeThreshold the threshold at which the cache should write new values to the motor
     */
    constructor(motorEx: DcMotorEx, changeThreshold: Double) : super(motorEx, changeThreshold) {
        this.motorEx = motorEx
    }

    /**
     * Individually energizes this particular motor
     *
     * @see .setMotorDisable
     * @see .isMotorEnabled
     */
    override fun setMotorEnable() {
        motorEx.setMotorEnable()
    }

    /**
     * Individually de-energizes this particular motor
     *
     * @see .setMotorEnable
     * @see .isMotorEnabled
     */
    override fun setMotorDisable() {
        motorEx.setMotorDisable()
    }

    /**
     * Returns whether this motor is energized
     *
     * @see .setMotorEnable
     * @see .setMotorDisable
     */
    override fun isMotorEnabled(): Boolean {
        return motorEx.isMotorEnabled
    }

    /**
     * Sets the velocity of the motor
     *
     * @param angularRate the desired ticks per second
     */
    override fun setVelocity(angularRate: Double) {
        motorEx.velocity = angularRate
    }

    /**
     * Sets the velocity of the motor
     *
     * @param angularRate the desired angular rate, in units per second
     * @param unit        the units in which angularRate is expressed
     * @see .getVelocity
     */
    override fun setVelocity(angularRate: Double, unit: AngleUnit) {
        motorEx.setVelocity(angularRate, unit)
    }

    /**
     * Returns the current velocity of the motor, in ticks per second
     *
     * @return the current velocity of the motor
     */
    override fun getVelocity(): Double {
        return motorEx.velocity
    }

    /**
     * Returns the current velocity of the motor, in angular units per second
     *
     * @param unit the units in which the angular rate is desired
     * @return the current velocity of the motor
     * @see .setVelocity
     */
    override fun getVelocity(unit: AngleUnit): Double {
        return motorEx.getVelocity(unit)
    }

    /**
     * Sets the PID control coefficients for one of the PID modes of this motor.
     * Note that in some controller implementations, setting the PID coefficients for one
     * mode on a motor might affect other modes on that motor, or might affect the PID
     * coefficients used by other motors on the same controller (this is not true on the
     * REV Expansion Hub).
     *
     * @param mode            either [RunMode.RUN_USING_ENCODER] or [RunMode.RUN_TO_POSITION]
     * @param pidCoefficients the new coefficients to use when in that mode on this motor
     * @see .getPIDCoefficients
     */
    @Deprecated("Use {@link #setPIDFCoefficients(RunMode, PIDFCoefficients)} instead", ReplaceWith("motorEx.setPIDCoefficients(mode, pidCoefficients)"))
    override fun setPIDCoefficients(mode: RunMode, pidCoefficients: PIDCoefficients) {
        motorEx.setPIDCoefficients(mode, pidCoefficients)
    }

    /**
     * [.setPIDFCoefficients] is a superset enhancement to [.setPIDCoefficients]. In addition
     * to the proportional, integral, and derivative coefficients previously supported, a feed-forward
     * coefficient may also be specified. Further, a selection of motor control algorithms is offered:
     * the originally-shipped Legacy PID algorithm, and a PIDF algorithm which avails itself of the
     * feed-forward coefficient. Note that the feed-forward coefficient is not used by the Legacy PID
     * algorithm; thus, the feed-forward coefficient must be indicated as zero if the Legacy PID
     * algorithm is used. Also: the internal implementation of these algorithms may be different: it
     * is not the case that the use of PIDF with the F term as zero necessarily exhibits exactly the
     * same behavior as the use of the LegacyPID algorithm, though in practice they will be quite close.
     *
     *
     * Readers are reminded that [DcMotor.RunMode.RUN_TO_POSITION] mode makes use of *both*
     * the coefficients set for RUN_TO_POSITION *and* the coefficients set for RUN_WITH_ENCODER,
     * due to the fact that internally the RUN_TO_POSITION logic calculates an on-the-fly velocity goal
     * on each control cycle, then (logically) runs the RUN_WITH_ENCODER logic. Because of that double-
     * layering, only the proportional ('p') coefficient makes logical sense for use in the RUN_TO_POSITION
     * coefficients.
     *
     * @param mode
     * @param pidfCoefficients
     * @see .setVelocityPIDFCoefficients
     * @see .setPositionPIDFCoefficients
     * @see .getPIDFCoefficients
     */
    @Throws(UnsupportedOperationException::class)
    override fun setPIDFCoefficients(mode: RunMode, pidfCoefficients: PIDFCoefficients) {
        motorEx.setPIDFCoefficients(mode, pidfCoefficients)
    }

    /**
     * A shorthand for setting the PIDF coefficients for the [DcMotor.RunMode.RUN_USING_ENCODER]
     * mode. [MotorControlAlgorithm.PIDF] is used.
     *
     * @param p
     * @param i
     * @param d
     * @param f
     * @see .setPIDFCoefficients
     */
    override fun setVelocityPIDFCoefficients(p: Double, i: Double, d: Double, f: Double) {
        motorEx.setVelocityPIDFCoefficients(p, i, d, f)
    }

    /**
     * A shorthand for setting the PIDF coefficients for the [DcMotor.RunMode.RUN_TO_POSITION]
     * mode. [MotorControlAlgorithm.PIDF] is used.
     *
     *
     * Readers are reminded that [DcMotor.RunMode.RUN_TO_POSITION] mode makes use of *both*
     * the coefficients set for RUN_TO_POSITION *and* the coefficients set for RUN_WITH_ENCODER,
     * due to the fact that internally the RUN_TO_POSITION logic calculates an on-the-fly velocity goal
     * on each control cycle, then (logically) runs the RUN_WITH_ENCODER logic. Because of that double-
     * layering, only the proportional ('p') coefficient makes logical sense for use in the RUN_TO_POSITION
     * coefficients.
     *
     * @param p
     * @see .setVelocityPIDFCoefficients
     * @see .setPIDFCoefficients
     */
    override fun setPositionPIDFCoefficients(p: Double) {
        motorEx.setPositionPIDFCoefficients(p)
    }

    /**
     * Returns the PID control coefficients used when running in the indicated mode
     * on this motor.
     *
     * @param mode either [RunMode.RUN_USING_ENCODER] or [RunMode.RUN_TO_POSITION]
     * @return the PID control coefficients used when running in the indicated mode on this motor
     */
    @Deprecated("Use {@link #getPIDFCoefficients(RunMode)} instead", ReplaceWith("motorEx.getPIDCoefficients(mode)"))
    override fun getPIDCoefficients(mode: RunMode): PIDCoefficients {
        return motorEx.getPIDCoefficients(mode)
    }

    /**
     * Returns the PIDF control coefficients used when running in the indicated mode
     * on this motor.
     *
     * @param mode either [RunMode.RUN_USING_ENCODER] or [RunMode.RUN_TO_POSITION]
     * @return the PIDF control coefficients used when running in the indicated mode on this motor
     * @see .setPIDFCoefficients
     */
    override fun getPIDFCoefficients(mode: RunMode): PIDFCoefficients {
        return motorEx.getPIDFCoefficients(mode)
    }

    /**
     * Sets the target positioning tolerance of this motor
     *
     * @param tolerance the desired tolerance, in encoder ticks
     * @see DcMotor.setTargetPosition
     */
    override fun setTargetPositionTolerance(tolerance: Int) {
        motorEx.targetPositionTolerance = tolerance
    }

    /**
     * Returns the current target positioning tolerance of this motor
     *
     * @return the current target positioning tolerance of this motor
     */
    override fun getTargetPositionTolerance(): Int {
        return motorEx.targetPositionTolerance
    }

    /**
     * Returns the current consumed by this motor.
     *
     * @param unit current units
     * @return the current consumed by this motor.
     */
    override fun getCurrent(unit: CurrentUnit): Double {
        return motorEx.getCurrent(unit)
    }

    /**
     * Returns the current alert for this motor.
     *
     * @param unit current units
     * @return the current alert for this motor
     */
    override fun getCurrentAlert(unit: CurrentUnit): Double {
        return motorEx.getCurrentAlert(unit)
    }

    /**
     * Sets the current alert for this motor
     *
     * @param current current alert
     * @param unit    current units
     */
    override fun setCurrentAlert(current: Double, unit: CurrentUnit) {
        motorEx.setCurrentAlert(current, unit)
    }

    /**
     * Returns whether the current consumption of this motor exceeds the alert threshold.
     *
     * @return whether the current consumption of this motor exceeds the alert threshold.
     */
    override fun isOverCurrent(): Boolean {
        return motorEx.isOverCurrent
    }
}