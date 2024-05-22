package org.firstinspires.ftc.teamcode.pedroPathing.localization;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Twist2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Vector;
import com.acmerobotics.roadrunner.ftc.LazyImu;

/**
 * This is the PoseUpdater class. This class handles getting pose data from the localizer and returning
 * the information in a useful way to the Follower.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/4/2024
 */
public class PoseUpdater {
    private HardwareMap hardwareMap;

    private IMU imu;

    private Localizer localizer;

    private Pose2d startingPose = new Pose2d(0,0,0);

    private Pose2d currentPose = startingPose;

    private Pose2d previousPose = startingPose;

    private Vector currentVelocity = new Vector();

    private Vector previousVelocity = new Vector();

    private Vector currentAcceleration = new Vector();

    private double xOffset = 0;
    private double yOffset = 0;
    private double headingOffset = 0;

    private long previousPoseTime;
    private long currentPoseTime;

    public final boolean useRRLocalizer = true;

    /**
     * Creates a new PoseUpdater from a HardwareMap.
     *
     * @param hardwareMap the HardwareMap
     */
    public PoseUpdater(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        if (useRRLocalizer) {
            localizer = new ThreeDeadWheelLocalizer(hardwareMap);
        } else {
            localizer = new ThreeWheelLocalizer(hardwareMap);
        }
    }

    /**
     * This updates the robot's pose, as well as updating the previous pose, velocity, and
     * acceleration. The cache for the current pose, velocity, and acceleration is cleared, and
     * the time stamps are updated as well.
     */
    public void update() {
        previousVelocity = getVelocity();
        previousPose = applyOffset(getRawPose());
        currentPose = null;
        currentVelocity = null;
        currentAcceleration = null;
        previousPoseTime = currentPoseTime;
        currentPoseTime = System.nanoTime();
        localizer.update();
    }

    /**
     * This sets the starting pose. Do not run this after moving at all.
     *
     * @param set the Pose to set the starting pose to.
     */
    public void setStartingPose(Pose2d set) {
        startingPose = set;
        previousPose = startingPose;
        previousPoseTime = System.nanoTime();
        currentPoseTime = System.nanoTime();
        localizer.setStartPose(set);
    }

    /**
     * This sets the current pose, using offsets so no reset time delay. This is better than the
     * Road Runner reset, in general. Think of using offsets as setting trim in an aircraft. This can
     * be reset as well, so beware of using the resetOffset() method.
     *
     * @param set The pose to set the current pose to.
     */
    public void setCurrentPoseWithOffset(Pose2d set) {
        Pose2d currentPose = getRawPose();
        setXOffset(set.position.x - currentPose.position.x);
        setYOffset(set.position.y - currentPose.position.y);
        setHeadingOffset(MathFunctions.getTurnDirection(currentPose.heading.toDouble(), set.heading.toDouble()) * MathFunctions.getSmallestAngleDifference(currentPose.heading.toDouble(), set.heading.toDouble()));
    }

    /**
     * This sets the offset for only the x position.
     *
     * @param offset This sets the offset.
     */
    public void setXOffset(double offset) {
        xOffset = offset;
    }

    /**
     * This sets the offset for only the y position.
     *
     * @param offset This sets the offset.
     */
    public void setYOffset(double offset) {
        yOffset = offset;
    }

    /**
     * This sets the offset for only the heading.
     *
     * @param offset This sets the offset.
     */
    public void setHeadingOffset(double offset) {
        headingOffset = offset;
    }

    /**
     * This returns the x offset.
     *
     * @return returns the x offset.
     */
    public double getXOffset() {
        return xOffset;
    }

    /**
     * This returns the y offset.
     *
     * @return returns the y offset.
     */
    public double getYOffset() {
        return yOffset;
    }

    /**
     * This returns the heading offset.
     *
     * @return returns the heading offset.
     */
    public double getHeadingOffset() {
        return headingOffset;
    }

    /**
     * This applies the offset to a specified Pose.
     *
     * @param pose The pose to be offset.
     * @return This returns a new Pose with the offset applied.
     */
    public Pose2d applyOffset(Pose2d pose) {
        return new Pose2d(pose.position.x+xOffset, pose.position.y+yOffset, pose.heading.toDouble()+headingOffset);
    }

    /**
     * This resets all offsets set to the PoseUpdater. If you have reset your pose using the
     * setCurrentPoseUsingOffset(Pose2d set) method, then your pose will be returned to what the
     * PoseUpdater thinks your pose would be, not the pose you reset to.
     */
    public void resetOffset() {
        setXOffset(0);
        setYOffset(0);
        setHeadingOffset(0);
    }

    /**
     * This returns the current pose, with offsets applied. If this is called multiple times in
     * a single update, the current pose is cached so that subsequent calls don't have to repeat
     * localizer calls or calculations.
     *
     * @return returns the current pose.
     */
    public Pose2d getPose() {
        if (currentPose == null) {
            currentPose = localizer.getPoseEstimate();
            return applyOffset(currentPose);
        } else {
            return applyOffset(currentPose);
        }
    }

    /**
     * This returns the current raw pose, without any offsets applied. If this is called multiple times in
     * a single update, the current pose is cached so that subsequent calls don't have to repeat
     * localizer calls or calculations.
     *
     * @return returns the raw pose.
     */
    public Pose2d getRawPose() {
        if (currentPose == null) {
            currentPose = localizer.getPoseEstimate();
            return currentPose;
        } else {
            return currentPose;
        }
    }

    /**
     * This sets the current pose using the Road Runner pose reset. This is slow.
     *
     * @param set the pose to set the current pose to.
     */
    public void setPose(Pose2d set) {
        resetOffset();
        localizer.setPoseEstimate(set);
    }

    /**
     * Returns the robot's pose from the previous update.
     *
     * @return returns the robot's previous pose.
     */
    public Pose2d getPreviousPose() {
        return previousPose;
    }

    /**
     * Returns the robot's change in pose from the previous update.
     *
     * @return returns the robot's delta pose.
     */
    public Pose2d getDeltaPose() {
        Twist2d delta = getPose().minus(previousPose);
        return new Pose2d(delta.line, delta.angle);
    }

    /**
     * This returns the velocity of the robot as a Vector. If this is called multiple times in
     * a single update, the velocity Vector is cached so that subsequent calls don't have to repeat
     * localizer calls or calculations.
     *
     * @return returns the velocity of the robot.
     */
    public Vector getVelocity() {
        if (currentVelocity == null) {
            currentVelocity = new Vector();
            currentVelocity.setOrthogonalComponents(getPose().position.x - previousPose.position.x, getPose().position.y - previousPose.position.y);
            currentVelocity.setMagnitude(MathFunctions.distance(
                    getPose(),previousPose) / ((currentPoseTime - previousPoseTime) / Math.pow(10.0, 9)));
            return MathFunctions.copyVector(currentVelocity);
        } else {
            return MathFunctions.copyVector(currentVelocity);
        }
    }

    /**
     * This returns the angular velocity of the robot as a double.
     *
     * @return returns the angular velocity of the robot.
     */
    public double getAngularVelocity() {
        return MathFunctions.getTurnDirection(previousPose.heading.toDouble(), getPose().heading.toDouble()) * MathFunctions.getSmallestAngleDifference(getPose().heading.toDouble(), previousPose.heading.toDouble()) / ((currentPoseTime-previousPoseTime)/Math.pow(10.0, 9));
    }

    /**
     * This returns the acceleration of the robot as a Vector. If this is called multiple times in
     * a single update, the acceleration Vector is cached so that subsequent calls don't have to
     * repeat localizer calls or calculations.
     *
     * @return returns the acceleration of the robot.
     */
    public Vector getAcceleration() {
        if (currentAcceleration == null) {
            currentAcceleration = MathFunctions.subtractVectors(getVelocity(), previousVelocity);
            currentAcceleration.setMagnitude(currentAcceleration.getMagnitude() / ((currentPoseTime - previousPoseTime) / Math.pow(10.0, 9)));
            return MathFunctions.copyVector(currentAcceleration);
        } else {
            return MathFunctions.copyVector(currentAcceleration);
        }
    }

    /**
     * This resets the heading of the robot to the IMU's heading, using offsets instead of Road
     * Runner's pose reset. This way, it's faster, but this can be wiped with the resetOffsets()
     * method.
     */
    public void resetHeadingToIMUWithOffsets() {
        setCurrentPoseWithOffset(new Pose2d(getPose().position.x, getPose().position.y, getNormalizedIMUHeading() + startingPose.heading.toDouble()));
    }

    /**
     * This returns the IMU heading normalized to be between [0, 2 PI] radians.
     *
     * @return returns the normalized IMU heading.
     */
    public double getNormalizedIMUHeading() {
        return MathFunctions.normalizeAngle(-imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
    }

    /**
     * This returns the Localizer.
     *
     * @return the Localizer
     */
    public Localizer getLocalizer() {
        return localizer;
    }
}
