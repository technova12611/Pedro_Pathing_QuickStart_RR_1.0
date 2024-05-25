package org.firstinspires.ftc.teamcode.pedroPathing.localization;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Vector;
import org.firstinspires.ftc.teamcode.pedroPathing.util.NanoTimer;

/**
 * This is the ThreeWheelLocalizer class. This class extends the Localizer superclass and is a
 * localizer that uses the three wheel odometry set up. The diagram below, which is taken from
 * Road Runner, shows a typical set up.
 *
 * The view is from the bottom of the robot looking upwards.
 *
 * left on robot is y pos
 *
 * front on robot is x pos
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |   left (y pos)
 *    |              |
 *    |              |
 *    \--------------/
 *      front (x pos)
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @version 1.0, 4/2/2024
 */
@Config
public class ThreeWheelLocalizer extends Localizer {
    private HardwareMap hardwareMap;
    private Pose startPose;
    private Pose displacementPose;
    private Pose currentVelocity;
    private Matrix prevRotationMatrix;
    private NanoTimer timer;
    private long deltaTimeNano;
    private Encoder leftEncoder;
    private Encoder rightEncoder;
    private Encoder strafeEncoder;
    private Pose leftEncoderPose;
    private Pose rightEncoderPose;
    private Pose strafeEncoderPose;
    private double
            totalHeading;
    public static double FORWARD_TICKS_TO_INCHES = 0.002948;//8192 * 1.37795 * 2 * Math.PI * 0.5008239963;
    public static double STRAFE_TICKS_TO_INCHES = 0.00302;//8192 * 1.37795 * 2 * Math.PI * 0.5018874659;
    public static double TURN_TICKS_TO_RADIANS = 0.002948;//8192 * 1.37795 * 2 * Math.PI * 0.5;

    private IMU imu;

    /**
     * This creates a new ThreeWheelLocalizer from a HardwareMap, with a starting Pose at (0,0)
     * facing 0 heading.
     *
     * @param map the HardwareMap
     */
    public ThreeWheelLocalizer(HardwareMap map) {
        this(map, new Pose());
    }

    /**
     * This creates a new ThreeWheelLocalizer from a HardwareMap and a Pose, with the Pose
     * specifying the starting pose of the localizer.
     *
     * @param map the HardwareMap
     * @param setStartPose the Pose to start from
     */
    public ThreeWheelLocalizer(HardwareMap map, Pose setStartPose) {
        // TODO: replace these with your encoder positions
        leftEncoderPose = new Pose(0, 7.31, 0);
        rightEncoderPose = new Pose(0, -7.31, 0);
        strafeEncoderPose = new Pose(-0.905, 0, Math.toRadians(90));

        hardwareMap = map;

        // TODO: replace these with your encoder ports
        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "par"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightBack"));
        strafeEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "intake_for_perp"));

        // TODO: reverse any encoders necessary
        leftEncoder.setDirection(Encoder.REVERSE);
        rightEncoder.setDirection(Encoder.REVERSE);
        strafeEncoder.setDirection(Encoder.FORWARD);

        setStartPose(setStartPose);
        timer = new NanoTimer();
        deltaTimeNano = 1;
        displacementPose = new Pose();
        currentVelocity = new Pose();
        totalHeading = 0;

        resetEncoders();

        LazyImu lazyImu;
        lazyImu = new LazyImu(hardwareMap, "imu", new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu = lazyImu.get();
        imu.resetYaw();
    }

    /**
     * This returns the current pose estimate.
     *
     * @return returns the current pose estimate as a Pose
     */
    @Override
    public Pose getPose() {
        return MathFunctions.addPoses(startPose, displacementPose);
    }

    public Pose2d getPoseEstimate() {
        Pose currentPose = getPose();
        return new Pose2d(currentPose.getX(), currentPose.getY(), currentPose.getHeading());
    }

    /**
     * This returns the current velocity estimate.
     *
     * @return returns the current velocity estimate as a Pose
     */
    @Override
    public Pose getVelocity() {
        return currentVelocity.copy();
    }

    /**
     * This returns the current velocity estimate.
     *
     * @return returns the current velocity estimate as a Vector
     */
    @Override
    public Vector getVelocityVector() {
        return currentVelocity.getVector();
    }

    /**
     * This sets the start pose. Changing the start pose should move the robot as if all its
     * previous movements were displacing it from its new start pose.
     *
     * @param setStart the new start pose
     */
    @Override
    public void setStartPose(Pose setStart) {
        startPose = setStart;
    }

    public void setStartPose(Pose2d setStart) {
        startPose = new Pose(setStart.position.x,setStart.position.y, setStart.heading.toDouble());
    }

    /**
     * This sets the Matrix that contains the previous pose's heading rotation.
     *
     * @param heading the rotation of the Matrix
     */
    public void setPrevRotationMatrix(double heading) {
        prevRotationMatrix = new Matrix(3,3);
        prevRotationMatrix.set(0, 0, Math.cos(heading));
        prevRotationMatrix.set(0, 1, -Math.sin(heading));
        prevRotationMatrix.set(1, 0, Math.sin(heading));
        prevRotationMatrix.set(1, 1, Math.cos(heading));
        prevRotationMatrix.set(2, 2, 1.0);
    }

    /**
     * This sets the current pose estimate. Changing this should just change the robot's current
     * pose estimate, not anything to do with the start pose.
     *
     * @param setPose the new current pose estimate
     */
    @Override
    public void setPose(Pose setPose) {
        displacementPose = MathFunctions.subtractPoses(setPose, startPose);
        resetEncoders();
    }

    public void setPoseEstimate(Pose2d setPose) {
        setPose(new Pose(setPose.position.x, setPose.position.y, setPose.heading.toDouble()));
    }

    /**
     * This updates the elapsed time timer that keeps track of time between updates, as well as the
     * change position of the Encoders. Then, the robot's global change in position is calculated
     * using the pose exponential method.
     */
    @Override
    public void update() {
        deltaTimeNano = timer.getElapsedTime();
        timer.resetTimer();

        updateEncoders();
        Matrix robotDeltas = getRobotDeltas();
        Matrix globalDeltas;
        setPrevRotationMatrix(getPose().getHeading());

        Matrix transformation = new Matrix(3,3);
        if (Math.abs(robotDeltas.get(2, 0)) < 0.001) {
            transformation.set(0, 0, 1.0 - (Math.pow(robotDeltas.get(2, 0), 2) / 6.0));
            transformation.set(0, 1, -robotDeltas.get(2, 0) / 2.0);
            transformation.set(1, 0, robotDeltas.get(2, 0) / 2.0);
            transformation.set(1, 1, 1.0 - (Math.pow(robotDeltas.get(2, 0), 2) / 6.0));
            transformation.set(2, 2, 1.0);
        } else {
            transformation.set(0, 0, Math.sin(robotDeltas.get(2, 0)) / robotDeltas.get(2, 0));
            transformation.set(0, 1, (Math.cos(robotDeltas.get(2, 0)) - 1.0) / robotDeltas.get(2, 0));
            transformation.set(1, 0, (1.0 - Math.cos(robotDeltas.get(2, 0))) / robotDeltas.get(2, 0));
            transformation.set(1, 1, Math.sin(robotDeltas.get(2, 0)) / robotDeltas.get(2, 0));
            transformation.set(2, 2, 1.0);
        }

        globalDeltas = Matrix.multiply(Matrix.multiply(prevRotationMatrix, transformation), robotDeltas);

        displacementPose.add(new Pose(globalDeltas.get(0, 0), globalDeltas.get(1, 0), globalDeltas.get(2, 0)));
        currentVelocity = new Pose(globalDeltas.get(0, 0) / (deltaTimeNano * Math.pow(10.0, 9)), globalDeltas.get(1, 0) / (deltaTimeNano * Math.pow(10.0, 9)), globalDeltas.get(2, 0) / (deltaTimeNano * Math.pow(10.0, 9)));

        totalHeading += globalDeltas.get(2, 0);
    }

    /**
     * This updates the Encoders.
     */
    public void updateEncoders() {
        leftEncoder.update();
        rightEncoder.update();
        strafeEncoder.update();
    }

    /**
     * This resets the Encoders.
     */
    public void resetEncoders() {
        leftEncoder.reset();
        rightEncoder.reset();
        strafeEncoder.reset();
    }

    /**
     * This calculates the change in position from the perspective of the robot using information
     * from the Encoders.
     *
     * @return returns a Matrix containing the robot relative movement.
     */
    private int headingCounter = 0;
    public Matrix getRobotDeltas() {
        Matrix returnMatrix = new Matrix(3,1);
        double headingDelta =  (rightEncoder.getDeltaPosition() - leftEncoder.getDeltaPosition()) / (leftEncoderPose.getY() - rightEncoderPose.getY());
        double calHeadingDelta =  TURN_TICKS_TO_RADIANS*headingDelta;
        if(headingCounter++ > 5) {
            YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
            Rotation2d heading = Rotation2d.exp(angles.getYaw(AngleUnit.RADIANS));
            double headingDelta1 = MathFunctions.normalizeAngle(heading.toDouble())-
                    MathFunctions.normalizeAngle(displacementPose.getHeading());
            headingCounter = 0;

//            Log.d("ThreeWheelLocalizer_logger",
//                    String.format("Calc: %3.3f, Calc1: %3.3f, heading: %3.3f, prev_heading: %3.3f, delta: %3.3f",
//                            headingDelta, calHeadingDelta,
//                            MathFunctions.normalizeAngle(heading.toDouble()),
//                            MathFunctions.normalizeAngle(displacementPose.getHeading()),
//                            headingDelta1)
//            );

            calHeadingDelta = headingDelta1;
        }
        // x/forward movement
        returnMatrix.set(0,0, FORWARD_TICKS_TO_INCHES * ((rightEncoder.getDeltaPosition() * leftEncoderPose.getY() - leftEncoder.getDeltaPosition() * rightEncoderPose.getY()) / (leftEncoderPose.getY() - rightEncoderPose.getY())));
        //y/strafe movement
        returnMatrix.set(1,0, STRAFE_TICKS_TO_INCHES * (strafeEncoder.getDeltaPosition() - strafeEncoderPose.getX() * headingDelta));
        // theta/turning
        returnMatrix.set(2,0, calHeadingDelta);

//        Log.d("ThreeWheelLocalizer_logger",
//                    String.format("Y: %3.3f, Y0: %3.3f, Y1: %3.3f, Y2: %3.3f, heading: %3.3f",
//                            returnMatrix.get(1,0),
//                            STRAFE_TICKS_TO_INCHES * (strafeEncoder.getDeltaPosition()),
//                            STRAFE_TICKS_TO_INCHES * (strafeEncoderPose.getX() * headingDelta),
//                            (STRAFE_TICKS_TO_INCHES * (strafeEncoder.getDeltaPosition())-
//                            TURN_TICKS_TO_RADIANS * (strafeEncoderPose.getX() * headingDelta)),
//                            calHeadingDelta)
//            );

        return returnMatrix;
    }

    /**
     * This returns how far the robot has turned in radians, in a number not clamped between 0 and
     * 2 * pi radians. This is used for some tuning things and nothing actually within the following.
     *
     * @return returns how far the robot has turned in total, in radians.
     */
    public double getTotalHeading() {
        return totalHeading;
    }

}
