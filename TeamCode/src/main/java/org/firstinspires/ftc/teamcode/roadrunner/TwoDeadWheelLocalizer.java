package org.firstinspires.ftc.teamcode.roadrunner;

import android.util.Log;

import androidx.annotation.GuardedBy;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utils.hardware.HardwareCreator;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

@Config
public final class TwoDeadWheelLocalizer implements Localizer {
    public static class Params {
        /*public double parYTicks = 6.5 / (24.0 / 8235.0); // y position of the parallel encoder (in tick units)
        public double perpXTicks = -5 / (24.0 / 8235.0); // x position of the perpendicular encoder (in tick units)*/
        public double parYTicks = -2441.8494827666686; //-2427.9661725218807;//7.0/(0.00297); // 11110.24332364755; // y position of the parallel encoder (in tick units)
        public double perpXTicks = -466.222994370873;//-596.222994370873;//-566.222994370873;//-439.8233849177432; //1.8/(0.00297); //-9980.239241262898; // x position of the perpendicular encoder (in tick units)
    }

    public static Params PARAMS = new Params();

    public final Encoder par, perp;
//    private final Object imuLock = new Object();
//    @GuardedBy("imuLock")
    public final IMU imu;

    private int lastParPos, lastPerpPos;
    private Rotation2d lastHeading;

    private final double inPerTick;

    private double lastRawHeadingVel, headingVelOffset;

    private double imuYawHeading = 0.0;

    private double imuHeadingVelo = 0.0;
    private ExecutorService imuExecutor = Executors.newSingleThreadExecutor();

    public TwoDeadWheelLocalizer(HardwareMap hardwareMap, IMU imu, double inPerTick) {
        par = new OverflowEncoder(new RawEncoder(HardwareCreator.createMotor(hardwareMap, "par")));
        par.setDirection(DcMotorSimple.Direction.REVERSE);
        perp = new OverflowEncoder(new RawEncoder(HardwareCreator.createMotor(hardwareMap, "intake_for_perp")));
        //perp.setDirection(DcMotorSimple.Direction.REVERSE);
        this.imu = imu;

        lastParPos = par.getPositionAndVelocity().position;
        lastPerpPos = perp.getPositionAndVelocity().position;
        lastHeading = Rotation2d.exp(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        this.inPerTick = inPerTick;

        FlightRecorder.write("TWO_DEAD_WHEEL_PARAMS", PARAMS);
    }

    public void startIMUThread(LinearOpMode opMode) {
//        imuExecutor.submit(new Runnable() {
//            @Override
//            public void run() {
//                while (!opMode.isStopRequested()) {
//                    synchronized (imuLock) {
//                        imuYawHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//                        imuHeadingVelo = imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
//                    }
//                }
//            }
//        });
    }

    // see https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/617
    private double getHeadingVelocity() {
        double rawHeadingVel = imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
        if (Math.abs(rawHeadingVel - lastRawHeadingVel) > Math.PI) {
            headingVelOffset -= Math.signum(rawHeadingVel) * 2 * Math.PI;
        }
        lastRawHeadingVel = rawHeadingVel;
        return headingVelOffset + rawHeadingVel;
    }

    public Twist2dDual<Time> update() {

//        Long t0 = System.currentTimeMillis();

        PositionVelocityPair parPosVel = par.getPositionAndVelocity();
        PositionVelocityPair perpPosVel = perp.getPositionAndVelocity();
        Rotation2d heading = Rotation2d.exp(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        int parPosDelta = parPosVel.position - lastParPos;
        int perpPosDelta = (int)(perpPosVel.position) - lastPerpPos;
        double headingDelta = heading.minus(lastHeading);

        double headingVel = getHeadingVelocity();

        Twist2dDual<Time> twist = new Twist2dDual<>(
                new Vector2dDual<>(
                        new DualNum<Time>(new double[] {
                                parPosDelta - PARAMS.parYTicks * headingDelta,
                                parPosVel.velocity - PARAMS.parYTicks * headingVel,
                        }).times(inPerTick),
                        new DualNum<Time>(new double[] {
                                perpPosDelta - PARAMS.perpXTicks * headingDelta,
                                perpPosVel.velocity - PARAMS.perpXTicks * headingVel,
                        }).times(inPerTick*0.993)),
                new DualNum<>(new double[] {
                        headingDelta,
                        headingVel,
                }));

        lastParPos = parPosVel.position;
        lastPerpPos = perpPosVel.position;
        lastHeading = heading;

//        Log.d("Localizer_logger", "update() called, elapsed time:" + (System.currentTimeMillis()-t0));

        return twist;
    }
}
