package org.firstinspires.ftc.teamcode;

import android.util.Log;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.roadrunner.LazyImu;
import org.firstinspires.ftc.teamcode.roadrunner.Localizer;
import org.firstinspires.ftc.teamcode.roadrunner.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.roadrunner.messages.PoseMessage;
import org.firstinspires.ftc.teamcode.roadrunner.TwoDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.utils.hardware.HardwareCreator;
import org.firstinspires.ftc.teamcode.utils.software.MovingArrayList;

import java.lang.Math;
import java.text.DecimalFormat;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

@Config
public final class MecanumDrive {

    public static class Params {
        // drive model parameters
        public double inPerTick = 0.002948; //0.002934; //24.0 / 8163.0;
        public double lateralInPerTick = 0.00273;
        public double trackWidthTicks = 4665.3368763274475;//4810.861094343746;//4623.060031773916;//4620.300191769058; //4982.1078188621495; //4691.229665989946;

        // feedforward parameters (in tick units)
        public double kS = 1.407;//1.43456;//1.2;
        public double kV = 0.0004003;// 0.0003895;//0.0004008; //0.00027;//0.00009;
        public double kA = 0.0000739; //0.000075;

        public double maxWheelVelSlow = 50;
        public double minProfileAccelSlow = -35;
        public double maxProfileAccelSlow = 50;

        // path profile parameters (in inches)
        public double maxWheelVel = 55;
        public double minProfileAccel = -40;
        public double maxProfileAccel = 60.0;

//        public double maxWheelVelHighSpeed = 72.5;
//        public double minProfileAccelHighSpeed = -52.0;
//        public double maxProfileAccelHighSpeed = 73.5;

        public double maxWheelVelHighSpeed = 70.0;
        public double minProfileAccelHighSpeed = -30.0;
        public double maxProfileAccelHighSpeed = 65.0;

        // turn profile parameters (in radians)
        public double maxAngVel = Math.PI; // shared with path
        public double maxAngAccel = Math.PI;

        // path controller gains
        public double axialGain = 7.25; //5.25;
        public double lateralGain = 16.25; //16.5;
        public double headingGain = 8.05; //7.5; // shared with turn

        public double axialVelGain = 0.525; //0.25;
        public double lateralVelGain = 0.25; //0.01;
        public double headingVelGain = 0.05; //0.01; // shared with turn
    }

    public static Params PARAMS = new Params();

    public final MecanumKinematics kinematics = new MecanumKinematics(
            PARAMS.inPerTick * PARAMS.trackWidthTicks, PARAMS.inPerTick / PARAMS.lateralInPerTick);

    public final TurnConstraints defaultTurnConstraints = new TurnConstraints(
            PARAMS.maxAngVel, -PARAMS.maxAngAccel, PARAMS.maxAngAccel);
    public final VelConstraint defaultVelConstraint = new MinVelConstraint(Arrays.asList(
            kinematics.new WheelVelConstraint(PARAMS.maxWheelVel),
            new AngularVelConstraint(PARAMS.maxAngVel)));
    public final AccelConstraint defaultAccelConstraint = new ProfileAccelConstraint(PARAMS.minProfileAccel,
            PARAMS.maxProfileAccel);

    public final VelConstraint highSpeedVelConstraint = new MinVelConstraint(Arrays.asList(
            kinematics.new WheelVelConstraint(PARAMS.maxWheelVelHighSpeed),
            new AngularVelConstraint(PARAMS.maxAngVel)));
    public final AccelConstraint highSpeedAccelConstraint = new ProfileAccelConstraint(PARAMS.minProfileAccelHighSpeed,
            PARAMS.maxProfileAccelHighSpeed);

    public final VelConstraint slowVelConstraint = new MinVelConstraint(Arrays.asList(
            kinematics.new WheelVelConstraint(PARAMS.maxWheelVelSlow),
            new AngularVelConstraint(PARAMS.maxAngVel)));
    public final AccelConstraint slowAccelConstraint = new ProfileAccelConstraint(PARAMS.minProfileAccelSlow,
            PARAMS.maxProfileAccelSlow);

    public final DcMotorEx leftFront, leftBack, rightBack, rightFront;

    public final VoltageSensor voltageSensor;

//    private final LazyImu lazyImu;

    public final IMU imu;

    public final Localizer localizer;
    public Pose2d pose;

    private final LinkedList<Pose2d> poseHistory = new LinkedList<>();

    public static Long previousLogTimestamp = null;

    public static Long autoStartTimestamp = null;

    private boolean cancelTrajectory = false;

    public MecanumDrive(HardwareMap hardwareMap, Pose2d pose, boolean isAuto) {
        this.pose = pose;

        LynxFirmware.throwIfModulesAreOutdated(hardwareMap);

        if(!isAuto) {
            for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
                module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            }
        } else {
            for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
                module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            }
        }

        leftFront = HardwareCreator.createMotor(hardwareMap, "leftFront");
        leftBack = HardwareCreator.createMotor(hardwareMap, "leftBack");
        rightBack = HardwareCreator.createMotor(hardwareMap, "rightBack");
        rightFront = HardwareCreator.createMotor(hardwareMap, "rightFront");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LazyImu lazyImu;
        lazyImu = new LazyImu(hardwareMap, "imu", new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu = lazyImu.get();

//        imu = hardwareMap.get(IMU.class, "imu");
//        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
//                RevHubOrientationOnRobot.UsbFacingDirection.UP));

//        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
//        RevHubOrientationOnRobot.UsbFacingDirection.DOWN));
//        imu.initialize(parameters);
//        imu.resetYaw();

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        //localizer = new TwoDeadWheelLocalizer(hardwareMap, imu, PARAMS.inPerTick);

        localizer = new ThreeDeadWheelLocalizer(hardwareMap, imu, PARAMS.inPerTick);

//        localizer = new ThreeDeadWheelLocalizer(hardwareMap, PARAMS.inPerTick);
//        ((ThreeDeadWheelLocalizer)localizer).imu = imu;

        previousLogTimestamp = null;
        autoStartTimestamp = null;

        FlightRecorder.write("MECANUM_PARAMS", PARAMS);
    }

    public void setDrivePowers(PoseVelocity2d powers) {
        MecanumKinematics.WheelVelocities<Time> wheelVels = new MecanumKinematics(1).inverse(
                PoseVelocity2dDual.constant(powers, 1));

        double maxPowerMag = 1;
        for (DualNum<Time> power : wheelVels.all()) {
            maxPowerMag = Math.max(maxPowerMag, power.value());
        }

        leftFront.setPower(wheelVels.leftFront.get(0) / maxPowerMag);
        leftBack.setPower(wheelVels.leftBack.get(0) / maxPowerMag);
        rightBack.setPower(wheelVels.rightBack.get(0) / maxPowerMag);
        rightFront.setPower(wheelVels.rightFront.get(0) / maxPowerMag);
    }

    public final class FollowTrajectoryAction implements Action {
        public final TimeTrajectory timeTrajectory;
        private double beginTs = -1;
        private final double[] xPoints, yPoints;
        private double previous_leftFront = 0.0;
        private double previous_leftBack = 0.0;
        private double previous_rightFront = 0.0;
        private double previous_rightBack = 0.0;
        private ElapsedTime elapsedTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        private ElapsedTime loopTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        MovingArrayList loopTime = new MovingArrayList(50);

        private boolean logProcessingTime = false;

        private int loopCounter = 0;
        private double lastVoltage = 0.0;

        public FollowTrajectoryAction(TimeTrajectory t) {
            timeTrajectory = t;
            List<Double> disps = com.acmerobotics.roadrunner.Math.range(
                    0, t.path.length(),
                    Math.max(2, (int) Math.ceil(t.path.length() / 2)));
            xPoints = new double[disps.size()];
            yPoints = new double[disps.size()];
            for (int i = 0; i < disps.size(); i++) {
                Pose2d p = t.path.get(disps.get(i), 1).value();
                xPoints[i] = p.position.x;
                yPoints[i] = p.position.y;
            }
        }
        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
                elapsedTimer.reset();
                loopTimer.reset();
                loopTime.clear();
                cancelTrajectory = false;
                lastVoltage = voltageSensor.getVoltage();
            } else {
                t = Actions.now() - beginTs;
            }

            if (t >= timeTrajectory.duration || cancelTrajectory) {
                leftFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);

                previous_leftFront = 0.0;
                previous_leftBack = 0.0;
                previous_rightBack = 0.0;
                previous_rightFront = 0.0;

                updatePoseEstimate();

                Log.d("Drive_loop_logger", " -- Avg loop time: " + String.format("A: %3.3f | M: %3.3f", loopTime.getAvg(),loopTime.getMean())
                        + " | cancelTrajectory: " + cancelTrajectory + " | Trajectory elapsed time: " + String.format("%3.3f", t*1000)
                        + " | Trajectory original time: " + String.format("%3.3f", timeTrajectory.duration*1000)
                        +  " | Estimated Pose: " + new PoseMessage(pose)
                        + " ---");

                return false;
            }

            if(logProcessingTime) {
                Log.d("Drive_loop_logger", "    0. start (other subsystems): " + String.format("%3.3f", elapsedTimer.milliseconds()));
                elapsedTimer.reset();
            }

            Pose2dDual<Time> txWorldTarget = timeTrajectory.get(t);

            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            if(logProcessingTime) {
                Log.d("Drive_loop_logger", "    1. updatePoseEstimate: " + String.format("%3.3f", elapsedTimer.milliseconds()));
                elapsedTimer.reset();
            }

            PoseVelocity2dDual<Time> command = new HolonomicController(
                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain)
                    .compute(txWorldTarget, pose, robotVelRobot);

//            driveCommandWriter.write(new DriveCommandMessage(command));

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);

            if(logProcessingTime) {
                Log.d("Drive_loop_logger", "    2_1. calculate kinematics: " + String.format("%3.3f", elapsedTimer.milliseconds()));
                elapsedTimer.reset();
            }

            double voltage = lastVoltage;

            if(loopCounter++ == 4) {
                voltage = voltageSensor.getVoltage();
                lastVoltage = voltage;
                loopCounter = 0;
            }

            if(logProcessingTime) {
                Log.d("Drive_logger", "      2_2. calculate getVoltage(): v: " + String.format("%3.3f", voltage)
                        + " t: " + String.format("%3.3f", elapsedTimer.milliseconds()));
                elapsedTimer.reset();
            }

            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS, PARAMS.kV / PARAMS.inPerTick,
                    PARAMS.kA / PARAMS.inPerTick);
            double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
            double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
            double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
            double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;

//            mecanumCommandWriter.write(new MecanumCommandMessage(
//                    voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
//            ));

            if(logProcessingTime) {
               Log.d("Drive_loop_logger", "      2_3. calculate power: " + String.format("%3.3f", elapsedTimer.milliseconds()));
               elapsedTimer.reset();
            }

            double delta_power = 0.01;
            if(Math.abs(leftFrontPower - previous_leftFront) > delta_power) {
                leftFront.setPower(leftFrontPower);
                previous_leftFront = leftFrontPower;
            }

            if(Math.abs(leftBackPower - previous_leftBack) > delta_power) {
                leftBack.setPower(leftBackPower);
                previous_leftBack = leftBackPower;
            }

            if(logProcessingTime) {
                Log.d("Drive_loop_logger", "    3_1. setLeftPower(): " + String.format("%3.3f", elapsedTimer.milliseconds()));
                elapsedTimer.reset();
            }

            if(Math.abs(rightBackPower - previous_rightBack) > delta_power) {
                rightBack.setPower(rightBackPower);
                previous_rightBack = rightBackPower;
            }

            if(Math.abs(rightFrontPower - previous_rightFront) > delta_power) {
                rightFront.setPower(rightFrontPower);
                previous_rightFront = rightFrontPower;
            }

            if(logProcessingTime) {
                Log.d("Drive_loop_logger", "      3_2. setRightPower(): " + String.format("%3.3f", elapsedTimer.milliseconds()));
                elapsedTimer.reset();
            }

            p.put("x", pose.position.x);
            p.put("y", pose.position.y);
            p.put("heading (deg)", Math.toDegrees(pose.heading.toDouble()));

            Pose2d error = txWorldTarget.value().minusExp(pose);
            p.put("xError", error.position.x);
            p.put("yError", error.position.y);
            p.put("headingError (deg)", Math.toDegrees(error.heading.toDouble()));

            // only draw when active; only one drive action should be active at a time
            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            drawRobot(c, pose);

            c.setStroke("#4CAF50FF");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);

            if(logProcessingTime) {
                Log.d("Drive_loop_logger", "    Dashboard drawing: " + String.format("%3.3f", elapsedTimer.milliseconds()));
                elapsedTimer.reset();
            }

            loopTime.add(loopTimer.milliseconds());

            if(logProcessingTime) {
                Log.d("Drive_loop_logger", "  Loop Time: " + String.format("%3.3f", loopTimer.milliseconds()));
            }
            loopTimer.reset();

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#4CAF507A");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);
        }
    }

    public final class TurnAction implements Action {
        private final TimeTurn turn;

        private double beginTs = -1;

        private ElapsedTime elapsedTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        private double previous_leftFront = 0.0;
        private double previous_leftBack = 0.0;
        private double previous_rightFront = 0.0;
        private double previous_rightBack = 0.0;

        public TurnAction(TimeTurn turn) {
            this.turn = turn;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
                elapsedTimer.reset();
            } else {
                t = Actions.now() - beginTs;
            }

            if (t >= turn.duration) {
                leftFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);
                previous_leftFront = 0.0;
                previous_leftBack = 0.0;
                previous_rightBack = 0.0;
                previous_rightFront = 0.0;

                return false;
            }

            Pose2dDual<Time> txWorldTarget = turn.get(t);

            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            PoseVelocity2dDual<Time> command = new HolonomicController(
                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain)
                    .compute(txWorldTarget, pose, robotVelRobot);

//            driveCommandWriter.write(new DriveCommandMessage(command));

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();
            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                    PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
            double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
            double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
            double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
            double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;

//            mecanumCommandWriter.write(new MecanumCommandMessage(
//                    voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
//            ));

            double delta_power = 0.0;
            if(Math.abs(leftFrontPower - previous_leftFront) > delta_power) {
                leftFront.setPower(leftFrontPower);
                previous_leftFront = leftFrontPower;
            }

            if(Math.abs(leftBackPower - previous_leftBack) > delta_power) {
                leftBack.setPower(leftBackPower);
                previous_leftBack = leftBackPower;
            }

            if(Math.abs(rightBackPower - previous_rightBack) > delta_power) {
                rightBack.setPower(rightBackPower);
                previous_rightBack = rightBackPower;
            }

            if(Math.abs(rightFrontPower - previous_rightFront) > delta_power) {
                rightFront.setPower(rightFrontPower);
                previous_rightFront = rightFrontPower;
            }

            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            drawRobot(c, pose);

            c.setStroke("#7C4DFFFF");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);

            //Log.d("Drive_logger", "Drive Loop time: " + elapsedTimer.milliseconds());
            elapsedTimer.reset();

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#7C4DFF7A");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);
        }
    }

    public PoseVelocity2d updatePoseEstimate() {
        Twist2dDual<Time> twist = localizer.update();
        pose = pose.plus(twist.value());

        poseHistory.add(pose);
        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }

//        estimatedPoseWriter.write(new PoseMessage(pose));

        return twist.velocity().value();
    }

    public void drawPoseHistory(Canvas c) {
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];

        int i = 0;
        for (Pose2d t : poseHistory) {
            xPoints[i] = t.position.x;
            yPoints[i] = t.position.y;

            i++;
        }

        c.setStrokeWidth(1);
        c.setStroke("#3F51B5");
        c.strokePolyline(xPoints, yPoints);
    }

    private static void drawRobot(Canvas c, Pose2d t) {
        final double ROBOT_RADIUS = 9;

        c.setStrokeWidth(1);
        c.strokeCircle(t.position.x, t.position.y, ROBOT_RADIUS);

        Vector2d halfv = t.heading.vec().times(0.5 * ROBOT_RADIUS);
        Vector2d p1 = t.position.plus(halfv);
        Vector2d p2 = p1.plus(halfv);
        c.strokeLine(p1.x, p1.y, p2.x, p2.y);
    }

    public TrajectoryActionBuilder actionBuilder(Pose2d beginPose) {
        Log.d("TrajectoryActionBuilder_default", "Default Drive path builder begin position: " + new PoseMessage(beginPose));
        return new TrajectoryActionBuilder(
                TurnAction::new,
                FollowTrajectoryAction::new,
                beginPose, 1e-6, 0.0,
                defaultTurnConstraints,
                defaultVelConstraint, defaultAccelConstraint,
                0.25, 0.1);
    }

    public TrajectoryActionBuilder actionBuilderSlow(Pose2d beginPose) {
        Log.d("TrajectoryActionBuilder_slow", "Slow Drive path builder begin position: " + new PoseMessage(beginPose));
        return new TrajectoryActionBuilder(
                TurnAction::new,
                FollowTrajectoryAction::new,
                beginPose, 1e-6, 0.0,
                defaultTurnConstraints,
                slowVelConstraint, slowAccelConstraint,
                0.25, 0.1);
    }

    public TrajectoryActionBuilder actionBuilderFast(Pose2d beginPose) {
        Log.d("TrajectoryActionBuilder_fast", "Fast Drive path builder begin position: " + new PoseMessage(beginPose));
        return new TrajectoryActionBuilder(
                TurnAction::new,
                FollowTrajectoryAction::new,
                beginPose, 1e-6, 0.0,
                defaultTurnConstraints,
                highSpeedVelConstraint, highSpeedAccelConstraint,
                0.25, 0.1);
    }

    public static class DrivePoseLoggingAction implements Action {
        MecanumDrive drive;
        String label;

        String message;

        boolean logPixelCount = true;

        public DrivePoseLoggingAction(MecanumDrive drive, String label) {
            this.drive = drive;
            this.label = label;
        }

        public DrivePoseLoggingAction(MecanumDrive drive, String label, String message) {
            this.drive = drive;
            this.label = label;
            this.message = message;
        }

        public DrivePoseLoggingAction(MecanumDrive drive, String label, boolean logPixelCount) {
            this.drive = drive;
            this.label = label;
            this.logPixelCount = logPixelCount;
        }

        public DrivePoseLoggingAction(MecanumDrive drive, String label, String message, boolean logPixelCount) {
            this.drive = drive;
            this.label = label;
            this.logPixelCount = logPixelCount;
            this.message = message;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            if(previousLogTimestamp == null) {
                MecanumDrive.previousLogTimestamp = System.currentTimeMillis();
            }
            if(MecanumDrive.autoStartTimestamp == null){
                MecanumDrive.autoStartTimestamp = System.currentTimeMillis();
            }
            DecimalFormat formatter = new DecimalFormat("###,### (ms)");

            Log.d("Drive_Logger", "Estimated Pose: " + new PoseMessage(drive.pose)
                    + "  [" + this.label + "]" + " | Elapsed time: "
                    + formatter.format((System.currentTimeMillis() - MecanumDrive.previousLogTimestamp))
                    + (logPixelCount? " | Pixel count:" + Intake.pixelsCount + " | Total Pixel count:" + Intake.totalPixelCount:"")
                    + (message != null? " | { " + this.message + " }": "")
                    + " | Auto Timer (s): " + String.format("%.3f",(System.currentTimeMillis() - MecanumDrive.autoStartTimestamp)/1000.0)
//                    + " | heading: " + String.format("%3.2f", this.drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES))
            );

            MecanumDrive.previousLogTimestamp = System.currentTimeMillis();
            return false;
        }
    }

    public static class AutoPositionCheckAction implements Action {
        MecanumDrive drive;
        Pose2d target;
        Boolean firstTime = null;

        public AutoPositionCheckAction(MecanumDrive drive, Pose2d position) {
            this.drive = drive;
            this.target = position;
        }

        @Override
        public boolean run(TelemetryPacket packet) {

            if(firstTime == null) {
                firstTime = true;
                Log.d("PositionCheck_Logger", "Estimated Pose: " + new PoseMessage(drive.pose)
                        + " | Target Pose: " + new PoseMessage(target)
                );
            }
            if(Math.abs(drive.pose.position.x - target.position.x) > 6.0 ||
                    Math.abs(drive.pose.position.y - target.position.y) > 6.0) {
                return true;
            }

            return false;
        }
    }

    public void cancelCurrentTrajectory() {
        this.cancelTrajectory = true;
    }

    public void resetCancelTrajectory() {
        this.cancelTrajectory = false;
    }

    public void startIMUThread(LinearOpMode opMode) {
        ((ThreeDeadWheelLocalizer)localizer).startIMUThread(opMode);
    }

    public double getVoltage() {
        return voltageSensor.getVoltage();
    }

    public void setPower(Pose pose) {
        setDrivePowers(new PoseVelocity2d(new Vector2d(pose.x, pose.y), pose.heading));
    }
}
