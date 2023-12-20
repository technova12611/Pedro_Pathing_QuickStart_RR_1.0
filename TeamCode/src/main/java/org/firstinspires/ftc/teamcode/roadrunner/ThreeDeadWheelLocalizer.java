package org.firstinspires.ftc.teamcode.roadrunner;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public final class ThreeDeadWheelLocalizer implements Localizer {
    public static class Params {
        public double par0YTicks = -2467.9133801212834;//-2426.512456471011; y position of the first parallel encoder (in tick units)
        public double par1YTicks = 2467.9133801212834;//2530.6258; // y position of the second parallel encoder (in tick units)
        public double perpXTicks = -566.222994370873; //-650.620899175634;//-560.3783221003024; // x position of the perpendicular encoder (in tick units)
    }

    public static Params PARAMS = new Params();

    public final Encoder par0, par1, perp;

    public final double inPerTick;

    private int lastPar0Pos, lastPar1Pos, lastPerpPos;

    public IMU imu;

    public ThreeDeadWheelLocalizer(HardwareMap hardwareMap, double inPerTick) {
        par0 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "par")));
        par0.setDirection(DcMotorSimple.Direction.REVERSE);

        par1 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "rightBack")));
        par1.setDirection(DcMotorSimple.Direction.REVERSE);

        perp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "intake_for_perp")));

        lastPar0Pos = par0.getPositionAndVelocity().position;
        lastPar1Pos = par1.getPositionAndVelocity().position;
        lastPerpPos = perp.getPositionAndVelocity().position;

        this.inPerTick = inPerTick;

        FlightRecorder.write("THREE_DEAD_WHEEL_PARAMS", PARAMS);
    }

    public Twist2dDual<Time> update() {
        PositionVelocityPair par0PosVel = par0.getPositionAndVelocity();
        PositionVelocityPair par1PosVel = par1.getPositionAndVelocity();
        PositionVelocityPair perpPosVel = perp.getPositionAndVelocity();

        int par0PosDelta = par0PosVel.position - lastPar0Pos;
        int par1PosDelta = par1PosVel.position - lastPar1Pos;
        int perpPosDelta = perpPosVel.position - lastPerpPos;

        Twist2dDual<Time> twist = new Twist2dDual<>(
                new Vector2dDual<>(
                        new DualNum<Time>(new double[] {
                                (PARAMS.par0YTicks * par1PosDelta - PARAMS.par1YTicks * par0PosDelta) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                                (PARAMS.par0YTicks * par1PosVel.velocity - PARAMS.par1YTicks * par0PosVel.velocity) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                        }).times(inPerTick),
                        new DualNum<Time>(new double[] {
                                (PARAMS.perpXTicks / (PARAMS.par0YTicks - PARAMS.par1YTicks) * (par1PosDelta - par0PosDelta) + perpPosDelta),
                                (PARAMS.perpXTicks / (PARAMS.par0YTicks - PARAMS.par1YTicks) * (par1PosVel.velocity - par0PosVel.velocity) + perpPosVel.velocity),
                        }).times(inPerTick)
                ),
                new DualNum<>(new double[] {
                        (par0PosDelta - par1PosDelta) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                        (par0PosVel.velocity - par1PosVel.velocity) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                })
        );

        lastPar0Pos = par0PosVel.position;
        lastPar1Pos = par1PosVel.position;
        lastPerpPos = perpPosVel.position;

//        Log.d("Localizer_Update", String.format("par0_delta: %d | par1_delta: %d | angle: %3.3f | IMU:%3.3f",
//                par0PosDelta, par1PosDelta, Math.toDegrees(twist.value().angle), imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)));

//        Log.d("Localizer_Update", String.format("par0_delta: %d | par1_delta: %d | angle: %3.3f",
//        par0PosDelta, par1PosDelta, Math.toDegrees(twist.value().angle)));

        return twist;
    }
}
