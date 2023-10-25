package org.firstinspires.ftc.teamcode.subsystem;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;

import org.firstinspires.ftc.teamcode.roadrunner.Localizer;

public class StateCopyLocalizer implements Localizer {
    public static Pose2d pose = new Pose2d(0.0,0.0,0.0);
    public static Pose2d vel = new Pose2d(0.0,0.0,0.0);
    public static Pose2d accel = new Pose2d(0.0,0.0,0.0);

    @NonNull
    public Pose2d getPoseEstimate() {
        return pose;
    }

    public void setPoseEstimate(@NonNull Pose2d pose2d) {
        pose = pose2d;
    }

    @Nullable
    public Pose2d getPoseVelocity() {
        return vel;
    }

    @Override
    public Twist2dDual<Time> update() {return null;}
}
