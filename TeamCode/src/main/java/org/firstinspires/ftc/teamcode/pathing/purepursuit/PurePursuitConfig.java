package org.firstinspires.ftc.teamcode.pathing.purepursuit;

import com.acmerobotics.dashboard.config.Config;

@Config
public class PurePursuitConfig {
    public static double xP = 0.125;
    public static double xD = 0.011;

    public static double yP = 0.0805;
    public static double yD = 0.011;

    public static double hP = 1.2;
    public static double hD = 0.045;

    public static double MAX_TRANSLATIONAL_SPEED = 0.6;
    public static double MAX_ROTATIONAL_SPEED = 0.4;
    public static double X_GAIN = 1.00;

    public static double ALLOWED_TRANSLATIONAL_ERROR = 0.8;
    public static double ALLOWED_HEADING_ERROR = 0.03;
}