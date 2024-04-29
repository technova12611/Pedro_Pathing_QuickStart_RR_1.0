package org.firstinspires.ftc.teamcode.pathing.purepursuit;

import org.firstinspires.ftc.teamcode.pathing.geometry.Point;
import org.firstinspires.ftc.teamcode.pathing.geometry.Pose;

import java.util.Locale;

public class Waypoint {
    private final Type type;
    private final Point point;
    private final double radius;

    public Waypoint(Point point, double radius) {
        this.type = point instanceof Pose ? Type.POSE : Type.POINT;
        this.point = point;
        this.radius = radius;
    }

    public Type getType() {
        return this.type;
    }

    public Point getPoint() {
        return this.point;
    }

    public double getRadius() {
        return this.radius;
    }

    public enum Type {
        POINT, POSE
    }

    @Override
    public String toString() {
        return String.format(Locale.ENGLISH, "%s %s %.2f", getType(), getPoint(), getRadius());
    }
}
