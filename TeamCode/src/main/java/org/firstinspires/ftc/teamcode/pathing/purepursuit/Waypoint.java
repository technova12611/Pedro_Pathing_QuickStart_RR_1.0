package org.firstinspires.ftc.teamcode.pathing.purepursuit;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.pathing.geometry.Point;
import org.firstinspires.ftc.teamcode.pathing.geometry.Pose;

import java.util.Locale;

public class Waypoint {
    private final Type type;
    private final Point point;
    private final double radius;

    private Waypoint(Point point, double radius) {
        this.type = point instanceof Pose ? Type.POSE : Type.POINT;
        this.point = point;
        this.radius = radius;
    }

    public Waypoint(Pose2d pose, double radius) {
        this.type =Type.POSE;
        this.point = new Pose(-pose.position.y, pose.position.x, pose.heading.toDouble());
        this.radius = radius;
    }

    public Waypoint(Vector2d vector, double radius) {
        this.type =Type.POINT;
        this.point = new Point(-vector.y, vector.x);
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
