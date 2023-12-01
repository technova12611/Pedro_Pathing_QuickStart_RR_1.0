package org.firstinspires.ftc.teamcode.roadrunner;

import com.acmerobotics.roadrunner.Pose2d;

public final class PoseMessage {
    public long timestamp;
    public double x;
    public double y;
    public double heading;

    public PoseMessage(Pose2d pose) {
        this.timestamp = System.nanoTime();
        this.x = pose.position.x;
        this.y = pose.position.y;
        this.heading = pose.heading.log();
    }

    public String toString() {
        return String.format("(%.3f, %.3f, %.3f)", this.x, this.y, Math.toDegrees(this.heading));
    }
}

