package org.firstinspires.ftc.teamcode.pedroPathing.util;

import com.acmerobotics.roadrunner.Pose2d;

public final class PoseMessage {
    public long timestamp;
    public double x;
    public double y;
    public double heading;

    public PoseMessage(Pose2d pose) {
        this.timestamp = System.nanoTime();
        if(pose == null) {
            this.x = 0.0;
            this.y = 0.0;
            this.heading = 0.0;
        } else {
            this.x = pose.position.x;
            this.y = pose.position.y;
            this.heading = pose.heading.toDouble();
        }
    }

    public String toString() {
        return String.format("(%2.2f, %2.2f, %3.2f)", this.x, this.y, Math.toDegrees(this.heading));
    }
}

