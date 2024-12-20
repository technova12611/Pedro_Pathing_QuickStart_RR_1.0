package org.firstinspires.ftc.teamcode.pedroPathing.util;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

public final class PoseMessage {
    public long timestamp;
    public double x;
    public double y;
    public double heading;

    public PoseMessage(Pose pose) {
        this.timestamp = System.nanoTime();
        if(pose == null) {
            this.x = 0.0;
            this.y = 0.0;
            this.heading = 0.0;
        } else {
            this.x = pose.getX();
            this.y = pose.getY();
            this.heading = pose.getHeading();
        }
    }

    public String toString() {
        return String.format("(%2.2f, %2.2f, %3.2f)", this.x, this.y, Math.toDegrees(this.heading));
    }
}

