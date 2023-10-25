package org.firstinspires.ftc.teamcode.subsystem;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

public class Dashboard {
    public static TelemetryPacket packet;
    public static Canvas fieldOverlay;
    private static FtcDashboard dashboard;

    public static void setUp() {
        resetDashboard();
        resetPacket();
    }

    public static void sendPacket() {
        if (dashboard != null && packet != null) {
            dashboard.sendTelemetryPacket(packet);
            resetPacket();
        } else {
            setUp();
            Log.e("Dashboard", "Attempted to send null packet. Created new packet");
        }
    }

    public static void resetPacket() {
        packet = new TelemetryPacket();
        fieldOverlay = packet.fieldOverlay();
    }

    public static FtcDashboard getInstance() {
        if (dashboard == null) setUp();
        return dashboard;
    }

    public static void resetDashboard() {
        dashboard = FtcDashboard.getInstance();
    }
}
