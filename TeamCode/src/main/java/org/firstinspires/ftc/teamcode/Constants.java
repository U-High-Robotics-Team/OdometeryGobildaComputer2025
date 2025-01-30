package org.firstinspires.ftc.teamcode;

public class Constants {
    // preformace constants
    public final int SLIDE_Y_MAX = 3000; // Maximum up position (if motor for slide is changed, then value needs to be changed)
    public final int SLIDE_X_MAX = 1500; // Maximum foward position (if motor for slide is changed, then value needs to be changed)
    public final int SLIDE_MIN = 0; // Minimum position (bottom)
    public final double SLIDE_POWER = 1;
    public final int SHOULDER_MAX = 1400;
    public final int SHOULDER_MIN = 0;// Minimum position (bottom)
    public final double SHOULDER_POWER = 0.6;
    public final double WRIST_UP = 0; // Wrist up position
    public final double WRIST_DOWN = 0.65; // Wrist down position
    public final double WRIST_CLIP = 0.3; // Wrist for clip (unused currently)
    public final double CLAW_OPEN = 0.6; // Claw open amount
    public final double CLAW_CLOSED = 0.25; // Claw closed amount
    public final double WHEEL_SPEED_MAX = 1;  // wheel base speed
    public final double WHEEL_SPEED_LIMITED = 0.17; // wheel base speed if slide is out

    // Thresholds
    public final double SLIDE_POSITION_THRESHOLD =  700; // position at which wheel speed is decrease

    // PID Controller Variables
    public final double kP = 0.0022; // bigger the error the faster we will fix it
    public final double kI = 0.00013; // provides extra boost when you get close to the target
    public final double kD = 0.00015; // dampens overshoot

    public Constants(){

    }
}
