package org.firstinspires.ftc.teamcode;

public class RobotTarget {
    public double x;
    public double y;
    public double heading;
    public double time;

    // Use the RobotState enum from MultiTaskAuto
    public MultiTaskAuto.RobotState state;

    public RobotTarget(double x, double y, double heading, double time, MultiTaskAuto.RobotState state) {
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.time = time;
        this.state = state;
    }
}
