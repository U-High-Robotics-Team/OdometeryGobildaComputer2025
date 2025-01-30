package org.firstinspires.ftc.teamcode;

public class RobotTargetV2 {
    public double x;
    public double y;
    public double heading;
    public double time;

    // Use the RobotState enum from MultiTaskAuto
    public Robot.RobotState state;

    public RobotTargetV2(double x, double y, double heading, double time, Robot.RobotState state) {
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.time = time;
        this.state = state;
    }
}
