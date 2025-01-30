package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name="Template for TeleOp")
public class TeleOpTemplate extends OpMode {

    Robot robot = new Robot();

    @Override
    public void loop() {
        robot.teleOpPeriodic(gamepad1, gamepad2, telemetry);
    }

    @Override
    public void start(){
        robot.startRobot();
    }


    @Override
    public void init() {
        Pose2D startingPosition = new Pose2D(DistanceUnit.MM, -871.5375, -1558.925, AngleUnit.RADIANS, 0);
        robot.initHardware(hardwareMap, startingPosition);
    }
}
