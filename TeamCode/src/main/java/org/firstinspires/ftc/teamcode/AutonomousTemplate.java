package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import java.lang.annotation.Target;
import com.qualcomm.robotcore.robot.RobotState;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous(name="Template for Autonomous")
public class AutonomousTemplate extends OpMode {

    RobotTargetV2[] targets = {
            // here you can add robot target for autonomous
    };

    Robot robot = new Robot();

    @Override
    public void loop() {
        robot.autoPeriodic(targets, telemetry);
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
