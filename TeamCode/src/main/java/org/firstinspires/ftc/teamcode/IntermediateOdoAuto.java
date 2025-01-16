package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous(name = "IntermediateOdoAuto")
// @Disabled
public class IntermediateOdoAuto extends LinearOpMode {

    GoBildaPinpointDriver odo; // Odometry object
    private DcMotor BLeft, BRight, FLeft, FRight;

    final double POSITION_TOLERANCE = 10; // Adjust based on accuracy needs

    @Override
    public void runOpMode() {
        // Initialize hardware
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        BLeft = hardwareMap.get(DcMotor.class, "backleft");
        BRight = hardwareMap.get(DcMotor.class, "backright");
        FLeft = hardwareMap.get(DcMotor.class, "frontleft");
        FRight = hardwareMap.get(DcMotor.class, "frontright");

        // Set motor directions
        BLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        FLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // reset odometry
        odo.resetPosAndIMU();

        // starting position
        Pose2D startingPosition = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.RADIANS, 0);
        odo.setPosition(startingPosition);

        // Wait for the start signal
        telemetry.addData("Status", "Initialized. Waiting for start...");
        telemetry.update();
        waitForStart();

        // Define target coordinates
        double targetX = 100; // Target X position in mm
        double targetY = 100;  // Target Y position in mm

        // Drive to target position
        driveToPosition(targetX, targetY);

        // Stop the robot once target is reached
        stopMotors();
    }

    private void driveToPosition(double targetX, double targetY) {
        while (opModeIsActive()) {
            // Get the robot's current position
            odo.update();
            Pose2D currentPosition = odo.getPosition();
            double currentX = currentPosition.getX(DistanceUnit.MM);
            double currentY = currentPosition.getY(DistanceUnit.MM);

            // Calculate errors
            double errorX = targetX - currentX;
            double errorY = targetY - currentY;

            // Break the loop if within tolerance
            if (Math.abs(errorX) < POSITION_TOLERANCE && Math.abs(errorY) < POSITION_TOLERANCE) {
                telemetry.addData("Status", "Target reached!");
                telemetry.update();
                break;
            }

            if(errorY < 1 && errorY > -1){
                errorY = 0;
            }


            if(errorX < 1 && errorX > -1){
                errorX = 0;
            }

            // Calculate motor powers
            double forward = errorX * 0.01; // Proportional control for Y (adjust multiplier)
            double strafe = -errorY * 0.01;  // Proportional control for X (adjust multiplier)

            // Limit motor powers
            forward = Math.max(-1, Math.min(1, forward));
            strafe = Math.max(-1, Math.min(1, strafe));

            // Set motor powers
            double[] motorPowers = calculateMotorPowers(forward, strafe);
            FLeft.setPower(motorPowers[0]);
            FRight.setPower(motorPowers[1]);
            BLeft.setPower(motorPowers[2]);
            BRight.setPower(motorPowers[3]);

            // Send telemetry data
            telemetry.addData("Current X", currentX);
            telemetry.addData("Current Y", currentY);
            telemetry.addData("Target X", targetX);
            telemetry.addData("Target Y", targetY);
            telemetry.addData("Error X", errorX);
            telemetry.addData("Error Y", errorY);
            telemetry.update();
        }
    }

    private double[] calculateMotorPowers(double forward, double strafe) {
        double[] motorPowers = new double[4];
        motorPowers[0] = forward + strafe;  // Front Left
        motorPowers[1] = forward - strafe;  // Front Right
        motorPowers[2] = forward - strafe;  // Back Left
        motorPowers[3] = forward + strafe;  // Back Right
        return motorPowers;
    }

    private void stopMotors() {
        FLeft.setPower(0);
        FRight.setPower(0);
        BLeft.setPower(0);
        BRight.setPower(0);
    }
}
