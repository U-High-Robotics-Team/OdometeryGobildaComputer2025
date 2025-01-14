package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous(name="GoToAutonomousTest")
//@Disabled

public class GoToAutonomousTest extends LinearOpMode {

    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer
    private DcMotor BLeft;
    private DcMotor BRight;
    private DcMotor FLeft;
    private DcMotor FRight;

    double kP = 0.0024; // bigger the error the faster we will fix it
    double kI = 0.00013; // provides extra boost when you get close to the target
    double kD = 0.00015; // dampens overshoot

    private double integralSum = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        BLeft = hardwareMap.get(DcMotor.class, "backleft");
        BRight = hardwareMap.get(DcMotor.class, "backright");
        FLeft = hardwareMap.get(DcMotor.class, "frontleft");
        FRight = hardwareMap.get(DcMotor.class, "frontright");

        BLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        FLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        odo.setOffsets(-84.0, -168.0); // These offsets are configured for your robot
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        // Set your desired starting position
        Pose2D startingPosition = new Pose2D(DistanceUnit.MM, -923.925, -1601.47, AngleUnit.RADIANS, 0);
        odo.setPosition(startingPosition);

        // Reset odometry and IMU while keeping the set position
        odo.resetPosAndIMU();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset());
        telemetry.addData("Y offset", odo.getYOffset());
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Device Scalar", odo.getYawScalar());
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        while (opModeIsActive()) {
            moveRobotTo(0, -1000, 0); // Example movement to target position
        }
    }

    public void moveRobotTo(double targetX, double targetY, double targetHeading) {
        Pose2D pos = odo.getPosition();
        ElapsedTime timer = new ElapsedTime();
        double currentX = pos.getX(DistanceUnit.MM);
        double currentY = pos.getY(DistanceUnit.MM);
        double currentHeading = pos.getHeading(AngleUnit.RADIANS);

        double deltaX = targetX - currentX;
        double deltaY = targetY - currentY;
        double deltaHeading = targetHeading - currentHeading;
        double distanceError = Math.hypot(deltaX, deltaY);

        double lastDeltaX = deltaX;
        double lastDeltaY = deltaY;
        double lastDeltaHeading = deltaHeading;

        telemetry.addData("DeltaX: ", deltaX);
        telemetry.addData("DeltaY: ", deltaY);
        telemetry.addData("CurrentX: ", currentX);
        telemetry.addData("CurrentY: ", currentY);

        telemetry.update();

        double maxTime = 5.0; // Maximum time before we stop trying to move (in seconds)
        timer.reset();

        while (distanceError >= 1 && Math.abs(deltaHeading) >= 0.1 && timer.seconds() < maxTime) {
            pos = odo.getPosition();
            currentX = pos.getX(DistanceUnit.MM);
            currentY = pos.getY(DistanceUnit.MM);
            currentHeading = pos.getHeading(AngleUnit.RADIANS);

            deltaX = targetX - currentX;
            deltaY = targetY - currentY;
            deltaHeading = targetHeading - currentHeading;

            // Local frame correction (turning)
            double deltaXLocal = deltaX * Math.cos(currentHeading) + deltaY * Math.sin(currentHeading);
            double deltaYLocal = -deltaX * Math.sin(currentHeading) + deltaY * Math.cos(currentHeading);

            // Calculate PID control for X, Y, and Heading
            double xPower = findPIDPower(deltaXLocal, timer, lastDeltaX);
            double yPower = findPIDPower(deltaYLocal, timer, lastDeltaY);
            double turnPower = findPIDPower(deltaHeading, timer, lastDeltaHeading);

            // Calculate wheel speeds
            double frontLeft = yPower + xPower + turnPower;
            double frontRight = yPower - xPower - turnPower;
            double backLeft = yPower - xPower + turnPower;
            double backRight = yPower + xPower - turnPower;

            FLeft.setPower(frontLeft);
            FRight.setPower(frontRight);
            BLeft.setPower(backLeft);
            BRight.setPower(backRight);

            lastDeltaX = deltaX;
            lastDeltaY = deltaY;
            lastDeltaHeading = deltaHeading;

            // Update distance error
            distanceError = Math.hypot(deltaX, deltaY);

            // Telemetry for debugging
            telemetry.addData("Distance Error: ", distanceError);
            telemetry.addData("Heading Error: ", Math.abs(deltaHeading));
            telemetry.update();
        }

        // Stop all motors once the target position is reached or timeout occurs
        FLeft.setPower(0);
        FRight.setPower(0);
        BLeft.setPower(0);
        BRight.setPower(0);

    }

    public double findPIDPower(double deltaError, ElapsedTime timer, double lastError) {
        double derivative = 0;
        double out = 0;
        double error = deltaError;

        // Derivative: Change of error over time
        derivative = (error - lastError) / timer.seconds();

        // Integral: Accumulating error over time
        integralSum += error * timer.seconds();

        // PID Output
        out = (kP * error) + (kI * integralSum) + (kD * derivative);

        // Return the power adjustment based on PID
        return out;
    }

    public void moveRobot() {
        double forward = -gamepad1.left_stick_y; // Inverted Y-axis
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;


        if (gamepad1.b) {
            odo.recalibrateIMU(); // Recalibrates the IMU without resetting position
        }

        Pose2D pos = odo.getPosition();
        double heading = pos.getHeading(AngleUnit.RADIANS);

        double cosAngle = Math.cos((Math.PI / 2) - heading);
        double sinAngle = Math.sin((Math.PI / 2) - heading);

        double globalForward = forward * cosAngle + strafe * sinAngle;
        double globalStrafe = -forward * sinAngle + strafe * cosAngle;

        double[] newWheelSpeeds = new double[4];

        newWheelSpeeds[0] = globalForward + globalStrafe + rotate;
        newWheelSpeeds[1] = globalForward - globalStrafe - rotate;
        newWheelSpeeds[2] = globalForward - globalStrafe + rotate;
        newWheelSpeeds[3] = globalForward + globalStrafe - rotate;

        FLeft.setPower(newWheelSpeeds[0]);
        FRight.setPower(newWheelSpeeds[1]);
        BLeft.setPower(newWheelSpeeds[2]);
        BRight.setPower(newWheelSpeeds[3]);

        // Telemetry
        telemetry.addData("Robot XPos: ", pos.getX(DistanceUnit.MM));
        telemetry.addData("Robot YPos: ", pos.getY(DistanceUnit.MM));
        telemetry.addData("Robot Heading: ", heading);
        telemetry.addData("Forward Speed: ", globalForward);
        telemetry.addData("Strafe Speed: ", globalStrafe);
        telemetry.update();
    }
}
