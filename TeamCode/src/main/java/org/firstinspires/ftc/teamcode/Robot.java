package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robot.RobotState;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Robot {

    // Declare all hardware components
    private GoBildaPinpointDriver odo;
    private DcMotor BLeft, BRight, FLeft, FRight, slide, shoulder;
    private Servo wrist, claw;

    // Constants
    private final Constants constants = new Constants();;
    private ElapsedTime presetTimer; //state machine timer
    private ElapsedTime timer; // autonomous timeout timer


    // States for Robot
    public enum RobotState {
        HOME,
        SUB_1,
        SUB_2,
        SUB_3,
        BASKET_1,
        BASKET_2,
        BASKET_3,
        BASKET_4,
        UNKNOWN
    }

    // Inital Targets
    private RobotState currentState = RobotState.HOME;
    private RobotState requestedState = RobotState.HOME;
    double shoulderTarget = constants.SHOULDER_MIN;
    double wristTarget = constants.WRIST_UP;
    double clawTarget = constants.CLAW_CLOSED;
    double slideTarget = constants.SLIDE_MIN;
    double wheelSpeed = constants.WHEEL_SPEED_MAX;
    double forward = 0;
    double strafe = 0;
    double rotate = 0;


    // Variables used for finding target to move to
    private int currentTargetIndex = 0; // Keeps track of the current target
    private double targetDuration = 0; // Stores timeout value


    // Constructor
    public Robot() {
    }

    // Initialize hardware
    public void initHardware(HardwareMap hardwareMap, Pose2D startPos) {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        BLeft = hardwareMap.get(DcMotor.class, "backleft");
        BRight = hardwareMap.get(DcMotor.class, "backright");
        FLeft = hardwareMap.get(DcMotor.class, "frontleft");
        FRight = hardwareMap.get(DcMotor.class, "frontright");
        slide = hardwareMap.get(DcMotor.class, "elevator");
        shoulder = hardwareMap.get(DcMotor.class, "arm");
        wrist = hardwareMap.get(Servo.class, "wrist");
        claw = hardwareMap.get(Servo.class, "claw");

        // Setting up the motors
        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        FLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        slide.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set the initial position of the odometry
        odo.setOffsets(-84.0, -168.0); // Your robot offsets
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        // Set starting position for Odometry
        odo.setPosition(startPos);
    }

    // Starting timers
    public void startRobot(){
        presetTimer = new ElapsedTime();
        timer = new ElapsedTime();
    }

    public void autoPeriodic(RobotTargetV2[] targets, Telemetry telemetry) {
        // Update odometry
        odo.update();

        if (currentTargetIndex < targets.length) {
            RobotTargetV2 target = targets[currentTargetIndex];
            targetDuration = target.time;
            requestedState = target.state;

            if (timer.seconds() < targetDuration) {
                autoMoveRobotTo(target.x, target.y, target.heading);
                stateMachine();
                moveClaw();
                moveSlide();
                moveShoulder();
                moveWrist();
            } else {
                currentTargetIndex++;
                timer.reset();
            }
        } else {
            stopMotors(); // Stop the robot once all targets are processed
        }

        // Display telemetry
        updateTelemetry(telemetry);
    }

    public void teleOpPeriodic(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        odo.update();
        configureKeybinds(gamepad1, gamepad2);
        updateTelemetry(telemetry);

        stateMachine();
        teleOpMoveRobot();
        moveSlide();
        moveShoulder();
        moveClaw();
        moveWrist();
    }


    public void configureKeybinds(Gamepad gamepad1, Gamepad gamepad2) {
        // Driver Controls
        this.forward = -gamepad1.left_stick_y * wheelSpeed; // (inverted Y-axis)
        this.strafe = gamepad1.left_stick_x * wheelSpeed;
        this.rotate = gamepad1.right_stick_x * wheelSpeed;

        if (gamepad1.right_trigger > 0.9){
            odo.resetPosAndIMU(); //resets the position to 0 and recalibrates the IMU
        }

        // Operator Controls (Preset States)
        if (gamepad2.right_bumper) {
            requestedState = RobotState.HOME;
        }
        if (gamepad2.right_trigger > 0.5) {
            requestedState = RobotState.SUB_1;
        }
        if (gamepad2.left_bumper) {
            requestedState = RobotState.BASKET_1;
        }
        if (gamepad2.left_trigger > 0.5) {
            requestedState = RobotState.BASKET_2;
        }
        if (gamepad2.a) {
            // Preset to grab block
            if (currentState == RobotState.SUB_1) {
                requestedState = RobotState.SUB_2;
            }
        }
        if (gamepad2.x) {
            // Preset to return to home
            if (currentState == RobotState.BASKET_2) {
                requestedState = RobotState.BASKET_3; // Same as home but with systematic process (ordering movements)
            }
        }
    }

    // Method to handle control inputs for field-centric drive control
    public void teleOpMoveRobot(){
        if(slide.getCurrentPosition()> constants.SLIDE_POSITION_THRESHOLD){
            wheelSpeed = constants.WHEEL_SPEED_LIMITED;
        }else{
            wheelSpeed = constants.WHEEL_SPEED_MAX;
        }

        Pose2D pos = odo.getPosition();
        double heading =  pos.getHeading(AngleUnit.RADIANS);

        double cosAngle = Math.cos((Math.PI / 2)-heading);
        double sinAngle = Math.sin((Math.PI / 2)-heading);

        double globalForward = forward * cosAngle + strafe * sinAngle;
        double globalStrafe = -forward * sinAngle + strafe * cosAngle;

        // Calculating individual wheel speeds
        double frontLeft = (globalForward + globalStrafe + rotate) * wheelSpeed;
        double frontRight = (globalForward - globalStrafe - rotate) * wheelSpeed;
        double backLeft = (globalForward - globalStrafe + rotate) * wheelSpeed;
        double backRight = (globalForward + globalStrafe - rotate) * wheelSpeed;

        moveMotors(frontLeft, frontRight, backLeft, backRight);
    }

    // Method for moving the robot to a target position
    public void autoMoveRobotTo(double targetX, double targetY, double targetHeading) {
        // Getting current positions
        Pose2D currentPosition = odo.getPosition();
        double currentX = currentPosition.getX(DistanceUnit.MM);
        double currentY = currentPosition.getY(DistanceUnit.MM);
        double currentHeading = currentPosition.getHeading(AngleUnit.RADIANS);

        if(slide.getCurrentPosition() > constants.SLIDE_X_MAX){
            wheelSpeed = constants.WHEEL_SPEED_LIMITED;
        }else{
            wheelSpeed = constants.WHEEL_SPEED_MAX;
        }

        // Finding errors using current and targets
        double deltaX = targetX - currentX;
        double deltaY = targetY - currentY;
        double deltaHeading = targetHeading - currentHeading;

        // Accounting for minor errors
        if (Math.abs(deltaY) < 0.5) {
            deltaY = 0;
        }
        if (Math.abs(deltaX) < 0.5) {
            deltaX = 0;
        }
        if (Math.abs(deltaHeading) < 0.001) {
            deltaHeading = 0;
        }

        // Inversing y-axis
        deltaY = -deltaY;

        double xPower = deltaX * constants.kP;
        double yPower = deltaY * constants.kP;
        double turnPower = -deltaHeading;

        // Negative currentHeading due to rotating global power counterclockwise
        double cosAngle = Math.cos(-currentHeading);
        double sinAngle = Math.sin(-currentHeading);

        // Using inverse rotational matrix
        double localX = xPower * cosAngle + yPower * sinAngle;
        double localY = -xPower * sinAngle + yPower * cosAngle;

        // Calculating individual wheel speeds
        double frontLeft = (localX + localY + turnPower) * wheelSpeed;
        double frontRight = (localX - localY - turnPower) * wheelSpeed;
        double backLeft = (localX - localY + turnPower) * wheelSpeed;
        double backRight = (localX + localY - turnPower) * wheelSpeed;

        moveMotors(frontLeft, frontRight, backLeft, backRight);
    }

    // State machine logic
    public void stateMachine() {
        switch (currentState) {
            case HOME:
                // immediate actions
                clawTarget = constants.CLAW_CLOSED;

                if(presetTimer.seconds() > 0.3){
                    wristTarget = constants.WRIST_UP;
                }
                // delayed actions
                if (presetTimer.seconds() > 0.2) {
                    shoulderTarget = constants.SHOULDER_MIN;
                    slideTarget = constants.SLIDE_MIN;
                }
                // allowed transistions from HOME: SUBMERSIBLE, BASKET_1
                if (requestedState == RobotState.SUB_1){
                    currentState = RobotState.SUB_1;
                    presetTimer.reset();  // start delay timer for wrist movement
                } else if (requestedState == RobotState.BASKET_1){
                    currentState = RobotState.BASKET_1;
                    presetTimer.reset();  // start delay timer for wrist movement
                }
                break;

            case SUB_1:
                // immediate actions
                clawTarget = constants.CLAW_OPEN;
                shoulderTarget = constants.SHOULDER_MIN;
                slideTarget = constants.SLIDE_X_MAX;
                wristTarget = constants.WRIST_CLIP;
                // allowed transistions from SUB: HOME, SUB_2
                if (requestedState == RobotState.HOME){
                    currentState = RobotState.HOME;
                    presetTimer.reset();  // start delay timer for wrist movement
                } else if (requestedState == RobotState.SUB_2){
                    currentState = RobotState.SUB_2;
                    presetTimer.reset();  // start delay timer for wrist movement
                }
                break;

            case SUB_2:
                // immediate actions
                wristTarget = constants.WRIST_DOWN;
                shoulderTarget = constants.SHOULDER_MIN;
                slideTarget = constants.SLIDE_X_MAX;
                // delayed actoins
                if(presetTimer.seconds() > 0.4){
                    clawTarget = constants.CLAW_CLOSED;
                }
                if(presetTimer.seconds() > 0.7){
                    currentState = RobotState.SUB_3;
                    presetTimer.reset();
                }
                // allowed transition
                break;


            case SUB_3:
                // immediate actions
                wristTarget = constants.WRIST_UP;
                // allowed transition
                if (requestedState == RobotState.HOME){
                    currentState = RobotState.HOME;
                    presetTimer.reset();  // start delay timer for wrist movement
                } else if (requestedState == RobotState.SUB_1){
                    currentState = RobotState.SUB_1;
                    presetTimer.reset();  // start delay timer for wrist movement
                }
                break;

            case BASKET_1:
                // immediate actions
                shoulderTarget = constants.SHOULDER_MAX;
                wristTarget = constants.WRIST_DOWN;
                // delayed actions
                // allowed transistions
                if (presetTimer.seconds() > 1.0) {
                    currentState = RobotState.BASKET_2;
                    presetTimer.reset();
                }
                break;

            case BASKET_2:
                slideTarget = constants.SLIDE_Y_MAX;
                clawTarget = constants.CLAW_CLOSED;

                if(presetTimer.seconds() > 1.4){
                    wristTarget = constants.WRIST_UP;
                }

                if(requestedState == RobotState.BASKET_3){
                    currentState = RobotState.BASKET_3;
                    presetTimer.reset();
                }
                break;

            case BASKET_3:
                // immediate actions
                clawTarget = constants.CLAW_OPEN;

                if(presetTimer.seconds()> 0.3){
                    currentState = RobotState.BASKET_4;
                    presetTimer.reset();
                }
                break;

            case BASKET_4:
                wristTarget = constants.WRIST_DOWN;

                if(presetTimer.seconds()>0.8){
                    slideTarget = constants.SLIDE_MIN;
                    shoulderTarget = constants.SHOULDER_MAX;
                }

                if(presetTimer.seconds()>1.4){
                    currentState = RobotState.HOME;
                    presetTimer.reset();
                }

                break;

            default:
                currentState = RobotState.UNKNOWN;

                break;
        }
    }

    // Update wheel speeds and apply motor powers based on robot movement calculations
    public void moveMotors(double frontLeft, double frontRight, double backLeft, double backRight) {
        FLeft.setPower(frontLeft);
        FRight.setPower(frontRight);
        BLeft.setPower(backLeft);
        BRight.setPower(backRight);
    }

    // Methods for controlling the arm, wrist, claw, etc.
    public void moveWrist() {
        wrist.setPosition(wristTarget);
    }

    public void moveClaw() {
        claw.setPosition(clawTarget);
    }

    public void moveShoulder() {
        shoulder.setTargetPosition((int) shoulderTarget);
        shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shoulder.setPower(Math.abs(constants.SHOULDER_POWER));
    }

    public void moveSlide() {
        // First Condition Accounts for Error Build Up
        // Second Condition Accounts for Changes Only at Home State
        // Third Condition Accounts for Situations Where There is An Immediate Statisfaction (SUB_1 to HOME)
        if(Math.abs(slideTarget - slide.getCurrentPosition())<20 && currentState == RobotState.HOME && presetTimer.seconds() > 0.5){
            slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        slide.setTargetPosition((int) slideTarget);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(Math.abs(constants.SLIDE_POWER));
    }

    // Stop all motors
    public void stopMotors() {
        moveMotors(0, 0, 0, 0);
    }

    public void updateTelemetry(Telemetry telemetry) {
        // Get current position from odometry
        Pose2D currentPosition = odo.getPosition();
        double currentX = currentPosition.getX(DistanceUnit.MM);
        double currentY = currentPosition.getY(DistanceUnit.MM);
        double currentHeading = currentPosition.getHeading(AngleUnit.RADIANS);

        // Telemetry data
        telemetry.addData("State Machine", "Current state: %s", currentState);
        telemetry.addData("Current X", currentX);
        telemetry.addData("Current Y", currentY);
        telemetry.addData("Current Heading", currentHeading);
        telemetry.addData("Shoulder Target", shoulderTarget);
        telemetry.addData("Claw Target", clawTarget);
        telemetry.addData("Wrist Target", wristTarget);
        telemetry.addData("Slide Target", slideTarget);
        telemetry.addData("Slide Current Position", slide.getCurrentPosition());
        telemetry.addData("Slide Error Position", Math.abs(slideTarget - slide.getCurrentPosition()));
        telemetry.addData("Wheel Speed", wheelSpeed);
        telemetry.update();
    }
}
