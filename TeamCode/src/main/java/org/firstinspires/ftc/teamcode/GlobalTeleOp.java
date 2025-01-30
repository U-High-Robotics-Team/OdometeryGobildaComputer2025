/*   MIT License
 *   Copyright (c) [2024] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.robot.RobotState;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DoubleBasketPark.RobotStates;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

/*
This opmode shows how to use the goBILDA® Pinpoint Odometry Computer.
The goBILDA Odometry Computer is a device designed to solve the Pose Exponential calculation
commonly associated with Dead Wheel Odometry systems. It reads two encoders, and an integrated
system of senors to determine the robot's current heading, X position, and Y position.

it uses an ESP32-S3 as a main cpu, with an STM LSM6DSV16X IMU.
It is validated with goBILDA "Dead Wheel" Odometry pods, but should be compatible with any
quadrature rotary encoder. The ESP32 PCNT peripheral is speced to decode quadrature encoder signals
at a maximum of 40mhz per channel. Though the maximum in-application tested number is 130khz.

The device expects two perpendicularly mounted Dead Wheel pods. The encoder pulses are translated
into mm and their readings are transformed by an "offset", this offset describes how far away
the pods are from the "tracking point", usually the center of rotation of the robot.

Dead Wheel pods should both increase in count when moved forwards and to the left.
The gyro will report an increase in heading when rotated counterclockwise.

The Pose Exponential algorithm used is described on pg 181 of this book:
https://github.com/calcmogul/controls-engineering-in-frc

For support, contact tech@gobilda.com

-Ethan Doak
 */

@TeleOp(name="Global TeleOp")
//@Disabled

public class GlobalTeleOp extends OpMode {

    // Timer for Servos
    private final ElapsedTime timer = new ElapsedTime();


    // Performance constants
    final int SLIDE_Y_MAX = 3000;
    final int SLIDE_X_MAX = 1800; // Maximum position (top)
    final int SLIDE_MIN = 0; // Minimum position (bottom)
    final double SLIDE_POWER = 1;
    final int SHOULDER_MAX = 1400;
    final int SHOULDER_MIN = 0;
    final double SHOULDER_POWER = 0.6;
    final double WRIST_UP = 0;
    final double WRIST_DOWN = 0.65;
    final double WRIST_CLIP = 0.3; // unused currently
    final double CLAW_OPEN = 0.65;
    final double CLAW_CLOSED = 0;
    final double WHEEL_SPEED_MAX = 1;
    final double WHEEL_SPEED_LIMITED = 0.17;

    // Threshold where speed is reduced when slide is extended
    final double SLIDE_POSITION_THRESHOLD = 700;
    // Threshold where slide estension is limited with should down to stay within size limits
    final double SHOULDER_POSITION_THRESHOLD = 800;

    // set initial position ...
    double shoulderTarget = SHOULDER_MIN;
    double wristTarget = WRIST_UP;
    double clawTarget = CLAW_CLOSED;
    double slideTarget = SLIDE_MIN;
    double wheelSpeed = WHEEL_SPEED_MAX;
    // ... then set current state to match above position
    RobotStates currentState = RobotStates.HOME;
    RobotStates requestedState = RobotStates.HOME;


    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer
    private DcMotor BLeft;
    private DcMotor BRight;
    private DcMotor FLeft;
    private DcMotor FRight;
    private DcMotor slide;
    private DcMotor shoulder;
    private Servo wrist;
    private Servo claw;

    double oldTime = 0;


    public void moveRobot(){

        if(slide.getCurrentPosition()>SLIDE_POSITION_THRESHOLD){
            wheelSpeed = WHEEL_SPEED_LIMITED;
        }else{
            wheelSpeed = WHEEL_SPEED_MAX;
        }

        double forward = -gamepad1.left_stick_y * wheelSpeed; // (inverted Y-axis)
        double strafe = gamepad1.left_stick_x * wheelSpeed;
        double rotate = gamepad1.right_stick_x * wheelSpeed;

        if (gamepad1.right_trigger > 0.9){
            odo.resetPosAndIMU(); //resets the position to 0 and recalibrates the IMU
        }

        Pose2D pos = odo.getPosition();
        double heading =  pos.getHeading(AngleUnit.RADIANS);

        double cosAngle = Math.cos((Math.PI / 2)-heading);
        double sinAngle = Math.sin((Math.PI / 2)-heading);

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
        telemetry.addData("Robot XPos: ", pos.getX(DistanceUnit.MM));
        telemetry.addData("Robot YPos: ", pos.getY(DistanceUnit.MM));
        telemetry.addData("Robot Heading: ", heading);
        telemetry.addData("Forward Speed : ", globalForward);
        telemetry.addData("Strafe Speed : ", globalStrafe);

        telemetry.addData("Forward Speed : ", globalForward);
        telemetry.addData("Strafe Speed : ", globalStrafe);


        // Reference for positivity.
//        this.wFL = ((xVelo + yVelo) + (AXLE_CONSTANT * tVelo))/ WHEEL_RADIUS;// + + +
//        this.wFR = ((xVelo - yVelo) - (AXLE_CONSTANT * tVelo))/ WHEEL_RADIUS;// + - -
//        this.wBL = ((xVelo - yVelo) + (AXLE_CONSTANT * tVelo))/ WHEEL_RADIUS;// + - +
//        this.wBR = ((xVelo + yVelo) - (AXLE_CONSTANT * tVelo))/ WHEEL_RADIUS;// + + -





    }

    @Override
    public void init() {
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");

        BLeft = hardwareMap.get(DcMotor.class, "backleft");
        BRight = hardwareMap.get(DcMotor.class, "backright");
        FLeft = hardwareMap.get(DcMotor.class, "frontleft");
        FRight = hardwareMap.get(DcMotor.class, "frontright");
        slide = hardwareMap.get(DcMotor.class, "elevator");
        shoulder = hardwareMap.get(DcMotor.class, "arm");
        claw = hardwareMap.get(Servo.class, "claw");
        wrist = hardwareMap.get(Servo.class, "wrist");

        // set encoders
        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set brakes
        shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // reverse the motor directions
        BLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        FLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        slide.setDirection(DcMotorSimple.Direction.REVERSE);

        odo.setOffsets(-84.0, -168.0); //these are tuned for 3110-0002-0001 Product Insight #1

        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        //odo.setEncoderResolution(13.26291192);

        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        odo.resetPosAndIMU();

        Pose2D startingPosition= new Pose2D(DistanceUnit.MM,-923.925, 1601.47, AngleUnit.RADIANS, 0);
        odo.setPosition(startingPosition);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset());
        telemetry.addData("Y offset", odo.getYOffset());
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Device Scalar", odo.getYawScalar());
        telemetry.update();

        // Wait for the game to start (driver presses START)

        resetRuntime();

    }


    public void gamepadInput() {
        // Preset States
        if (gamepad2.right_bumper) {
            requestedState = RobotStates.HOME;
        }
        if (gamepad2.right_trigger > 0.5) {
            requestedState = RobotStates.SUB_1;
        }
        if (gamepad2.left_bumper) {
            requestedState = RobotStates.BASKET_1;
        }
        if (gamepad2.left_trigger > 0.5) {
            requestedState = RobotStates.BASKET_2;
        }
        if (gamepad2.a) {
            // Preset to grab block
            if (currentState == RobotStates.SUB_1) {
                requestedState = RobotStates.SUB_2;
            } else {
                // clawTarget = WRIST_DOWN;
            }
        }
        if (gamepad2.x) {
            // Preset to return to home
            if (currentState == RobotStates.BASKET_2) {
                requestedState = RobotStates.BASKET_3; // Same as home but with systematic process (ordering movements)
            } else {
                //clawTarget = CLAW_OPEN;
            }
        }
    }

    public void stateMachine() {
        switch (currentState) {
            case HOME:
                // immediate actions
                clawTarget = CLAW_CLOSED;

                if(timer.seconds() > 0.3){
                    wristTarget = WRIST_UP;
                }
                // delayed actions
                if (timer.seconds() > 0.2) {
                    shoulderTarget = SHOULDER_MIN;
                    slideTarget = SLIDE_MIN;
                }
                // allowed transistions from HOME: SUBMERSIBLE, BASKET_1
                if (requestedState == RobotStates.SUB_1){
                    currentState = RobotStates.SUB_1;
                    timer.reset();  // start delay timer for wrist movement
                } else if (requestedState == RobotStates.BASKET_1){
                    currentState = RobotStates.BASKET_1;
                    timer.reset();  // start delay timer for wrist movement
                }
                break;

            case SUB_1:
                // immediate actions
                clawTarget = CLAW_OPEN;
                shoulderTarget = SHOULDER_MIN;
                slideTarget = SLIDE_X_MAX;
                wristTarget = WRIST_CLIP;
                // allowed transistions from SUB: HOME, SUB_2
                if (requestedState == RobotStates.HOME){
                    currentState = RobotStates.HOME;
                    timer.reset();  // start delay timer for wrist movement
                } else if (requestedState == RobotStates.SUB_2){
                    currentState = RobotStates.SUB_2;
                    timer.reset();  // start delay timer for wrist movement
                }
                break;

            case SUB_2:
                // immediate actions
                wristTarget = WRIST_DOWN;
                shoulderTarget = SHOULDER_MIN;
                slideTarget = SLIDE_X_MAX;
                // delayed actoins
                if(timer.seconds() > 0.4){
                    clawTarget = CLAW_CLOSED;
                }
                if(timer.seconds() > 0.7){
                    currentState = RobotStates.SUB_3;
                    timer.reset();
                }
                // allowed transition
                break;


            case SUB_3:
                // immediate actions
                wristTarget = WRIST_UP;
                // allowed transition
                if (requestedState == RobotStates.HOME){
                    currentState = RobotStates.HOME;
                    timer.reset();  // start delay timer for wrist movement
                } else if (requestedState == RobotStates.SUB_1){
                    currentState = RobotStates.SUB_1;
                    timer.reset();  // start delay timer for wrist movement
                }
                break;

            case BASKET_1:
                // immediate actions
                shoulderTarget = SHOULDER_MAX;
                wristTarget = WRIST_DOWN;
                // delayed actions
                // allowed transistions
                if (timer.seconds() > 1.0) {
                    currentState = RobotStates.BASKET_2;
                    timer.reset();
                }
                break;

            case BASKET_2:
                slideTarget = SLIDE_Y_MAX;
                clawTarget = CLAW_CLOSED;

                if(timer.seconds() > 1.4){
                    wristTarget = WRIST_UP;
                }

                if(requestedState == RobotStates.BASKET_3){
                    currentState = RobotStates.BASKET_3;
                    timer.reset();
                }
                break;

            case BASKET_3:
                // immediate actions
                clawTarget = CLAW_OPEN;

                if(timer.seconds()> 0.3){
                    currentState = RobotStates.BASKET_4;
                    timer.reset();
                }
                break;

            case BASKET_4:
                wristTarget = WRIST_DOWN;

                if(timer.seconds()>0.8){
                    slideTarget = SLIDE_MIN;
                    shoulderTarget = SHOULDER_MAX;
                }

                if(timer.seconds()>1.4){
                    currentState = RobotStates.HOME;
                    timer.reset();
                }

                break;

            default:
                currentState = RobotStates.UNKNOWN;

                break;
        }

        telemetry.addData("State Machine", "Current state: %s", currentState);

    }

    public void moveWrist() {
        wrist.setPosition(wristTarget);
        // telemetry.addData("Wrist Current / Target ", "(%.2f)", wristTarget);
    }

    public void moveClaw() {
        claw.setPosition(clawTarget);
        // telemetry.addData("Claw Current / Target ", "(%.2f)", clawTarget);
    }

    public void moveShoulder() {
        shoulder.setTargetPosition((int)shoulderTarget);
        shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shoulder.setPower(Math.abs(SHOULDER_POWER));
        // telemetry.addData("Shoulder Current / Target ", "(%.2f, %.2f)", shoulder.getCurrentPosition(), shoulderTarget);
    }

    public void moveSlide() {
        slide.setTargetPosition((int)slideTarget);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(Math.abs(SLIDE_POWER));
        // telemetry.addData("Slide Current / Target ", "(%.2f, %.2f)", slide.getCurrentPosition(), slideTarget);
    }

    @Override
    public void loop() {
        moveRobot();
        gamepadInput();
        stateMachine();
        moveRobot();
        moveShoulder();
        moveSlide();
        moveWrist();
        moveClaw();

        telemetry.addData("Robot X: ", odo.getPosX());
        telemetry.addData("Robot X: ", odo.getPosY());
        Pose2D pos = odo.getPosition();
        telemetry.addData("Robot X: ", pos.getHeading(AngleUnit.DEGREES));
        telemetry.update();

        odo.update();
    }
}