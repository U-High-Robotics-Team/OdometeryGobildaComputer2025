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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous(name="DoubleBasketPark")
//@Disabled

public class DoubleBasketPark extends OpMode {

    RobotTarget[] targets = {
            new RobotTarget(-1405, -1405, Math.PI / 4, 4, RobotStates.BASKET_1),
            new RobotTarget(-1405, -1405, Math.PI / 4, 1.5, RobotStates.BASKET_3),
            new RobotTarget(-1190, -1250, Math.PI / 2, 2.3, RobotStates.SUB_1),
            new RobotTarget(-1190, -1250, Math.PI / 2, 1.4, RobotStates.SUB_2),
            new RobotTarget(-1405, -1405, Math.PI / 4, 1.2, RobotStates.HOME),
            new RobotTarget(-1405, -1405, Math.PI / 4, 3.5, RobotStates.BASKET_1),
            new RobotTarget(-1405, -1405, Math.PI / 4, 2.5, RobotStates.BASKET_3),
            new RobotTarget(-1450, -1250, Math.PI / 2, 2.2, RobotStates.SUB_1),
            new RobotTarget(-1450, -1250, Math.PI / 2, 1.4, RobotStates.SUB_2),
            new RobotTarget(-1405, -1405, Math.PI / 4, 1.2, RobotStates.HOME),
            new RobotTarget(-1405, -1405, Math.PI / 4, 3.7, RobotStates.BASKET_1),
            new RobotTarget(-1405, -1405, Math.PI / 4, 2.4, RobotStates.BASKET_3),
            new RobotTarget(1150, -1420, Math.PI / 4, 10, RobotStates.HOME)
            // new RobotTarget(0, 0, 0, 3, RobotState.HOME) // Example of adding multiple targets
    };

    // RobotTarget[] targets = {
    //         new RobotTarget(-1405, -1405, Math.PI / 4, 8, RobotStates.BASKET_1),
    //         new RobotTarget(-1405, -1405, Math.PI / 4, 8, RobotStates.BASKET_3),
    //         new RobotTarget(-1190, -1250, Math.PI / 2, 8, RobotStates.SUB_1),
    //         new RobotTarget(-1190, -1250, Math.PI / 2, 8, RobotStates.SUB_2),
    //         new RobotTarget(-1405, -1405, Math.PI / 4, 8, RobotStates.HOME),
    //         new RobotTarget(-1405, -1405, Math.PI / 4, 8, RobotStates.BASKET_1),
    //         new RobotTarget(-1405, -1405, Math.PI / 4, 8, RobotStates.BASKET_3),
    //         new RobotTarget(-1430, -1250, Math.PI / 2, 8, RobotStates.SUB_1),
    //         new RobotTarget(-1430, -1250, Math.PI / 2, 8, RobotStates.SUB_2),
    //         new RobotTarget(-1405, -1405, Math.PI / 4, 8, RobotStates.HOME),
    //         new RobotTarget(-1405, -1405, Math.PI / 4, 8, RobotStates.BASKET_1),
    //         new RobotTarget(-1405, -1405, Math.PI / 4, 8, RobotStates.BASKET_3),
    //         new RobotTarget(1150, -1420, Math.PI / 4, 10, RobotStates.HOME)
    //         // new RobotTarget(0, 0, 0, 3, RobotState.HOME) // Example of adding multiple targets
    // };


    boolean isTeleOp = false;

    // Timer for Servos
    private ElapsedTime presetTimer = new ElapsedTime();

    // Timer used for exiting early if needed
    ElapsedTime timer = new ElapsedTime();

    // Performance constants
    final int SLIDE_Y_MAX = 3000;
    final int SLIDE_X_MAX = 1500; // Maximum position (top)
    final int SLIDE_MIN = 0; // Minimum position (bottom)
    final double SLIDE_POWER = 1;
    final int SHOULDER_MAX = 1400;
    final int SHOULDER_MIN = 0;
    final double SHOULDER_POWER = 0.6;
    final double WRIST_UP = 0;
    final double WRIST_DOWN = 0.65;
    final double WRIST_CLIP = 0.3; // unused currently
    final double CLAW_OPEN = 0.6;
    final double CLAW_CLOSED = 0.25;
    final double WHEEL_SPEED_MAX = 1;
    final double WHEEL_SPEED_LIMITED = 0.17;

    // Thresholds
    final double SLIDE_POSITION_THRESHOLD =  400;
    final double SHOULDER_POSITION_THRESHOLD = 500;

    // Initial Targets
    RobotStates currentState = RobotStates.HOME;
    RobotStates requestedState = RobotStates.HOME;
    double shoulderTarget = SHOULDER_MIN;
    double wristTarget = WRIST_UP;
    double clawTarget = CLAW_CLOSED;
    double slideTarget = SLIDE_MIN;
    double wheelSpeed = WHEEL_SPEED_MAX;

    enum RobotStates {
        HOME,
        SUB_1,
        SUB_2,
        SUB_3,
        BASKET_1,
        BASKET_2,
        BASKET_3,
        BASKET_4,
        UNKNOWN             // when moved manually into another pose
    }


    // Initializing hardware objects

    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer
    private DcMotor BLeft;
    private DcMotor BRight;
    private DcMotor FLeft;
    private DcMotor FRight;
    private DcMotor slide;
    private DcMotor shoulder;
    private Servo wrist;
    private Servo claw;
    double addX = 0;
    double addY = 0;
    double addTheta = 0;

    double kP = 0.0022; // bigger the error the faster we will fix it
    double kI = 0.00013; // provides extra boost when you get close to the target
    double kD = 0.00015; // dampens overshoot

    private double integralSum = 0;
    double lastError = 0;
    double targetDuration = 0;

    private int currentTargetIndex = 0; // Keeps track of the current target

    @Override
    public void loop() {
        if (currentTargetIndex < targets.length) {
            RobotTarget target = targets[currentTargetIndex];
            this.targetDuration = target.time;
            requestedState = target.state;

            if (timer.seconds() < targetDuration) {
                moveRobotTo(target.x, target.y, target.heading);
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

        // updating odo positions
        odo.update();

        // getting current positions
        Pose2D currentPosition = odo.getPosition();
        double currentX = currentPosition.getX(DistanceUnit.MM);
        double currentY = currentPosition.getY(DistanceUnit.MM);
        double currentHeading = currentPosition.getHeading(AngleUnit.RADIANS);

        telemetry.addData("Current X", currentX);
        telemetry.addData("Current Y", currentY);
        telemetry.addData("Current Heading", currentHeading);

    }

    @Override
    public void start(){
        timer = new ElapsedTime();
        presetTimer = new ElapsedTime();
    }


    @Override
    public void init() {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        BLeft = hardwareMap.get(DcMotor.class, "backleft");
        BRight = hardwareMap.get(DcMotor.class, "backright");
        FLeft = hardwareMap.get(DcMotor.class, "frontleft");
        FRight = hardwareMap.get(DcMotor.class, "frontright");
        slide = hardwareMap.get(DcMotor.class, "elevator");
        shoulder = hardwareMap.get(DcMotor.class, "arm");
        claw = hardwareMap.get(Servo.class, "claw");
        wrist = hardwareMap.get(Servo.class, "wrist");

        // Setting encoders
        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set brakes
        shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reverse the motor directions
        BLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        FLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        slide.setDirection(DcMotorSimple.Direction.REVERSE);

        odo.setOffsets(-84.0, -168.0); // These offsets are configured for your robot
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        //odo.resetPosAndIMU();
        //try {
        //Thread.sleep(500);
        //} catch(InterruptedException e){
        //    }
        // Set your desired starting position
        Pose2D startingPosition = new Pose2D(DistanceUnit.MM, -871.5375, -1558.925, AngleUnit.RADIANS, 0);
        odo.setPosition(startingPosition);

        odo.update();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset());
        telemetry.addData("Y offset", odo.getYOffset());
        telemetry.addData("Current X", startingPosition.getX(DistanceUnit.MM));
        telemetry.addData("Current Y", startingPosition.getY(DistanceUnit.MM));
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Current Heding", startingPosition.getHeading(AngleUnit.RADIANS));
        telemetry.addData("Device Scalar", odo.getYawScalar());
        telemetry.update();


    }

    public void moveRobotTo(double targetX, double targetY, double targetHeading) {
        odo.update();

        // Getting current positions
        Pose2D currentPosition = odo.getPosition();
        double currentX = currentPosition.getX(DistanceUnit.MM);
        double currentY = currentPosition.getY(DistanceUnit.MM);
        double currentHeading = currentPosition.getHeading(AngleUnit.RADIANS);

        if(slide.getCurrentPosition() > SLIDE_X_MAX){
            wheelSpeed = WHEEL_SPEED_LIMITED;
        }else{
            wheelSpeed = WHEEL_SPEED_MAX;
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

        // Using proportional controller for power
        // double xPower = Math.max(Math.min(findPIDPower(deltaX), 1), -1);
        // double yPower = Math.max(Math.min(findPIDPower(deltaY), 1), -1);


        double xPower = deltaX * kP;
        double yPower = deltaY * kP;
        double turnPower = -deltaHeading;

        //double turnPower = -Math.max(Math.min(findPower(deltaHeading), 1), -1);


        // double xPower = deltaX * kP;
        // double yPower = deltaY * kP;
        // double turnPower = -Math.toDegrees(deltaHeading) * 0.09;



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

        FLeft.setPower(frontLeft);
        FRight.setPower(frontRight);
        BLeft.setPower(backLeft);
        BRight.setPower(backRight);

        telemetry.addData("Turn Power: ", turnPower);
        telemetry.addData("CurrentX: ", currentX);
        telemetry.addData("CurrentY: ", currentY);
        telemetry.addData("Current Heading: ", currentHeading);
        telemetry.addData("DeltaX", deltaX);
        telemetry.addData("DeltaY", deltaY);
        telemetry.addData("DeltaHeading", deltaHeading);
        telemetry.update();
    }

    public void stopMotors() {
        FLeft.setPower(0);
        FRight.setPower(0);
        BLeft.setPower(0);
        BRight.setPower(0);
    }

    public void moveWrist() {
        wrist.setPosition(wristTarget);
    }

    public void moveClaw() {
        claw.setPosition(clawTarget);
    }

    public void moveShoulder() {
        shoulder.setTargetPosition((int) shoulderTarget);
        shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shoulder.setPower(Math.abs(SHOULDER_POWER));
    }

    public void moveSlide() {
        slide.setTargetPosition((int) slideTarget);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(Math.abs(SLIDE_POWER));
    }

    // state machine



    public void stateMachine() {
        switch (currentState) {
            case HOME:
                // immediate actions
                clawTarget = CLAW_CLOSED;

                if(presetTimer.seconds() > 0.3){
                    wristTarget = WRIST_UP;
                }
                // delayed actions
                if (presetTimer.seconds() > 0.2) {
                    shoulderTarget = SHOULDER_MIN;
                    slideTarget = SLIDE_MIN;
                }
                // allowed transistions from HOME: SUBMERSIBLE, BASKET_1
                if (requestedState == RobotStates.SUB_1){
                    currentState = RobotStates.SUB_1;
                    presetTimer.reset();  // start delay timer for wrist movement
                } else if (requestedState == RobotStates.BASKET_1){
                    currentState = RobotStates.BASKET_1;
                    presetTimer.reset();  // start delay timer for wrist movement
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
                    presetTimer.reset();  // start delay timer for wrist movement
                } else if (requestedState == RobotStates.SUB_2){
                    currentState = RobotStates.SUB_2;
                    presetTimer.reset();  // start delay timer for wrist movement
                }
                break;

            case SUB_2:
                // immediate actions
                wristTarget = WRIST_DOWN;
                shoulderTarget = SHOULDER_MIN;
                slideTarget = SLIDE_X_MAX;
                // delayed actoins
                if(presetTimer.seconds() > 0.4){
                    clawTarget = CLAW_CLOSED;
                }
                if(presetTimer.seconds() > 0.7){
                    currentState = RobotStates.SUB_3;
                    presetTimer.reset();
                }
                // allowed transition
                break;


            case SUB_3:
                // immediate actions
                wristTarget = WRIST_UP;
                // allowed transition
                if (requestedState == RobotStates.HOME){
                    currentState = RobotStates.HOME;
                    presetTimer.reset();  // start delay timer for wrist movement
                } else if (requestedState == RobotStates.SUB_1){
                    currentState = RobotStates.SUB_1;
                    presetTimer.reset();  // start delay timer for wrist movement
                }
                break;

            case BASKET_1:
                // immediate actions
                shoulderTarget = SHOULDER_MAX;
                wristTarget = WRIST_DOWN;
                // delayed actions
                // allowed transistions
                if (presetTimer.seconds() > 1.0) {
                    currentState = RobotStates.BASKET_2;
                    presetTimer.reset();
                }
                break;

            case BASKET_2:
                slideTarget = SLIDE_Y_MAX;
                clawTarget = CLAW_CLOSED;

                if(presetTimer.seconds() > 1.4){
                    wristTarget = WRIST_UP;
                }

                if(requestedState == RobotStates.BASKET_3){
                    currentState = RobotStates.BASKET_3;
                    presetTimer.reset();
                }
                break;

            case BASKET_3:
                // immediate actions
                clawTarget = CLAW_OPEN;

                if(presetTimer.seconds()> 0.3){
                    currentState = RobotStates.BASKET_4;
                    presetTimer.reset();
                }
                break;

            case BASKET_4:
                wristTarget = WRIST_DOWN;

                if(presetTimer.seconds()>0.8){
                    slideTarget = SLIDE_MIN;
                    shoulderTarget = SHOULDER_MAX;
                }

                if(presetTimer.seconds()>1.4){
                    currentState = RobotStates.HOME;
                    presetTimer.reset();
                }

                break;

            default:
                currentState = RobotStates.UNKNOWN;

                break;
        }
        Pose2D currentPosition = odo.getPosition();
        double currentX = currentPosition.getX(DistanceUnit.MM);
        double currentY = currentPosition.getY(DistanceUnit.MM);
        double currentHeading = currentPosition.getHeading(AngleUnit.RADIANS);

        telemetry.addData("State Machine", "Current state: %s", currentState);
        telemetry.addData("CurrentX: ", currentX);
        telemetry.addData("CurrentY: ", currentY);
        telemetry.addData("Current Heading: ", currentHeading);
    }

    public double findPIDPower(double delta){
        double derivative = 0;
        double out = 0;
        double error = 0;

        error = delta;

        // rate of change of the error
        derivative = (delta - lastError) / timer.seconds();

        // sum of all error over time
        out = (kP * delta) + (kD * derivative);

        this.lastError = delta;

        return out;
    }
}
