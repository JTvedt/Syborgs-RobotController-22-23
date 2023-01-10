package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.ArrayList;

@Autonomous(name="Autonomous Base")
public class AutonomousBase extends LinearOpMode {
    public final ElapsedTime runtime = new ElapsedTime();

    public static final double BASE_POWER = 0.35;
    public static final double MAX_POWER = 0.8;
    public static final double TILE_CM = 60;

    public static final double PULSES_PER_REVOLUTION = 537.7;
    public static final double WHEEL_CIRCUMFERENCE = 40.02;
    public static final double TICKS_PER_CM = PULSES_PER_REVOLUTION / WHEEL_CIRCUMFERENCE;

    public static final int WAIT_TIME = 300;

    public ArrayList<DcMotor> wheelList;
    public ArrayList<DcMotor> slideList;

    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;

    public DcMotor leftSlide;
    public DcMotor rightSlide;

    public Servo leftClaw;
    public Servo rightClaw;

    public BNO055IMU imu;
    public double initialAngle = 0;

    @Override
    public void runOpMode() {
        initialize();

        // Autonomous processes go here
        telemetry.addData("Angle: ", getAngle());
        telemetry.update();
        sleep(1000);
        telemetry.addData("Angle: ", getAngle());
        telemetry.update();
        sleep(1000);
    }

    public double foo() {
        return 3.141;
    }

    public void initialize() {
        // Store wheels in list for iteration purposes
        wheelList = new ArrayList<DcMotor>();
        wheelList.add(hardwareMap.get(DcMotor.class, "FL"));
        wheelList.add(hardwareMap.get(DcMotor.class, "FR"));
        wheelList.add(hardwareMap.get(DcMotor.class, "BL"));
        wheelList.add(hardwareMap.get(DcMotor.class, "BR"));

        frontLeft = hardwareMap.get(DcMotor.class, "FL");
        frontRight = hardwareMap.get(DcMotor.class, "FR");
        backLeft = hardwareMap.get(DcMotor.class, "BL");
        backRight = hardwareMap.get(DcMotor.class, "BR");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // Slide motors
        slideList = new ArrayList<DcMotor>();
        slideList.add(hardwareMap.get(DcMotor.class, "LS"));
        slideList.add(hardwareMap.get(DcMotor.class, "RS"));

        leftSlide = hardwareMap.get(DcMotor.class, "LS");
        rightSlide = hardwareMap.get(DcMotor.class, "RS");

        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setDirection(DcMotor.Direction.REVERSE);

        // Servo
        // leftClaw = hardwareMap.get(Servo.class, "LC");
        // rightClaw = hardwareMap.get(Servo.class, "RC");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();
    }

    /**
     * Moves robot in a linear direction
     * @param distance Distance to travel, in centimeters
     * @param direction Direction of movement, in degrees
     */
    public void linearMove(double distance, double direction) {
        double radianAngle = direction * (Math.PI / 180);
        double horizontal = Math.cos(radianAngle);
        double vertical = Math.sin(radianAngle) * 0.87;
        int tickCount = (int)(distance * TICKS_PER_CM);

        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setTargetPosition((int)((horizontal + vertical) * tickCount));
        frontRight.setTargetPosition((int)((-horizontal + vertical) * tickCount));
        backLeft.setTargetPosition((int)((-horizontal + vertical) * tickCount));
        backRight.setTargetPosition((int)((horizontal + vertical) * tickCount));

        setMode(DcMotor.RunMode.RUN_TO_POSITION);
        setPower(1.0);

        while (isMoving()) {
            // Wait for robot to stop
        }

        try {
            Thread.sleep(WAIT_TIME);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    /**
     * Spins the robot either clockwise or counterclockwise
     * @param angle angle to turn in degrees, positive counterclockwise, negative clockwise
     */
    public void spin(double angle) {
        double radianAngle = angle * (Math.PI/180);
        int tickCount = (int)(radianAngle * 425);

        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setTargetPosition(-tickCount);
        frontRight.setTargetPosition(tickCount);
        backLeft.setTargetPosition(-tickCount);
        backRight.setTargetPosition(tickCount);

        setMode(DcMotor.RunMode.RUN_TO_POSITION);
        setPower(1.0);

        while (isMoving()) {
            // Wait for movement to finish
        }
    }

    /**
     * Moves robot forwards, negative distance for backward
     * @param distance cm moved forward
     */
    public void drive(double distance) {
        linearMove(distance, 90);
    }

    /**
     * Strafes the robot to the right, negative distance for left
     * @param distance cm moved to the right
     */
    public void strafe(double distance) {
        linearMove(distance, 0);
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotor motor: wheelList) {
            motor.setMode(runMode);
        }
    }

    public void setPower(double power) {
        for (DcMotor motor : wheelList) {
            motor.setPower(power * BASE_POWER);
        }
    }

    // Sets motor power to 0 to stop the robot
    public void stopMovement() {
        setPower(0);
    }

    /**
     * Checks the current state of the robot to determine if should keep moving
     * @return true if any motors are busy.
     */
    public boolean isMoving() {
        for (DcMotor motor : wheelList) {
            if (motor.isBusy()) return true;
        }

        return false;
    }

    // Gets the degree value as a radian value
    public double getAngle() {
        double imuAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        // if (imuAngle < -Math.PI) imuAngle += 2 * Math.PI;
        // if (imuAngle > Math.PI) imuAngle -= 2 * Math.PI;
        return imuAngle;
    }

    // Sets the initialAngle of the robot to where it is currently facing
    public void resetAngle() {
        initialAngle = getAngle();
    }

    /**
     * Sets the claw to a position
     * @param state state to put the claw in
     */
    public void setClaw(ClawState state) {
        if (state == ClawState.OPEN) {
            leftClaw.setPosition(0.1);
            rightClaw.setPosition(0.9);
        } else if (state == ClawState.CLOSE) {
            leftClaw.setPosition(0.9);
            rightClaw.setPosition(0.1);
        }
    }

    public enum ClawState {
        OPEN, CLOSE
    }
}
