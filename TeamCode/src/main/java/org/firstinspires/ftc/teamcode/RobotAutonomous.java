package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.ArrayList;

@Disabled
@Autonomous(name="Autonomous Base")
public class RobotAutonomous extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();

    private static final double BASE_POWER = 0.35;
    private static final double MAX_POWER = 0.8;
    private static final double TILE_CM = 60;
    private static final int WAIT_TIME = 300;

    private static final double PULSES_PER_REVOLUTION = 537.7;
    private static final double WHEEL_CIRCUMFERENCE = 40.02;
    private static final double TICKS_PER_CM = PULSES_PER_REVOLUTION / WHEEL_CIRCUMFERENCE;

    private ArrayList<DcMotor> wheelList;
    private ArrayList<DcMotor> slideList;

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private DcMotor leftSlide;
    private DcMotor rightSlide;

    private BNO055IMU imu;
    private double initialAngle = 0;

    @Override
    public void runOpMode() {
        initialize();
        // Autonomous processes go here

        telemetry.addData("Angle: ", getAngle());
        spin(90);
        telemetry.addData("Angle: ", getAngle());
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

        // Slide motors
        slideList = new ArrayList<DcMotor>();
        slideList.add(hardwareMap.get(DcMotor.class, "LS"));
        slideList.add(hardwareMap.get(DcMotor.class, "RS"));

        leftSlide = hardwareMap.get(DcMotor.class, "LS");
        rightSlide = hardwareMap.get(DcMotor.class, "RS");

        // Reverse left side motors
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
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

    private void setMode(DcMotor.RunMode runMode) {
        for (DcMotor motor: wheelList) {
            motor.setMode(runMode);
        }
    }

    private void setPower(double power) {
        for (DcMotor motor : wheelList) {
            motor.setPower(power * BASE_POWER);
        }
    }

    // Sets motor power to 0 to stop the robot
    private void stopMovement() {
        setPower(0);
    }

    /**
     * Checks the current state of the robot to determine if should keep moving
     * @return true if any motors are busy.
     */
    private boolean isMoving() {
        for (DcMotor motor : wheelList) {
            if (motor.isBusy()) return true;
        }

        return false;
    }

    private double getAngle() {
        double imuAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - initialAngle;
        if (imuAngle < -180) imuAngle += 360;
        if (imuAngle > 180) imuAngle -= 360;
        return imuAngle;
    }

    private void resetAngle() {
        initialAngle = getAngle();
    }
}