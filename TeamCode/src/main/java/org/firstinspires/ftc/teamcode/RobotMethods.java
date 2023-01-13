package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.ArrayList;

public class RobotMethods {
    public final ElapsedTime runtime = new ElapsedTime();

    public static final double BASE_POWER = 0.6;
    public static final double PULSES_PER_REVOLUTION = 537.7;
    public static final double WHEEL_CIRCUMFERENCE = 40.02;
    public static final double TICKS_PER_CM = PULSES_PER_REVOLUTION / WHEEL_CIRCUMFERENCE;

    public static final int WAIT_TIME = 300;

    public LinearOpMode parent;
    public HardwareMap hardwareMap;
    public Telemetry telemetry;

    public ArrayList<DcMotor> wheelList;
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;

    public DcMotor leftSlide;
    public DcMotor rightSlide;

    public Servo leftClaw;
    public Servo rightClaw;

    public BNO055IMU imu;
    public double zeroAngle = 0;
    public boolean pinch = false; // true for gripped

    // Test method to send a message
    public RobotMethods(LinearOpMode parent) {
        this.parent = parent;
        hardwareMap = parent.hardwareMap;
        telemetry = parent.telemetry;

        telemetry.addData ("Status", "...");
        telemetry.update();

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

        for (DcMotor motor : wheelList) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // Slide motors
        leftSlide = hardwareMap.get(DcMotor.class, "LS");
        rightSlide = hardwareMap.get(DcMotor.class, "RS");

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setDirection(DcMotor.Direction.REVERSE);

        // Servo
        leftClaw = hardwareMap.get(Servo.class, "LC");
        rightClaw = hardwareMap.get(Servo.class, "RC");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        resetAngle();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        parent.waitForStart();
        runtime.reset();
        toggleClaw(true);
    }

    /**
     * TeleOp drive function
     * @param angle angle to move
     * @param rawMagnitude magnitude at which robot moves, can't be too much
     * @param turnPower amount the robot should spin
     */
    public void teleDrive(double angle, double rawMagnitude, double turnPower) {
        double magnitude = Range.clip(rawMagnitude, -1.0, 1.0);
        double leftPower = Math.sin(angle) + Math.cos(angle);
        double rightPower = Math.sin(angle) - Math.cos(angle);

        if (Math.abs(leftPower) > 1) magnitude /= Math.abs(leftPower);
        if (Math.abs(rightPower) > 1) magnitude /= Math.abs(rightPower);

        frontLeft.setPower(magnitude * leftPower + turnPower);
        frontRight.setPower(magnitude * rightPower - turnPower);
        backLeft.setPower(magnitude * leftPower + turnPower);
        backRight.setPower(magnitude * rightPower - turnPower);
    }

    public void teleDrive(double drive, double strafe, double turn, double magnitude) {
        frontLeft.setPower((drive + strafe + turn) * magnitude);
        frontRight.setPower((drive - strafe - turn) * magnitude);
        backLeft.setPower((drive - strafe + turn) * magnitude);
        backRight.setPower((drive + strafe - turn) * magnitude);
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
            telemetry.addData("FL Difference:", frontLeft.getTargetPosition() - frontLeft.getCurrentPosition());
            telemetry.addData("FR Difference:", frontRight.getTargetPosition() - frontRight.getCurrentPosition());
            telemetry.addData("BL Difference:", backLeft.getTargetPosition() - backLeft.getCurrentPosition());
            telemetry.addData("BR Difference:", backRight.getTargetPosition() - backRight.getCurrentPosition());

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

    // Drive robot distance (cm) forward, negative distance for reverse
    public void drive(double distance) {
        linearMove(distance, 90);
    }

    // Strafe robot distance (cm) to right, negative distance for left
    public void strafe(double distance) {
        linearMove(distance, 0);
    }
    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotor motor: wheelList) motor.setMode(runMode);
    }

    public void setPower(double power) {
        for (DcMotor motor : wheelList) motor.setPower(power * BASE_POWER);
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
        double rawAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
        double netAngle = rawAngle - zeroAngle;

        if (netAngle < -Math.PI) netAngle += 2 * Math.PI;
        if (netAngle > Math.PI) netAngle -= 2 * Math.PI;

        return netAngle;
    }

    // Sets the zeroAngle of the robot to where it is currently facing
    public void resetAngle() {
        zeroAngle = getAngle();

        if (zeroAngle < -Math.PI) zeroAngle += 2 * Math.PI;
        if (zeroAngle > Math.PI) zeroAngle -= 2 * Math.PI;
    }

    // Moves the slides up and down
    public void setSlides(int height) {
        leftSlide.setTargetPosition(height);
        rightSlide.setTargetPosition(height);

        leftSlide.setPower(height < leftSlide.getCurrentPosition() ? 0.85 : 0.5);
        rightSlide.setPower(height < rightSlide.getCurrentPosition() ? 0.85 : 0.5);

        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void waitForSlides() {
        while (leftSlide.isBusy() && rightSlide.isBusy()) {

        }
    }

    // toggles the claw, between gripped and un-gripped
    public void toggleClaw() {
        pinch = !pinch;
        leftClaw.setPosition(pinch ? 0.7 : 1.0);
        rightClaw.setPosition(pinch ? 0.3 : 0.0);
    }

    // true to close, false to open claw
    public void toggleClaw(boolean state) {
        pinch = !state;
        toggleClaw();
    }

    public void foo() {
        telemetry.addData("Foo", "Bar");
        telemetry.update();
    }
}
