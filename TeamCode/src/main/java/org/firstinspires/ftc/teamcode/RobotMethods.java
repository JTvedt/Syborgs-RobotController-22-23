package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.cv.CvPipeline;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

public class RobotMethods {
    public final ElapsedTime runtime = new ElapsedTime();

    public static final double BASE_POWER = 0.35;
    public static final double PULSES_PER_REVOLUTION = 537.7;
    public static final double WHEEL_CIRCUMFERENCE = 15.76;
    public static final double TICKS_PER_INCH = PULSES_PER_REVOLUTION / WHEEL_CIRCUMFERENCE;
    public static final double TICKS_PER_TILE = TICKS_PER_INCH * 24;
    public static final double TICKS_PER_CM = TICKS_PER_INCH * 2.54;

    public static final int WAIT_TIME = 400;
    public static final int TICK_THRESHOLD = 300;

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

    public DriveType mode = DriveType.POV;
    public DistanceUnit driveUnit = DistanceUnit.INCHES;
    public BNO055IMU imu;
    public double zeroAngle = 0;
    public boolean pinch = false; // true for gripped
    public boolean manualSlides = false;

    public CvPipeline pipeline;
    public OpenCvCamera camera;

    // Test method to send a message
    public RobotMethods(LinearOpMode parent, OpModeType type, Side side) {
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

        // Computer Vision
        if (type == OpModeType.AUTONOMOUS) {
            WebcamName webcamName = hardwareMap.get(WebcamName.class, "Camera");
            // TODO plug in numbers
            if (side == Side.LEFT) pipeline = new CvPipeline(0, 0, 1, 1);
            else if (side == Side.RIGHT) pipeline = new CvPipeline(0, 0, 1, 1);
            else pipeline = new CvPipeline(0, 0, 1, 1);
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
            camera.setPipeline(pipeline);
            camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode) {

                }
            });
        }

        // IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        resetAngle();

        while (!imu.isGyroCalibrated())
        {
            // Wait for callibration
            Thread.yield();
        }

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        parent.waitForStart();
        runtime.reset();
    }

    public RobotMethods(LinearOpMode parent, OpModeType type) {
        this(parent, type, Side.RIGHT);
    }

    public RobotMethods(LinearOpMode parent) {
        this(parent, OpModeType.AUTONOMOUS, Side.RIGHT);
    }

    public enum OpModeType {
        TELEOP, AUTONOMOUS;
    }

    public enum Side {
        LEFT, RIGHT;
    }

    /**
     * TeleOp drive function
     * @param angle angle to move
     * @param magnitude magnitude at which robot moves, can't be too much
     * @param turnPower amount the robot should spin
     */
    public void teleDrive(double angle, double magnitude, double turnPower, double multiplier) {
        if (this.mode == DriveType.POV) angle -= getAngle();
        double horizontal = Math.cos(angle);
        double vertical = Math.sin(angle) * 0.87;

        magnitude *= multiplier;
        turnPower *= multiplier;

        if (magnitude * (vertical + horizontal) > 1) magnitude = Math.abs(1/(horizontal + vertical));
        else if (magnitude * (vertical - horizontal) < -1) magnitude = Math.abs(1/(horizontal - vertical));

        frontLeft.setPower((vertical + horizontal) * magnitude + turnPower);
        frontRight.setPower((vertical - horizontal) * magnitude - turnPower);
        backLeft.setPower((vertical - horizontal) * magnitude + turnPower);
        backRight.setPower((vertical + horizontal) * magnitude - turnPower);
    }

    /**
     * Moves robot in a linear direction
     * @param distance Distance to travel, in inches
     * @param direction Direction of movement, in degrees
     */
    public void linearMove(double distance, double direction) {
        double radianAngle = direction * (Math.PI / 180) - ((mode == DriveType.POV ? getAngle() : 0));
        double horizontal = Math.cos(radianAngle);
        double vertical = Math.sin(radianAngle) * 0.87;
        int tickCount = toTicks(distance);

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setTargetPosition((int)((horizontal + vertical) * tickCount));
        frontRight.setTargetPosition((int)((-horizontal + vertical) * tickCount));
        backLeft.setTargetPosition((int)((-horizontal + vertical) * tickCount));
        backRight.setTargetPosition((int)((horizontal + vertical) * tickCount));

        setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);
        setDrivePower(BASE_POWER);

        while (isMoving()) {
            int diff = getMeanDifference();
            if (diff > TICK_THRESHOLD) setDrivePower(BASE_POWER);
            else setDrivePower(BASE_POWER - (1 - (double)diff/TICK_THRESHOLD) * 0.25);

            telemetry.addData("FL Difference:", frontLeft.getTargetPosition() - frontLeft.getCurrentPosition());
            telemetry.addData("FR Difference:", frontRight.getTargetPosition() - frontRight.getCurrentPosition());
            telemetry.addData("BL Difference:", backLeft.getTargetPosition() - backLeft.getCurrentPosition());
            telemetry.addData("BR Difference:", backRight.getTargetPosition() - backRight.getCurrentPosition());

            telemetry.addData("FL Power:", frontLeft.getPower());
            telemetry.addData("FR Power:", frontRight.getPower());
            telemetry.addData("BL Power:", backLeft.getPower());
            telemetry.addData("BR Power:", backRight.getPower());
            telemetry.update();
        }

        rest();
    }

    /**
     * Spins the robot either clockwise or counterclockwise
     * @param angle angle to turn in degrees, positive counterclockwise, negative clockwise
     */
    public void spin(double angle) {
        double radianAngle = angle * (Math.PI/180);
        int tickCount = (int)(radianAngle * 425);

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setTargetPosition(-tickCount);
        frontRight.setTargetPosition(tickCount);
        backLeft.setTargetPosition(-tickCount);
        backRight.setTargetPosition(tickCount);

        setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);
        setDrivePower(BASE_POWER);

        while (isMoving()) {
            setDrivePower(BASE_POWER);
        }

        rest();
    }

    // TODO finish this method
    public void spinDrive(double distance, double ddirection, double angle) {
        double radianDirection = angle * (Math.PI/180) - ((mode == DriveType.POV ? getAngle() : 0));
        double radianAngle = angle * (Math.PI/180);
        double horizontal = Math.cos(radianDirection);
        double vertical = Math.sin(radianDirection);

        int driveTicks = (int)(distance * TICKS_PER_INCH);
        int spinTicks = (int)(radianAngle * 425);

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setTargetPosition((int)((horizontal + vertical) * driveTicks) - spinTicks);
        frontRight.setTargetPosition((int)((-horizontal + vertical) * driveTicks) + spinTicks);
        backLeft.setTargetPosition((int)((-horizontal + vertical) * driveTicks) - spinTicks);
        backRight.setTargetPosition((int)((horizontal + vertical) * driveTicks) + spinTicks);

        int maxTicks = 0;
        for (DcMotor motor: wheelList) if (motor.getTargetPosition() > maxTicks) maxTicks = motor.getTargetPosition();

        setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);
        setDrivePower(BASE_POWER);

        while (isMoving()) {
            for (DcMotor motor : wheelList) motor.setPower(0.5 * (motor.getTargetPosition() / maxTicks));
        }

        rest();
    }

    // Drive robot distance (in) forward, negative distance for reverse
    public void drive(double distance) {
        linearMove(distance, 90);
    }

    // Strafe robot distance (in) to right, negative distance for left
    public void strafe(double distance) {
        linearMove(distance, 0);
    }

    // Moves the robot using taxicab distance (x, y)
    public void taxicabMove(double x, double y) {
        linearMove(Math.hypot(y, x), Math.atan2(y, x));
    }

    // Spin to a particular direction (degrees)
    public void spinTo(double newAngle) {
        spin(newAngle - (getAngle() * 180/Math.PI));
    }

    public void setDriveMode(DcMotor.RunMode runMode) {
        for (DcMotor motor: wheelList) motor.setMode(runMode);
    }

    public void setDrivePower(double power) {
        for (DcMotor motor : wheelList) motor.setPower(power);
    }

    public int getMeanDifference() {
        int buffer = 0;
        for (DcMotor motor : wheelList) buffer += Math.abs(motor.getTargetPosition() - motor.getCurrentPosition());
        return buffer/4;
    }

    // Sets motor power to 0 to stop the robot
    public void stopMovement() {
        setDrivePower(0);
    }

    // Sets the mode
    public void setDriveType(DriveType mode) {
        this.mode = mode;
    }

    public enum DriveType {
        POV, DIRECTIONAL;
    }

    public void setDriveUnit(DistanceUnit unit) {
        this.driveUnit = unit;
    }

    public enum DistanceUnit {
        INCHES, CENTIMETERS, TILES;
    }

    public int toTicks(double distance) {
        if (driveUnit == DistanceUnit.INCHES) return (int)(distance * TICKS_PER_INCH);
        else if (driveUnit == DistanceUnit.CENTIMETERS) return (int)(distance * TICKS_PER_CM);
        else if (driveUnit == DistanceUnit.TILES) return (int)(distance * TICKS_PER_TILE);
        else return 0;
    }

    public boolean isMoving(int mOfError) {
        return (getMeanDifference() / 4 > mOfError);
    }

    public boolean isMoving(int mOfError, int max) {
        int buffer = 0;
        for (DcMotor motor : wheelList) {
            if (Math.abs(motor.getTargetPosition() - motor.getCurrentPosition()) > max) return true;
            buffer += Math.abs(motor.getTargetPosition() - motor.getCurrentPosition());
        }
        return (buffer / 4 > mOfError);
    }

    public boolean isMoving() {
        return isMoving(16);
    }

    public void rest() {
        try {
            Thread.sleep(WAIT_TIME);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
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

    // Sets the slide to move to a position
    public void setSlides(int height) {
        double power = height < slideTarget() ? 0.85 : 0.5;

        manualSlides = false;
        leftSlide.setTargetPosition(height);
        rightSlide.setTargetPosition(height);

        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftSlide.setPower(power);
        rightSlide.setPower(power);
    }

    // Move the slides manually
    public void moveSlides(double power) {
        manualSlides = true;
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftSlide.setPower(power * (power < 0 ? 0.85 : 0.5));
        rightSlide.setPower(power * (power < 0 ? 0.85 : 0.5));
    }

    public void waitForSlides() {
        while (Math.abs(slidePosition() - slideTarget()) > 16) {
            telemetry.addData("Slide Difference", Math.abs(slidePosition() - slideTarget()));
        }

        rest();
    }

    public int slidePosition() {
        return (leftSlide.getCurrentPosition() + rightSlide.getCurrentPosition()) / 2;
    }

    public int slideTarget() {
        return leftSlide.getTargetPosition();
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

    public void setClaw(double pinch) {
        this.pinch = true;
        leftClaw.setPosition(1 - pinch);
        rightClaw.setPosition(pinch);
    }

    public void foo() {
        telemetry.addData("Foo", "Bar");
        telemetry.update();
    }
}