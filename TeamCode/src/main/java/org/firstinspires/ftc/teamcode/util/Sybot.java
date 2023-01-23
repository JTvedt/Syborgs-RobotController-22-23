package org.firstinspires.ftc.teamcode.util;

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

/**
 * Class containing methods for everything on the robot
 * responsible for driving, subsystems, and everything
 * @author Jeffrey Tvedt
 */
public class Sybot {
    public final ElapsedTime runtime = new ElapsedTime();

    public static final double BASE_POWER = 0.35;
    public static final double PULSES_PER_REVOLUTION = 537.7;
    public static final double WHEEL_CIRCUMFERENCE = 15.76;
    public static final double TICKS_PER_INCH = PULSES_PER_REVOLUTION / WHEEL_CIRCUMFERENCE;
    public static final double TICKS_PER_TILE = TICKS_PER_INCH * 24;
    public static final double TICKS_PER_CM = TICKS_PER_INCH * 2.54;
    public static final double SLIDE_SPEED_UP = 0.95;
    public static final double SLIDE_SPEED_DOWN = 0.6;

    public static final int WAIT_TIME = 400;
    public static final int TICK_THRESHOLD = 300;
    public static int SLIDE_THRESHOLD = -1120;

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

    public DriveType driveType = DriveType.POV;
    public DistanceUnit driveUnit = DistanceUnit.INCHES;
    public BNO055IMU imu;
    public double zeroAngle = 0;
    public boolean pinch = false; // true for gripped
    public boolean manualSlides = false;
    public boolean mirror = false;
    public boolean enableThreads = true;
    public volatile int slideHeight = 0;

    public CvPipeline pipeline;
    public OpenCvCamera camera;

    public ArrayList<Thread> threadQueue = new ArrayList<Thread>();

    /**
     * Constructor for Sybot, takes in the parent, OpMode type, and starting side
     * Starting side is only relevant for autonomous
     * @param parent The OpMode that will be using this class's methods
     * @param type The type of OpMode, Autonomous or TeleOp
     * @param side Starting position of the robot, relevant for autonomous.
     *             If placed on the left side, strafing will be mirrored by default
     */
    public Sybot(LinearOpMode parent, OpModeType type, StartSide side) {
        this.parent = parent;
        hardwareMap = parent.hardwareMap;
        telemetry = parent.telemetry;

        telemetry.addData("Status", "...");
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
            // CV + Camera
            WebcamName webcamName = hardwareMap.get(WebcamName.class, "Camera");
            // TODO plug in numbers
            if (side == StartSide.LEFT) pipeline = new CvPipeline(0, 0, 1, 1);
            else if (side == StartSide.RIGHT) pipeline = new CvPipeline(0, 0, 1, 1);
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
            // Wait for calibration
            Thread.yield();
        }

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        parent.waitForStart();
        runtime.reset();
    }

    /**
     * Constructor that does not require side
     * @param parent The OpMode that will be using this class's methods
     * @param type The type of OpMode, TeleOp or Autonomous
     */
    public Sybot(LinearOpMode parent, OpModeType type) {
        this(parent, type, StartSide.UNSPECIFIED);
    }

    /**
     * Constructor that takes in the parent opmode
     * @param parent The OpMode that will be using this class's methods
     */
    public Sybot(LinearOpMode parent) {
        this(parent, OpModeType.AUTONOMOUS, StartSide.UNSPECIFIED);
    }

    public enum OpModeType {
        TELEOP, AUTONOMOUS;
    }

    /**
     * Starting position of the robot, on the left side or right side.
     * Left side will mirror the robot
     */
    public enum StartSide {
        LEFT, RIGHT, UNSPECIFIED;
    }

    /**
     * TeleOp drive function
     * @param direction Direction in which the robot will move, measured in degrees
     * @param magnitude magnitude at which robot moves, roughly a measure of velocity
     * @param turnPower in which direction the robot should rotate and how much,
     *                  positive values to turn clockwise and negative valeus to turn counterclockwise
     */
    public void teleDrive(double direction, double magnitude, double turnPower, double multiplier) {
        if (this.driveType == DriveType.POV) direction -= getAngle();
        double horizontal = Math.cos(direction);
        double vertical = Math.sin(direction) * 0.87;

        magnitude *= multiplier;
        turnPower *= multiplier;

        if (magnitude * (vertical + horizontal) > 1) magnitude = Math.abs(1/(horizontal + vertical));
        else if (magnitude * (vertical - horizontal) < -1) magnitude = Math.abs(1/(horizontal - vertical));

        frontLeft.setPower((vertical + horizontal) * magnitude + turnPower);
        frontRight.setPower((vertical - horizontal) * magnitude - turnPower);
        backLeft.setPower((vertical - horizontal) * magnitude + turnPower);
        backRight.setPower((vertical + horizontal) * magnitude - turnPower);
    }

    public void teleDrive(double direction, double magnitude, double turnPower) {
        teleDrive(direction, magnitude, turnPower, 1);
    }

    /**
     * Moves robot in a linear direction.
     * Mirrored on left side by default
     * @param distance Distance to travel based on the distance unit
     * @param direction Direction of movement, in degrees, 0 degrees to go right
     */
    public void polarMove(double distance, double direction) {
        double radianAngle = direction * (Math.PI / 180) - (driveType == DriveType.POV ? getAngle() : 0);
        double horizontal = Math.cos(radianAngle) * (mirror ? -1 : 1);
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
     * @param angle angle to turn in degrees, positive values turn counterclockwise and negative values turn clockwise
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
    // NOT TESTED DO NOT USE
    public void spinDrive(double distance, double direction, double angle) {
        double radianDirection = direction * (Math.PI/180) - getAngle();
        double radianAngle = angle * (Math.PI/180);
        double horizontal = Math.cos(radianDirection);
        double vertical = Math.sin(radianDirection) * 0.87;

        int driveTicks = (int)(distance * TICKS_PER_INCH);
        int spinTicks = (int)(radianAngle * 425);
        double spin = Math.signum(spinTicks) * 0.2;

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setTargetPosition((int)((vertical + horizontal) * driveTicks) - spinTicks);
        frontRight.setTargetPosition((int)((vertical - horizontal) * driveTicks) + spinTicks);
        backLeft.setTargetPosition((int)((vertical - horizontal) * driveTicks) - spinTicks);
        backRight.setTargetPosition((int)((vertical + horizontal) * driveTicks) + spinTicks);

        while (isMoving()) {
            radianDirection = direction * (Math.PI/180) - getAngle();
            horizontal = Math.cos(radianDirection);
            vertical = Math.sin(radianDirection) * 0.87;

            // Update power to move in a straight line
            frontLeft.setPower((horizontal + vertical) * 0.65);
            frontRight.setPower((horizontal - vertical) * 0.65);
            backLeft.setPower((horizontal - vertical) * 0.65);
            backRight.setPower((horizontal + vertical) * 0.65);
        }
    }

    /**
     * Moves the robot either forwards or backwards
     * @param distance distance for the robot to move, positive values to move forward, negative values to move backward
     */
    public void drive(double distance) {
        polarMove(distance, 90);
    }

    /**
     * Moves the robot to either side.
     * Moves independent of robot angle by default.
     * Mirrored on left side by default.
     * @param distance distance for the robot to move, positive values to move to the right, negative values to move to the left.
     */
    public void strafe(double distance) {
        polarMove(distance, 0);
    }

    /**
     * Drives the robot in a particular direction using cartesian coordinates.
     * Mirrored on left side by default.
     * @param x The amount the robot should move left and right
     * @param y The amount the robot should move forward
     */
    public void cartesianMove(double x, double y) {
        polarMove(Math.hypot(y, x), Math.atan2(y, x));
    }

    /**
     * NOT TESTED DO NOT USE
     * Spins the robot to a new position based on its current one
     * @param newAngle The new angle to spin to relative to old angle
     */
    public void spinTo(double newAngle) {
        // TODO make this method work consistently
        spin(newAngle - (getAngle() * 180/Math.PI));
    }

    /**
     * Sets the mode of all wheels of the robot
     * @param runMode the run mode to be applied to all wheels
     */
    public void setDriveMode(DcMotor.RunMode runMode) {
        for (DcMotor motor: wheelList) motor.setMode(runMode);
    }

    /**
     * Sets the power to all four wheels
     * @param power the power applied to all wheels
     */
    public void setDrivePower(double power) {
        for (DcMotor motor : wheelList) motor.setPower(power);
    }

    /**
     * Gets the average ticks remaining on each wheel.
     * Always returns a positive value
     * @return The average ticks remaining
     */
    public int getMeanDifference() {
        int buffer = 0;
        for (DcMotor motor : wheelList) buffer += Math.abs(motor.getTargetPosition() - motor.getCurrentPosition());
        return buffer/4;
    }

    /**
     * Cuts power to all wheels, forces a brake
     */
    public void stopMovement() {
        setDrivePower(0);
    }

    /**
     * Sets the mode of movement, whether POV or directional
     * @param driveType the type of driving to be done,
     *                  POV to move regardless of robot angle,
     *                  DIRECTIONAL to move relative to angle
     */
    public void setDriveType(DriveType driveType) {
        this.driveType = driveType;
    }

    /**
     * Sets the unit to be used when calculating ticks
     * @param unit The unit to be used in methods like polarMove() and drive()
     */
    public void setDriveUnit(DistanceUnit unit) {
        this.driveUnit = unit;
    }

    /**
     * The type of movement for the robot, determines whether the imu is used for calculation
     */
    public enum DriveType {
        POV,
        DIRECTIONAL
    }

    /**
     * Converts a given distance into ticks
     * @param distance input distance to be converted, any unit
     * @return distance represented as ticks for encoders
     */
    public int toTicks(double distance) {
        if (driveUnit == DistanceUnit.INCHES) return (int)(distance * TICKS_PER_INCH);
        else if (driveUnit == DistanceUnit.CENTIMETERS) return (int)(distance * TICKS_PER_CM);
        else if (driveUnit == DistanceUnit.TILES) return (int)(distance * TICKS_PER_TILE);
        else return 0;
    }

    /**
     * Checks to determine if the robot is moving or not
     * @param marginOfError The amount of ticks remaining is allowed to consider the robot mobile
     * @return boolean value of the robot's movement. True if moving, false if stopped
     */
    public boolean isMoving(int marginOfError) {
        return getMeanDifference() / 4 > marginOfError;
    }

    /**
     * Checks to determine if the robot is moving or not
     * @return boolean value of the robot's movement. True if moving, false if stopped
     */
    public boolean isMoving() {
        return isMoving(16);
    }

    /**
     * Puts the thread to sleep for about 300 milliseconds
     */
    public void rest() {
        try {
            Thread.sleep(WAIT_TIME);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    /**
     * Returns the direction in which the robot is facing
     * @return the robot's direction as a value from -PI to PI, measured in radians
     */
    public double getAngle() {
        double rawAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
        double netAngle = rawAngle - zeroAngle;

        if (netAngle < -Math.PI) netAngle += 2 * Math.PI;
        if (netAngle > Math.PI) netAngle -= 2 * Math.PI;

        return netAngle;
    }

    /**
     * Resets the angle so that the direction the robot currently faces is zero degrees
     */
    public void resetAngle() {
        zeroAngle = getAngle();

        if (zeroAngle < -Math.PI) zeroAngle += 2 * Math.PI;
        if (zeroAngle > Math.PI) zeroAngle -= 2 * Math.PI;
    }

    // TODO refactor this to combine PushSlides and DropSlides
    /**
     * Runnable that pushes the slides to their target position without using encoders.
     * Runs on a separate thread. Thread safety not tested.
     */
    public class PushSlides implements Runnable {
        int height;

        public PushSlides(int height) {
            this.height = height;
        }

        @Override
        public void run() {
            if (slideTarget() == height) return;

            leftSlide.setTargetPosition(height);
            rightSlide.setTargetPosition(height);

//            leftSlide.setPower(height < slidePosition() ? SLIDE_SPEED_UP : SLIDE_SPEED_DOWN);
//            rightSlide.setPower(height < slidePosition() ? SLIDE_SPEED_UP : SLIDE_SPEED_DOWN);
            leftSlide.setPower(0.5);
            rightSlide.setPower(0.5);

            leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            while (Math.abs(slideTarget() - slidePosition()) > 30) {
                telemetry.addData("Slide Position", slidePosition());
                telemetry.addData("Slide Target", slideTarget());
                telemetry.addData("Difference", slideTarget() - slidePosition());
                telemetry.update();
                if (slideTarget() != height || manualSlides || !enableThreads) return;
            }

            telemetry.addData("Target", "reached");
            telemetry.update();

            leftSlide.setPower(0.0);
            rightSlide.setPower(0.0);
        }
    }

    public class DropSlides implements Runnable {
        @Override
        public void run() {
            // TODO configure this number
            while (slidePosition() < SLIDE_THRESHOLD) {
                if (!enableThreads || slideTarget() != 0) return;
            }

            leftSlide.setPower(0.95);
            rightSlide.setPower(0.95);

            leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    public void pushSlides(int height) {
        if (height == slideTarget()) return;
        new Thread(new PushSlides(height)).start();
    }

    /**
     * Sets the slide position using the motor encoders
     * @param height height to which the slide should move to, negative values are high, 0 at rest
     */
    public void setSlides(int height) {
        if (height == slideTarget()) return;

        double power = height < slideTarget() ? SLIDE_SPEED_UP : SLIDE_SPEED_DOWN;

        manualSlides = false;
        leftSlide.setTargetPosition(height);
        rightSlide.setTargetPosition(height);

        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftSlide.setPower(power);
        rightSlide.setPower(power);
    }

    /**
     * Drops the slides to ground level
     */
    public void dropSlides() {
        leftSlide.setTargetPosition(0);
        rightSlide.setTargetPosition(0);

        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftSlide.setPower(0);
        rightSlide.setPower(0);

        new Thread(new DropSlides()).start();
    }

    /**
     * Moves the slides manually by setting their power, overrides encoders
     * @param power power at which the slides should move multiplied by a constant
     */
    public void moveSlides(double power) {
        manualSlides = true;
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftSlide.setPower(power * (power < 0 ? SLIDE_SPEED_UP : SLIDE_SPEED_DOWN));
        rightSlide.setPower(power * (power < 0 ? SLIDE_SPEED_UP : SLIDE_SPEED_DOWN));
    }

    /**
     * Puts the current thread on pause until the slides are in their target position
     */
    public void waitForSlides() {
        while (Math.abs(slidePosition() - slideTarget()) > 16) {
            telemetry.addData("Slide Difference", Math.abs(slidePosition() - slideTarget()));
        }

        rest();
    }

    /**
     * Resets the slide encoders so the current position is 0
     */
    public void resetSlides() {
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * Returns the current slide position
     * @return the average encoder value of both slides
     */
    public int slidePosition() {
        return (leftSlide.getCurrentPosition() + rightSlide.getCurrentPosition()) / 2;
    }

    /**
     * Returns the target value at which the slides attempt to move to
     * @return target position of the slides
     */
    public int slideTarget() {
        return leftSlide.getTargetPosition();
    }

    /**
     * Toggles the claw between a closed and open position
     */
    public void toggleClaw() {
        pinch = !pinch;
        leftClaw.setPosition(pinch ? 0.7 : 1.0);
        rightClaw.setPosition(pinch ? 0.3 : 0.0);
    }

    /**
     * Sets the claw to either an open or closed position
     * @param state new state of claw to be in, true for closed, false for open
     */
    public void setClaw(boolean state) {
        pinch = !state;
        toggleClaw();
    }

    /**
     * Sets the claw to a specific position based on the pinch value
     * @param pinch value at which the claws should close, 0.0 represents a fully open claw
     */
    public void setClaw(double pinch) {
        this.pinch = true;
        leftClaw.setPosition(1 - pinch);
        rightClaw.setPosition(pinch);
    }
}