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
import org.firstinspires.ftc.teamcode.cv.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.cv.EasyOpenCvPipeline;

import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

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

    public static final double SLIDE_SPEED_UP = 0.7;
    public static final double SLIDE_SPEED_DOWN = 0.4;

    public static final int WAIT_TIME = 200;
    public static final int TICK_THRESHOLD = 300;
    public static final int SLIDE_THRESHOLD = -1120;
    public static final int SLIDE_HIGH_TICKS = -800;
    public static final int CLAW_TIME = 710;

    public static double LEFT_OPEN = 0.66;
    public static double LEFT_CLOSE = 0.48;
    public static double RIGHT_OPEN = 0.0;
    public static double RIGHT_CLOSE = 0.18;

    private final LinearOpMode parent;
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;

    private final ArrayList<DcMotor> wheelList;
    private final DcMotor frontLeft;
    private final DcMotor frontRight;
    private final DcMotor backLeft;
    private final DcMotor backRight;

    public final DcMotor leftSlide;
    public final DcMotor rightSlide;

    private final Servo leftClaw;
    private final Servo rightClaw;

    private final BNO055IMU imu;

    private DriveType driveType = DriveType.POV;
    private DistanceUnit driveUnit = DistanceUnit.INCHES;
    private double zeroAngle = 0;
    public boolean enableThreads = true;
    public boolean pinch = false; // true for gripped
    public boolean manualSlides = false;
    public boolean slideRelease = false;
    public boolean mirrorDirection = false;
    public int parkZone = -1;
    public int slideDelta = 0;

    public static CvImplementation cvImplementation = CvImplementation.APRIL_TAGS;
    public OpenCvCamera camera;

    public EasyOpenCvPipeline eocvPipeline;
    public AprilTagDetectionPipeline aprilTagsPipeline;

    //Tag IDs of sleeve
    static final int LEFT_TAG = 9;
    static final int MIDDLE_TAG = 10;
    static final int RIGHT_TAG = 11;

    AprilTagDetection tagOfInterest;

    //Used in telemetry debugging
    static final double FEET_PER_METER = 3.28084;


    /**
     * Constructor for Sybot, takes in the parent, OpMode type, and starting side
     * Starting side is only relevant for autonomous
     * @param parent The OpMode that will be using this class's methods
     * @param opModeType The type of OpMode, Autonomous or TeleOp
     * @param side Starting position of the robot, relevant for autonomous.
     *             If placed on the left side, strafing will be mirrored by default
     */
    public Sybot(LinearOpMode parent, OpModeType opModeType, StartSide side) {
        this.parent = parent;
        hardwareMap = parent.hardwareMap;
        telemetry = parent.telemetry;

        telemetry.addData("Status", "...");
        telemetry.update();

        // Store wheels in list for iteration purposes
        wheelList = new ArrayList<>();
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
        if (opModeType == OpModeType.AUTONOMOUS) {
            initializeCv(side);
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

        switch (opModeType) {
            case AUTONOMOUS:
                // During init phase search for the cone
                searchForSleeve();
                break;
            case TELEOP:
                parent.waitForStart();
                break;
        }
        runtime.reset();
    }

    public void initializeCv(StartSide side) {
        switch (cvImplementation) {
            case EASYOPENCV:
                if (side == StartSide.LEFT) eocvPipeline = new EasyOpenCvPipeline(0.29, 0.11, 0.45, 0.27);
                else if (side == StartSide.RIGHT) eocvPipeline = new EasyOpenCvPipeline(0.29, 0.11, 0.45, 0.27);
                else eocvPipeline = new EasyOpenCvPipeline(0.3, 0.02, 0.49, 0.18);

                openCamera(eocvPipeline);
                break;
            case APRIL_TAGS:
                // Default April Tag Params
                final double FX = 578.272;
                final double FY = 578.272;
                final double CX = 402.145;
                final double CY = 221.506;

                // In meters
                final double TAG_SIZE = 0.166;
                aprilTagsPipeline = new AprilTagDetectionPipeline(TAG_SIZE, FX, FY, CX, CY);

                openCamera(aprilTagsPipeline);
                telemetry.setMsTransmissionInterval(50);
        }
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

    public void addAprilTagsToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

    public void openCamera(OpenCvPipeline pipeline) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Camera"), cameraMonitorViewId);
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

    public void searchForSleeve() {
        while (!parent.isStarted() && !parent.isStopRequested()) {
            switch (cvImplementation) {
                case EASYOPENCV:
                    parkZone = eocvPipeline.getZone();
                    telemetry.addData("Seen", new String[]{"Red ", "Green ", "Blue "}[parkZone] + parkZone);
                    telemetry.addData("Color data", eocvPipeline.getColor());
                    telemetry.update();
                    break;
                case APRIL_TAGS:
                    parkZone = getAprilTagsZone();
                    break;
            }
        }

        telemetry.addData("Sleeve visible", parkZone);
    }
    /**
     * Returns Parking Zone
     */
    public int getAprilTagsZone() {
        ArrayList<AprilTagDetection> currentDetections = aprilTagsPipeline.getLatestDetections();
        boolean tagFound = false;

        for (AprilTagDetection tag : currentDetections) {
            if (tag.id == LEFT_TAG || tag.id == MIDDLE_TAG || tag.id == RIGHT_TAG) {
                tagOfInterest = tag;
                tagFound = true;
                break;
            }
        }

        if (tagFound) {
            telemetry.addLine("Tag visible\n\nLocation data:");
            addAprilTagsToTelemetry(tagOfInterest);
        } else {
            telemetry.addLine("No tag present");

            if(tagOfInterest == null) {
                telemetry.addLine("(The tag has never been seen)");
            } else {
                telemetry.addLine("\nTag last seen at:");
                addAprilTagsToTelemetry(tagOfInterest);
            }
        }

        telemetry.update();
        rest(20);
        if(tagOfInterest == null){
            return -1;
        }
        return tagOfInterest.id;
    }

    public int retrieveZone() {
        telemetry.addData("CV Implementation", cvImplementation);
        telemetry.addData("Parking in", parkZone);
        telemetry.update();
        rest(600);
        camera.stopStreaming();

        return parkZone;
    }

    public enum CvImplementation {
        EASYOPENCV,
        APRIL_TAGS

    }

    public static void setImplementation(CvImplementation implementation) {
        cvImplementation = implementation;
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
     * To be run after stop is pressed, closes all threads
     */
    public void stop() {
        enableThreads = false;
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
        double radianDirection = Angle.toRadians(direction) - (driveType == DriveType.POV ? getAngle() : 0);
        double horizontal = Math.cos(radianDirection) * (mirrorDirection ? -1 : 1);
        double vertical = Math.sin(radianDirection) * 0.87;
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
        }

        rest();
    }


    /**
     * Spins the robot either clockwise or counterclockwise
     * @param angle angle to turn in degrees, positive values turn counterclockwise and negative values turn clockwise
     */
    public void spin(double angle) {
        int tickCount = (int)(Angle.toRadians(angle) * 425);

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
        double radianDirection = Angle.toRadians(direction) - getAngle();
        double radianAngle = Angle.toRadians(angle);
        double targetAngle = Angle.bound(getAngle() + radianAngle);
        double horizontal = Math.cos(radianDirection);
        double vertical = Math.sin(radianDirection) * 0.87;

        int driveTicks = (int)(distance * TICKS_PER_INCH);
        int spinTicks = (int)(radianAngle * 425);
        double baseSpin = 0.3 * Math.signum(radianAngle);
        double spin = 0;

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setTargetPosition((int)((vertical + horizontal) * driveTicks) - spinTicks);
        frontRight.setTargetPosition((int)((vertical - horizontal) * driveTicks) + spinTicks);
        backLeft.setTargetPosition((int)((vertical - horizontal) * driveTicks) - spinTicks);
        backRight.setTargetPosition((int)((vertical + horizontal) * driveTicks) + spinTicks);

        setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (isMoving()) {
            horizontal = Math.cos(radianDirection);
            vertical = Math.sin(radianDirection) * 0.87;

            // Calculate spin based on tick differences
            int avgDisparity = ((frontLeft.getCurrentPosition() - backRight.getCurrentPosition()) + (frontRight.getCurrentPosition() - backLeft.getCurrentPosition()))/2;
//            if (avgDisparity > 30) spin = baseSpin;
//            else spin = 0;
            spin = 0;

            // Update power to move in a straight line
            frontLeft.setPower((horizontal + vertical) * 0.65 - spin);
            frontRight.setPower((horizontal - vertical) * 0.65 + spin);
            backLeft.setPower((horizontal - vertical) * 0.65 - spin);
            backRight.setPower((horizontal + vertical) * 0.65 - spin);

            frontLeft.setPower(0.6);
            frontRight.setPower(0.6);
            backLeft.setPower(0.6);
            backRight.setPower(0.6);
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
        polarMove(Math.hypot(y, x), Angle.toDegrees(Math.atan2(y, x)));
    }

    /**
     * NOT TESTED DO NOT USE
     * Spins the robot to a new position based on its current one
     * @param newAngle The new angle to spin to relative to old angle
     */
    public void spinTo(double newAngle) {
        double radAngle = Angle.toRadians(newAngle);
        setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (Math.abs(getAngle() - radAngle) > Math.PI/270) {
            double power = smoothAngle(radAngle) * 1.5;
            if (Math.abs(power) < 0.175)
                power = Math.signum(power) * 0.175;

            frontLeft.setPower(-power);
            frontRight.setPower(power);
            backLeft.setPower(-power);
            backRight.setPower(power);

            telemetry.addData("Target", radAngle);
            telemetry.addData("Angle", getAngle());
            telemetry.addData("Difference", Math.abs(getAngleDifference(radAngle)));
            telemetry.addData("Power", power);
            telemetry.update();
        }
        telemetry.addLine("Target Reached");
        telemetry.update();

        stopMovement();
        rest(300);
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
     * @param time ticks to wait
     */
    public void rest(int time) {
        try {
            Thread.sleep(time);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public void rest() {
        rest(WAIT_TIME);
    }

    /**
     * Returns the direction in which the robot is facing
     * @return the robot's direction as a value from -PI to PI, measured in radians
     */
    public double getAngle() {
        double rawAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
        return Angle.bound(rawAngle - zeroAngle);
    }

    /**
     * Gets the most efficient possible angle to turn to turn from current position to a target angle
     * @param targetAngle the target angle for the robot to turn to, in radians
     * @return positive value if turning clockwise, negative value if turning counterclockwise
     */
    public double getAngleDifference(double targetAngle) {
        return Angle.difference(getAngle(), targetAngle);
    }

    /**
     * Resets the angle so that the direction the robot currently faces is zero degrees
     */
    public void resetAngle() {
        zeroAngle = Angle.round(getAngle());
    }

    public double smoothAngle(double angle) {
        final double[] thresholds = {144d, 72d, 36d, 18d, 9d, .5d};
        final double[] coefficients = {0, 0.025, 0.05, 0.1, 0.2, 0.4};

        double angleDiff = getAngleDifference(angle);
        for (int i = 0; i < thresholds.length; i++) {
            if (Math.abs(angleDiff) < Math.PI/thresholds[i])
                return coefficients[i] * Math.signum(angleDiff);
        }
        return 0;
    }

    public double smoothAngle() {
        return smoothAngle(Angle.round(getAngle(), 4));
    }

    /**
     * Sets the slide position using the motor encoders
     * @param height height to which the slide should move to, negative values are high, 0 at rest
     */
    public void setSlides(int height) {
        if (height == slideTarget()) {
            return;
        }

        double power = height < slideTarget() ? SLIDE_SPEED_UP : SLIDE_SPEED_DOWN;

        manualSlides = false;
        leftSlide.setTargetPosition(height);
        rightSlide.setTargetPosition(height);

        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftSlide.setPower(power);
        rightSlide.setPower(power);

        new Thread(new ReleaseSlides()).start();
    }

    /**
     * Runnable class that drops the slides to ground position.
     * Not tested for thread safety, probably safe.
     */
    public class DropSlides implements Runnable {
        @Override
        public void run() {
            if (slideTarget() == 0) return;

            leftSlide.setTargetPosition(0);
            rightSlide.setTargetPosition(0);

            leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftSlide.setPower(0);
            rightSlide.setPower(0);

            int lastPos = slidePosition();
            int stuckTicks = 0;
            while (true) {
                if (!enableThreads || slideTarget() != 0) {
                    leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    return;
                }

                int pos = slidePosition();
                slideDelta = pos - lastPos;

                if (slidePosition() > SLIDE_THRESHOLD) break;
                if (slideDelta < 25 && pos > -2500 || slideDelta < 5) stuckTicks++;
                else stuckTicks = 0;
                if (stuckTicks >= 3) break;

                lastPos = pos;
            }

            leftSlide.setPower(0.95);
            rightSlide.setPower(0.95);

            leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    /**
     * Drops the slides to ground level
     * THIS IS DEPRECATED USE setSlides(0) INSTEAD
     */
    @Deprecated
    public void dropSlides() {
        setSlides(0); // temporary
//        new Thread(new DropSlides()).start();
    }

    /**
     * Runnable class that disables the encoders once the slides reach their target
     * Runs on a separate thread and continuously checks for the slide position
     * Not tested for thread safety, probably safe
     */
    public class ReleaseSlides implements Runnable {
        @Override
        public void run() {
            if (!slideRelease) return;
            int target = slideTarget();

            while (Math.abs(slidePosition() - target) > 16) {
                if (slideTarget() != target || !enableThreads || !slideRelease) {
                    return;
                }
            }

            moveSlides(0.0);
        }
    }

    public void lockSlides() {
        manualSlides = false;
        leftSlide.setTargetPosition(leftSlide.getCurrentPosition());
        rightSlide.setTargetPosition(rightSlide.getCurrentPosition());

        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftSlide.setPower(0.5);
        rightSlide.setPower(0.5);
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

    public class PinchSlide implements Runnable {
        boolean state;
        public PinchSlide(boolean state) {
            this.state = state;
        }

        @Override
        public void run() {
            setClaw(state);
            rest(CLAW_TIME);
            if (state) setSlides(SLIDE_HIGH_TICKS);
            else dropSlides();
        }
    }

    /**
     * Grabs a cone and moves the slides up simultaneously.
     * Will grab cone and move up or release cone and move down.
     * @param state to which position the slide moves,
     *              true for closed claw and up,
     *              false for open claw and down
     */
    public void pinchSlide(boolean state) {
        new Thread(new PinchSlide(state)).start();
    }

    /**
     * Grabs a cone and moves both slides up or down simultaneously
     * Will do whatever opposite of the claw's current state
     */
    public void pinchSlide() {
        pinchSlide(!pinch);
    }

    /**
     * Puts the current thread on pause until the slides are in their target position
     */
    public void waitForSlides() {
        while (Math.abs(slidePosition() - slideTarget()) > 16) {
//            telemetry.addData("Slide Difference", Math.abs(slidePosition() - slideTarget()));
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
        setClaw(!pinch);
    }

    /**
     * Sets the claw to either an open or closed position
     * @param state new state of claw to be in, true for closed, false for open
     */
    public void setClaw(boolean state) {
        setClaw(state ? LEFT_CLOSE : LEFT_OPEN, state ? RIGHT_CLOSE : RIGHT_OPEN);
    }

    /**
     * Sets the claw to a specific position based on the pinch value
     * @param pinch value at which the claws should close, 0.0 represents a fully open claw
     */
    public void setClaw(double pinch) {
        setClaw(1 - pinch, pinch);
    }

    public void setClaw(double leftPinch, double rightPinch) {
        leftClaw.setPosition(leftPinch);
        rightClaw.setPosition(rightPinch);
        pinch = leftPinch < LEFT_OPEN && rightPinch > RIGHT_OPEN;
    }
}