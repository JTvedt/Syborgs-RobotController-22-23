package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Disabled
@Deprecated
@TeleOp(name="Robot TeleOp")
public class RobotTeleOp extends OpMode {
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private BNO055IMU imu;

    // Angle at which getAngle() should return 0
    private double initialAngle = 0;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class, "FL");
        frontRight = hardwareMap.get(DcMotor.class, "FR");
        backLeft = hardwareMap.get(DcMotor.class, "BL");
        backRight = hardwareMap.get(DcMotor.class, "BR");

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        // Stick Controls
        double stickAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x);
        double magnitude = Range.clip(Math.hypot(gamepad1.left_stick_y, gamepad1.left_stick_x) * 0.5, -1, 1);

        magnitude *= gamepad1.x ? 2 : 1;
        magnitude /= gamepad1.y ? 2 : 1;

        frontLeft.setPower((Math.sin(stickAngle) + Math.cos(stickAngle)) * magnitude);
        frontRight.setPower((Math.sin(stickAngle) - Math.cos(stickAngle)) * magnitude);
        backLeft.setPower((Math.sin(stickAngle) - Math.cos(stickAngle)) * magnitude);
        backRight.setPower((Math.sin(stickAngle) + Math.cos(stickAngle)) * magnitude);

        // ok for some reason the left_stick likes to sort of "snap" to values of 1.0 and -1.0
        telemetry.addData("LStick y: ", "%f", gamepad1.left_stick_y);
        telemetry.addData("LStick X: ", "%f", gamepad1.left_stick_x);
        // Also I don't know why but the stick angle is always in one of 8 cardinal directions
        telemetry.addData("Stick Angle: ", "%f", stickAngle);
        telemetry.addData("Robot Direction: ", "%f", getAngle());
        telemetry.update();
    }

    // Stops the robot in its place, prevents drifting
    private void brake() {

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