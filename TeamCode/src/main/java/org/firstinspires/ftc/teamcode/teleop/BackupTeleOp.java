package org.firstinspires.ftc.teamcode.teleop;


import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//import com.qualcomm.robotcore.hardware.DcMotor$RunMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Deprecated
@Disabled
@TeleOp(name = "Backup TeleOp")
public class BackupTeleOp extends LinearOpMode {

    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor backRight;
    private DcMotor backLeft;
    private DcMotor leftArm;
    private DcMotor rightArm;

    private Servo leftClaw;
    private Servo rightClaw;

    double cntPower;
    double current = 0;


    @Override
    public void runOpMode() {

        frontRight  = hardwareMap.get(DcMotor.class, "FR");
        frontLeft = hardwareMap.get(DcMotor.class, "FL");
        backRight  = hardwareMap.get(DcMotor.class, "BR");
        backLeft = hardwareMap.get(DcMotor.class, "BL");

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        leftArm  = hardwareMap.get(DcMotor.class, "LS");
        rightArm = hardwareMap.get(DcMotor.class, "RS");

        leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArm.setDirection(DcMotor.Direction.REVERSE);

        leftClaw = hardwareMap.get(Servo.class, "LC");
        rightClaw = hardwareMap.get(Servo.class, "RC");

        waitForStart();

        while(opModeIsActive())
        {
            //first pad controls
            double drive = gamepad1.left_stick_y * 0.5;
            double strafe = -gamepad1.left_stick_x * 0.7;
            double spin = gamepad1.right_stick_x * 0.4;
            double up = gamepad2.right_stick_y;
            up *= (up > 0 ? 0.9 : 1 ) * (up < 0 ? 0.5 : 1);

            //giving power to the motors
            frontLeft.setPower(drive + strafe - spin );
            frontRight.setPower(drive - strafe + spin);
            backLeft.setPower(drive - strafe - spin );
            backRight.setPower(drive + strafe + spin );
            leftArm.setPower(up);
            rightArm.setPower(up);

            
            if (gamepad2.a) {
                leftClaw.setPosition(1.0);
                rightClaw.setPosition(0.0);
            }
            if (gamepad2.b) {
                leftClaw.setPosition(0.7);
                rightClaw.setPosition(0.3);
            }

            
            telemetry.update();
        }
    }

    

    
    public void rotate(double power, int degrees)
    {
        

        double leftPower, rightPower;


        if (degrees < 0) {   // turn right.
            leftPower = power;
            rightPower = -power;
        } else if (degrees > 0) {   // turn left.
            leftPower = -power;
            rightPower = power;
        } else return;

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setPower(-leftPower);
        frontRight.setPower(-rightPower);
        backLeft.setPower(-leftPower);
        backRight.setPower(-rightPower);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        sleep(100);

        
    }

    
}
