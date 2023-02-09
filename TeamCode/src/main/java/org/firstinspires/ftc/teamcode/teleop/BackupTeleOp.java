package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Backup TeleOp")
public class BackupTeleOp extends LinearOpMode {

    public DcMotor frontRight;
    public DcMotor frontLeft;
    public DcMotor backRight;
    public DcMotor backLeft;
    public DcMotor leftSlide;
    public DcMotor rightSlide;
    public Servo leftClaw;
    public Servo rightClaw;

    @Override
    public void runOpMode() {

        //chassis wheels mapping
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

        //slides mapping
        leftSlide  = hardwareMap.get(DcMotor.class, "LS");
        rightSlide = hardwareMap.get(DcMotor.class, "RS");

        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setDirection(DcMotor.Direction.REVERSE);

        //claws mapping
        leftClaw = hardwareMap.get(Servo.class, "LC");
        rightClaw = hardwareMap.get(Servo.class, "RC");

        //slide speed (default: 0.9)
        public int slideSpeed = 0.9;
        
        waitForStart();

        while(opModeIsActive())
        {
            //first pad controls
            double drive = gamepad1.left_stick_y * 0.5;
            double strafe = -gamepad1.left_stick_x * 0.7;
            double spin = gamepad1.right_stick_x * 0.4;
            //slides controls
            double reach = gamepad2.right_stick_y * 0.6;
            reach *= (reach > 0 ? slideSpeed : 1 ) * (reach < 0 ? slideSpeed : 1);

            //giving power to the motors
            frontLeft.setPower(drive + strafe - spin );
            frontRight.setPower(drive - strafe + spin);
            backLeft.setPower(drive - strafe - spin );
            backRight.setPower(drive + strafe + spin );
            leftSlide.setPower(reach);
            rightSlide.setPower(reach);

            //claw controls
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




    //yashi's re-alignment method
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
