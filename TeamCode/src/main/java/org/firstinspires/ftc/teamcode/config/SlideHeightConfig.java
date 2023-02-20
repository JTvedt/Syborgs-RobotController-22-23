package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.Sybot;

/**
 * Used to configure heights of the slides for autonomous grabbing of slides off the side of the field
 */
@TeleOp(name="Slides Height Config", group="config")
public class SlideHeightConfig extends LinearOpMode {
    public Sybot robot;
    int bottom = -4130;
    int current;
    public void runOpMode(){
        robot = new Sybot(this);

        robot.leftSlide.setTargetPosition(0);
        robot.rightSlide.setTargetPosition(0);

        robot.leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftSlide.setPower(0.7);
        robot.rightSlide.setPower(0.7);

        while (opModeIsActive()) runLoop();
    }

    public void runLoop() {
        if(gamepad1.dpad_left){
            robot.setSlides(bottom);
            current = bottom;
        }
        if(gamepad1.dpad_up){robot.setSlides(current -= 50);}
        if(gamepad1.dpad_down){robot.setSlides(current += 50);}

        telemetry.addData("Encoder Value",current);
        telemetry.update();
    }

}
