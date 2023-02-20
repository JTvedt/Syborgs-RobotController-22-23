package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.Sybot;

@Autonomous(name="Get Angle Config", group="config")
public class GetAngleConfig extends LinearOpMode {
    public Sybot robot;
    @Override
    public void runOpMode(){
        robot = new Sybot(this, Sybot.OpModeType.AUTONOMOUS);
        while(opModeIsActive()) {
            telemetry.addData("Angle", robot.getAngle());
            telemetry.update();
        }
    }
}
