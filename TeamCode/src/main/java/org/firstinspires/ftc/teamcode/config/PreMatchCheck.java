package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.Sybot;

@Autonomous(name="Pre Match Check", group="config")
public class PreMatchCheck extends LinearOpMode {
    public Sybot robot;
    @Override
    public void runOpMode(){
        robot = new Sybot(this, Sybot.OpModeType.AUTONOMOUS);
        robot.setClaw(false);
        int parkZone = robot.retrieveZone();
        robot.setClaw(true);
        robot.setSlides(Sybot.SLIDE_HIGH_TICKS);
        robot.dropSlides();
    }
}
