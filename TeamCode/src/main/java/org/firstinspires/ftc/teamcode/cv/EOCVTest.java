package org.firstinspires.ftc.teamcode.cv;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.Sybot;

@Autonomous(name="EasyOpenCV Test", group="test")
public class EOCVTest extends LinearOpMode {
    public Sybot robot;

    @Override
    public void runOpMode() {
        Sybot.setImplementation(Sybot.CvImplementation.EASYOPENCV);
        robot = new Sybot(this, Sybot.OpModeType.AUTONOMOUS, Sybot.StartSide.RIGHT);
        robot.retrieveZone();
        sleep(4000);
    }
}