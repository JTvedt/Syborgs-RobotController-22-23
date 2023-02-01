package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.Sybot;

/**
 * In case slides aren't working this is a parking only auton using april tags
 * @author Tyler Philip & Jeffrey Tvedt
 */
@Autonomous(name="Park Only Auton")
public class ParkOnlyAuton extends LinearOpMode{
    public Sybot robot;
    @Override
    public void runOpMode() {
        robot = new Sybot(this, Sybot.OpModeType.AUTONOMOUS, Sybot.StartSide.LEFT);
        robot.setDriveUnit(DistanceUnit.TILES);
        Sybot.cvImplementation = Sybot.CvImplementation.APRIL_TAGS;

        sleep(400);
        int parkZone = robot.getZone();
        robot.drive(1.25);
        telemetry.addData("April Parking in", parkZone);
        telemetry.update();

        sleep(400);
        if(parkZone == 9){
            robot.strafe(-1);
            telemetry.addData("PARKING IN","LEFT");
            telemetry.update();
        }else if(parkZone == 11){
            robot.strafe(1);
            telemetry.addData("PARKING IN","LEFT");
            telemetry.update();
        }
    }


}
