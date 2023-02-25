package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.Sybot;

/**
 *2 cone autonomous using cartesian movement
 * @author Jeffrey Tvedt & Alex Malladi & Tyler Philip
 */
@Autonomous(name="2 Cone Auton AT Turn")
public class TwoLeftAT extends LinearOpMode{
    public Sybot robot;
    @Override
    public void runOpMode() {
        Sybot.setImplementation(Sybot.CvImplementation.APRIL_TAGS);
        robot = new Sybot(this, Sybot.OpModeType.AUTONOMOUS);
        robot.setDriveUnit(DistanceUnit.INCHES);
        robot.setClaw(true);
        sleep(670);
        robot.setSlides(Sybot.SLIDE_HIGH_TICKS);

        // Move to cone 1
        robot.drive(53);
        robot.spin(-33);
        robot.cartesianMove(8, 7.7);
        robot.waitForSlides();
        robot.setSlides(-4000);
        sleep(600);
        robot.setClaw(false);
        sleep(300);
        robot.setSlides(Sybot.SLIDE_HIGH_TICKS);

        // DO NOT TOUCH ANYTHING ABOVE THIS
        // WE SPENT WAY TOO LONG ON IT AND DO NOT WANT TO FIX IT AGAIN


        // Exit and move to cone stack
        robot.cartesianMove(-7.8, -8.8);
        robot.setSlides(-1000);
        robot.waitForSlides();
        robot.spin(115);
        robot.strafe(-27);

        //Bring Slides down to cone stack
        robot.setSlides(-900);
        robot.waitForSlides();
        robot.strafe(-3.5);
        robot.setClaw(true); //Crabs cone
        robot.drive(2);
        sleep(300);
        robot.setSlides(Sybot.SLIDE_HIGH_TICKS); //Brings cone up

        //Moves back to high junc tion
        robot.strafe(27);
        robot.spin(-125);
        robot.cartesianMove(8,8.5);

        //Places cone on the junction
        robot.setSlides(-4000);
        sleep(600);
        robot.setClaw(false);
        sleep(300);
        robot.setSlides(Sybot.SLIDE_HIGH_TICKS);

    }

}
