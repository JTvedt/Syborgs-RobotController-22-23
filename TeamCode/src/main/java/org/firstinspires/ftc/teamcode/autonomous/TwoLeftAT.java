package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.Sybot;

/**
 *2 cone autonomous using cartesian movement
 * @author Jeffrey Tvedt & Alex Malladi & Tyler Philip
 * COMMENTS that start with !!!! are important as they are values that sometimes change in the code based on preformance and small movement fails
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
        robot.drive(51);// !!!! value sometimes changes to 51 instead of 53
        robot.spin(-26);
        robot.cartesianMove(8, 7.7);
        robot.waitForSlides();
        robot.setSlides(-4000);
        sleep(600);
        robot.setClaw(false);
        sleep(300);
        robot.setSlides(Sybot.SLIDE_HIGH_TICKS);

        // Exit and move to cone stack
        robot.cartesianMove(-7.8, -8.8);
        robot.setSlides(-1000);
        robot.waitForSlides();
        robot.spin(108);
        robot.drive(2);// Commented out depending on if initial value is 53

        robot.strafe(-27);

        //Bring Slides down to cone stack
        robot.setSlides(-850);
        robot.waitForSlides();
        robot.drive(2.5);
        robot.strafe(-3); //!!!! Commented out occasionally
        robot.setClaw(true); //Crabs cone
        robot.drive(2);
        sleep(300);
        robot.setSlides(Sybot.SLIDE_HIGH_TICKS); //Brings cone up
        // DO NOT TOUCH ANYTHING ABOVE THIS
        // WE SPENT WAY TOO LONG ON IT AND DO NOT WANT TO FIX IT AGAIN

        //Moves back to high junction
        robot.strafe(25 );
        robot.spin(-132);
        robot.cartesianMove(8, 8.8);

        //Places cone on the junction
        robot.setSlides(-4130);
        sleep(600);
        robot.setClaw(false);
        sleep(300);
        robot.setSlides(Sybot.SLIDE_HIGH_TICKS);

    }

}
