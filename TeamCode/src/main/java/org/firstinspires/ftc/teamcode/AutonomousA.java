package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Left Hand Autonomous")
public class AutonomousA extends RobotAutonomous {
    @Override
    public void runOpMode() {
        initialize();
        spin(-90);
    }
}
