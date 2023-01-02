package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Test Autonomous", group="Linear Opmode")
public class RobotTest extends LinearOpMode {
    private DriveTrain motors;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        motors = new DriveTrain(hardwareMap);

        waitForStart();
        motors.spin(90);
    }
}
