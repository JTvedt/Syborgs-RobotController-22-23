package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Functional Autonomous", group="Linear Opmode")
public class RobotAutonomous extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private DriveTrain motors = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        motors = new DriveTrain(hardwareMap);

        waitForStart();

        runtime.reset();

        // Autonomous processes go here
    }
}
