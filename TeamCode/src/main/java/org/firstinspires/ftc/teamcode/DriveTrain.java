package org.firstinspires.ftc.teamcode;

import java.util.HashMap;
import java.util.ArrayList;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveTrain {
    private static final double BASE_POWER = 0.35;
    private static final double MAX_POWER = 0.8;
    private static final double TILE_CM = 60;
    private static final int WAIT_TIME = 300;

    private static final double PULSES_PER_REVOLUTION = 537.7;
    private static final double CM_PER_REVOLUTION = 40.02;
    private static final double TICKS_PER_CM = PULSES_PER_REVOLUTION / CM_PER_REVOLUTION;

    private final ArrayList<DcMotor> motorList;
    private final HashMap<String, Integer> motorMap;
    private final HashMap<DcMotor, Double> powerMap;

    public DriveTrain(HardwareMap hardwareMap) {
        // Store motors in list for iteration purposes
        motorList = new ArrayList<DcMotor>();
        motorList.add(hardwareMap.get(DcMotor.class, "FL"));
        motorList.add(hardwareMap.get(DcMotor.class, "FR"));
        motorList.add(hardwareMap.get(DcMotor.class, "BL"));
        motorList.add(hardwareMap.get(DcMotor.class, "BR"));

        // String references mapped to ArrayList
        motorMap = new HashMap<String, Integer>();
        motorMap.put("FL", 0);
        motorMap.put("FR", 1);
        motorMap.put("BL", 2);
        motorMap.put("BR", 3);

        // Power coefficient applied to each motor
        powerMap = new HashMap<DcMotor, Double>();
        powerMap.put(getMotor("FL"), 1.0);
        powerMap.put(getMotor("FR"), 1.0);
        powerMap.put(getMotor("BL"), 1.0);
        powerMap.put(getMotor("BR"), 1.0);

        // Reverse left side motors
        getMotor("FL").setDirection(DcMotor.Direction.REVERSE);
        getMotor("BL").setDirection(DcMotor.Direction.REVERSE);
    }

    /**
     * Moves robot in a linear direction
     * @param distance Distance to travel, in centimeters
     * @param direction Direction of movement, in degrees
     */
    public void linearMove(double distance, double direction) {
        double radianDirection = direction * (Math.PI / 180);
        double horizontal = Math.cos(radianDirection);
        double vertical = Math.sin(radianDirection) * 0.87;
        int tickCount = (int)(distance * TICKS_PER_CM);

        resetEncoders();

        getMotor("FL").setTargetPosition((int)((horizontal + vertical) * tickCount));
        getMotor("FR").setTargetPosition((int)((-horizontal + vertical) * tickCount));
        getMotor("BL").setTargetPosition((int)((-horizontal + vertical) * tickCount));
        getMotor("BR").setTargetPosition((int)((horizontal + vertical) * tickCount));

        startMovement();
        setPower(1.0);

        while (isMoving()) {

        }

        try {
            Thread.sleep(WAIT_TIME);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    /**
     * Spins the robot either clockwise or counterclockwise
     * @param angle angle to turn in degrees, positive counterclockwise, negative clockwise
     */
    public void spin(double angle) {
        double radianAngle = angle * (Math.PI/180);
        int tickCount = (int)(radianAngle * 425);

        resetEncoders();

        getMotor("FL").setTargetPosition(-tickCount);
        getMotor("FR").setTargetPosition(tickCount);
        getMotor("BL").setTargetPosition(-tickCount);
        getMotor("BR").setTargetPosition(tickCount);

        startMovement();
        setPower(1.0);

        while (isMoving()) {

        }
    }

    /**
     * Moves robot forwards, negative distance for backward
     * @param distance cm moved forward
     */
    public void drive(double distance) {
        linearMove(distance, 90);
    }

    /**
     * Strafes the robot to the right, negative distance for left
     * @param distance cm moved to the right
     */
    public void strafe(double distance) {
        linearMove(distance, 0);
    }

    // Returns the motor referenced by the tag
    private DcMotor getMotor(String tag) {
        return motorList.get(motorMap.get(tag));
    }

    // Resets the encoders
    private void resetEncoders() {
        for (DcMotor motor : motorList) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    // Tells the motors to start moving to designated spot
    private void startMovement() {
        for (DcMotor motor : motorList) {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    // Sets motor power to 0 to stop the robot
    private void stopMovement() {
        for (DcMotor motor : motorList) {
            motor.setPower(0);
        }
    }

    // Set the power on the power mapping
    private void setMotorPower(String motorName, double powerCoefficient) {
        double power = Math.min(Math.max(powerCoefficient, -MAX_POWER/BASE_POWER), MAX_POWER/BASE_POWER) * BASE_POWER;
        powerMap.put(getMotor(motorName), power);
    }

    private void setPower() {
        for (DcMotor motor : motorList) {
            motor.setPower(BASE_POWER * powerMap.get(motor));
        }
    }

    private void setPower(double power) {
        for (DcMotor motor : motorList) {
            motor.setPower(BASE_POWER * power);
        }
    }

    /**
     * Checks the current state of the robot to determine if should keep moving
     * @return true if any motors are busy.
     */
    private boolean isMoving() {
        for (DcMotor motor : motorList) {
            if (motor.isBusy()) return true;
        }

        return false;
    }
}
