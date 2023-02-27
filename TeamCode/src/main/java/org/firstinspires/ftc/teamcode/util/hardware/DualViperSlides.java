package org.firstinspires.ftc.teamcode.util.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.subsystems.Slide;

// TODO abstract more of this stuff to go into a separate class
public class DualViperSlides extends Slide {
    private final DcMotor leftSlide;
    private final DcMotor rightSlide;

    private SlideMode slideMode;

    public DualViperSlides(HardwareMap hardwareMap) {
        leftSlide = hardwareMap.get(DcMotor.class, "LS");
        rightSlide = hardwareMap.get(DcMotor.class, "RS");

        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // prevents slides from sliding down from gravity
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public enum SlideMode {
        ENCODERS, // move slides using encoders and RUN_TO_POSITION
        MANUAL // move slides with the stick instead
    }

    public SlideMode getSlideMode() {
        return slideMode;
    }
    public void setSlideMode(SlideMode slideMode) {
        this.slideMode = slideMode;
    }

    public int getPosition() {
        return (leftSlide.getCurrentPosition() + rightSlide.getCurrentPosition()/2);
    }

    public int getTarget() {
        return leftSlide.getTargetPosition();
    }

    public void setPosition(int targetHeight) {
        leftSlide.setTargetPosition(targetHeight);
        rightSlide.setTargetPosition(targetHeight);

        leftSlide.setPower(0.7);
        rightSlide.setPower(0.7);

        setSlideMode(SlideMode.ENCODERS);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void moveSlides(double power) {
        leftSlide.setPower(power);
        rightSlide.setPower(power);

        setSlideMode(SlideMode.MANUAL);
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
