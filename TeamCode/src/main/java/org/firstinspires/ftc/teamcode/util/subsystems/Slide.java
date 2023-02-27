package org.firstinspires.ftc.teamcode.util.subsystems;

public abstract class Slide extends Subsystem {
    public abstract int getPosition();
    public abstract int getTarget();

    public abstract void setPosition(int ticks);
    public abstract void moveSlides(double power);
}
