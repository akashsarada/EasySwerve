package org.troyargonauts.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.troyargonauts.common.math.Angle;
import org.troyargonauts.common.math.Vector2D;

public class Swerve extends SubsystemBase{
    public Swerve() {}

    @Override
    public void periodic() {}

    public double calculatePower(double forward, double strafe, double turn) {
        Vector2D leftStickInput = new Vector2D(strafe, forward);
        double leftStickInputDistance = leftStickInput.distance();

        return leftStickInputDistance + (turn / 2);
    }

    public double calculateAngle(double strafe, double turn) {
        Vector2D leftStickInput = new Vector2D(strafe, 0);
        Angle leftStickInputAngle = leftStickInput.getAngle();

        double turnAngle = turn * 180;

        return leftStickInputAngle.toDegrees() + turnAngle;
    }

}

