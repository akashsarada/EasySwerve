package org.troyargonauts.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import org.troyargonauts.robot.subsystems.DriveSubsystem;

public class PIDDistanceY extends PIDCommand {
    public PIDDistanceY(double targetX, DriveSubsystem swerve) {
        super(
                new PIDController(0.09, 0.15, 0.0006),
                // Close loop on heading
                swerve::getForwardEncoder,
                // Set reference to target
                (targetX),
                // Pipe output to turn robot
                output -> swerve.drive(output, 0, 0, false, true),
                // Require the drive
                swerve);

        getController()
                //to inches 1 inch  = 0.0254 meters
                .setTolerance(0.0125);
    }


    //0.09, 0.15, 0.0006

    @Override
    public boolean isFinished() {
        return getController().atSetpoint();
    }
}
