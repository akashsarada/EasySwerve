package org.troyargonauts.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import org.troyargonauts.robot.subsystems.DriveSubsystem;

public class PIDDistanceX extends PIDCommand {
    public PIDDistanceX(double targetY, DriveSubsystem swerve) {
        super(
                new PIDController(0.09, 0.15, 0.0006),
                // Close loop on heading
                swerve::getStrafeEncoder,
                // Set reference to target
                (targetY),
                // Pipe output to turn robot
                output -> swerve.drive(0, -output, 0, false, true),
                // Require the drive
                swerve);



        getController().setTolerance(0.0125);
    }

    @Override
    public boolean isFinished() {
        return getController().atSetpoint();
    }
}
