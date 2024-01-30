package org.troyargonauts.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import org.troyargonauts.robot.Constants;
import org.troyargonauts.robot.subsystems.DriveSubsystem;

public class RotatePIDCommand extends PIDCommand {
    /**
     * Turns to robot to the specified angle.
     *
     * @param targetAngleDegrees The angle to turn to
     * @param drive The drive subsystem to use
     */
    public RotatePIDCommand(double targetAngleDegrees, DriveSubsystem drive) {
        super(
                new PIDController(Constants.RotateConstants.ROTATE_P, Constants.RotateConstants.ROTATE_I, Constants.RotateConstants.ROTATE_D),
                // Close loop on heading
                drive::getHeading,
                // Set reference to target
                targetAngleDegrees,
                // Pipe output to turn robot
                output -> drive.drive(0, 0, output, false, true),
                // Require the drive
                drive);

        // Set the controller to be continuous (because it is an angle controller)
        getController().enableContinuousInput(-180, 180);
        // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
        // setpoint before it is considered as having reached the reference
        getController()
                .setTolerance(Constants.RotateConstants.TURN_TOLERANCE_DEG, Constants.RotateConstants.TURN_RATE_TOLERANCE_DEG_PER_SEC);
    }

    @Override
    public boolean isFinished() {
        // End when the controller is at the reference.
        return getController().atSetpoint();
    }
}
