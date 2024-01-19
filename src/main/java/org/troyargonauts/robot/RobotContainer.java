// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.troyargonauts.robot;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import org.troyargonauts.common.input.Gamepad;
import org.troyargonauts.common.input.gamepads.Xbox;
import org.troyargonauts.common.math.OMath;
import org.troyargonauts.common.streams.IStream;
import org.troyargonauts.robot.Constants.*;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The driver's controller
  private final Gamepad driver = new Xbox(OIConstants.DRIVER_CONTROLLER_PORT);
  private final Gamepad operator = new Xbox(OIConstants.OPERATOR_CONTROLLER_PORT);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    Robot.getSwerve().setDefaultCommand(
            new RunCommand(
                    () -> {
                      double forward = IStream.create(driver::getLeftY).filtered(x -> OMath.deadband(x, Constants.OIConstants.DEADBAND)).get();
                      double strafe = IStream.create(driver::getLeftX).filtered(x -> OMath.deadband(x, Constants.OIConstants.DEADBAND)).get();
                      double turn = IStream.create(driver::getRightX).filtered(x -> OMath.deadband(x, Constants.OIConstants.DEADBAND)).get();

                      Robot.getSwerve().drive(forward, strafe, turn, false, true);
                    }, Robot.getSwerve()
            )
    );

    driver.getLeftButton().toggleOnTrue(
            new InstantCommand(Robot.getSwerve()::setX)
    );

    driver.getBottomButton().toggleOnTrue(
            new InstantCommand(Robot.getSwerve()::zeroHeading)
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(Trajectory trajectory) {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.MAX_SPEED_METERS_PER_SECOND,
        AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.DRIVE_KINEMATICS);

    // Reset odometry to the starting pose of the trajectory.
    Robot.getSwerve().resetOdometry(trajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return Robot.getSwerve().driveToTrajectory(trajectory).andThen(() -> Robot.getSwerve().drive(0, 0, 0, false, false));
  }
}
