// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.troyargonauts.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import org.troyargonauts.common.input.Gamepad;
import org.troyargonauts.common.input.gamepads.Xbox;
import org.troyargonauts.common.math.OMath;
import org.troyargonauts.common.streams.IStream;
import org.troyargonauts.robot.Constants.*;
import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The driver's controller
  private final Gamepad driver = new Xbox(OIConstants.DRIVER_CONTROLLER_PORT);

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

                      Robot.getSwerve().drive(forward, strafe, turn, true, true);
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
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.DRIVE_KINEMATICS);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        Robot.getSwerve()::getPose, // Functional interface to feed supplier
        DriveConstants.DRIVE_KINEMATICS,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        Robot.getSwerve()::setModuleStates,
        Robot.getSwerve());

    // Reset odometry to the starting pose of the trajectory.
    Robot.getSwerve().resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> Robot.getSwerve().drive(0, 0, 0, false, false));
  }
}
