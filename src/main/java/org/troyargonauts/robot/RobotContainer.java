// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.troyargonauts.robot;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.troyargonauts.common.input.Gamepad;
import org.troyargonauts.common.input.gamepads.Xbox;
import org.troyargonauts.common.math.OMath;
import org.troyargonauts.common.streams.IStream;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
    // The robot's subsystems and commands are defined here...

    // Replace with Gamepad Xbox Controllers
    public final Gamepad driver = new Xbox(Constants.ControllerPorts.DRIVER_PORT);
    public final Gamepad operator = new Xbox(Constants.ControllerPorts.OPERATOR_PORT);
    
    
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer()
    {
        // Configure the trigger bindings
        configureBindings();
    }
    
    
    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings()
    {

        //Sets the Swerve Drive to the Inputs from the Driver Joysticks
        Robot.getSwerve().setDefaultCommand(
                new RunCommand(
                        () -> {
                            double forward = IStream.create(driver::getLeftY).filtered(x -> OMath.deadband(x, Constants.ControllerPorts.DEADBAND)).get();
                            double strafe = IStream.create(driver::getLeftX).filtered(x -> OMath.deadband(x, Constants.ControllerPorts.DEADBAND)).get();
                            double turn = IStream.create(driver::getRightX).filtered(x -> OMath.deadband(x, Constants.ControllerPorts.DEADBAND)).get();

                            Robot.getSwerve().setSwerve(forward, strafe, turn);
                        }, Robot.getSwerve()
                )
        );


    }
}
