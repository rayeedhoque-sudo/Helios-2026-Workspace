// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import frc.robot.Constants.TunerConstants;
import frc.robot.commands.CommandSwerveDrivetrain;
import frc.robot.subsystems.misc.HopperSubsystem;
import frc.robot.subsystems.misc.IntakeSubsystem;
import frc.robot.subsystems.misc.ShooterSubsystem;
import frc.robot.subsystems.misc.HopperSubsystem.HOPPERSTATE;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired tp speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick2 = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final ShooterSubsystem shoterSS = new ShooterSubsystem(false, drivetrain);
    public final IntakeSubsystem intakeSS = new IntakeSubsystem();
    public final HopperSubsystem hopperSS = new HopperSubsystem();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        
        //DRIVE SUBSYSTEM
            // Note that X is defined as forward according to WPILib convention,
            // and Y is defined as to the left according to WPILib convention.
                drivetrain.setDefaultCommand(
                    // Drivetrain will execute this command periodically
                    drivetrain.applyRequest(() ->
                        drive.withVelocityX(-joystick2.getLeftY() * MaxSpeed * 0.5) // Drive forward with negative Y (forward)
                            .withVelocityY(-joystick2.getLeftX() * MaxSpeed * 0.5) // Drive left with negative X (left)
                            .withRotationalRate(-joystick2.getRightX() * MaxAngularRate) // Drive counterclockwise 
                    )
                );
                
                //Auto Align
                // joystick2.a().onTrue(drivetrain.rotateToAngle(shoterSS.getDegreesToAlignToTarget()));
            // Idle while the robot is disabled. This ensures the configured
            // neutral mode is applied to the drive motors while disabled.
                final var idle = new SwerveRequest.Idle();
                RobotModeTriggers.disabled().whileTrue(
                    drivetrain.applyRequest(() -> idle).ignoringDisable(true)   
                ); 

            //Drive Train Break
                joystick2.start().whileTrue(drivetrain.applyRequest(() -> brake));

            // Reset the field-centric heading on left bumper press.
                joystick2.povCenter().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

            drivetrain.registerTelemetry(logger::telemeterize);

        //IntakeSubsystem 
            joystick2.rightBumper().whileTrue(intakeSS.intakeCommand()).whileFalse(intakeSS.stowCommand());
            joystick2.leftBumper().whileTrue(intakeSS.outtakeCommand()).whileFalse(intakeSS.stowCommand());
        //Hopper Subsystem
            joystick2.x().whileTrue(hopperSS.setHopperState(HOPPERSTATE.RUN)).onFalse(hopperSS.setHopperState(HOPPERSTATE.STOW));
        // Shooter Subsystem

            joystick2.rightTrigger().whileTrue(
                shoterSS.enableLiveData(true)
            ).whileFalse(
                shoterSS.enableLiveData(false)
            );
    }

    public Command getAutonomousCommand() {
        return Commands.none();
    }
}