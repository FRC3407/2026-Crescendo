// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;

/** Default command for driving the {@code DriveSubystem} using joysticks */
public class DriveCommand extends Command {

    private final DriveSubsystem driveSubsystem;
    private final DoubleSupplier forwardStick;
    private final DoubleSupplier sidewaysStick;
    private final DoubleSupplier rotStick;

    /**
     * Drive the robot using joysticks.
     * 
     * @param forwardStick Joystick for forward translation.
     * @param sidewaysStick Joystick for sideways translation.
     * @param rotStick Joystick axis for rotation.
     * @param drive DriveSubsystem
     */
    public DriveCommand(DoubleSupplier forwardStick, DoubleSupplier sidewaysStick, DoubleSupplier rotStick,
            DriveSubsystem drive) {
        this.forwardStick = forwardStick;
        this.sidewaysStick = sidewaysStick;
        this.rotStick = rotStick;
        this.driveSubsystem = drive;
        SmartDashboard.putData(this);
        addRequirements(this.driveSubsystem);
    }

    @Override
    public void execute() {
        double xSpeed = Math.pow(
            MathUtil.applyDeadband(sidewaysStick.getAsDouble(),OIConstants.kDriveDeadband) * -1,
            OIConstants.kDriveExponent) * Math.signum(sidewaysStick.getAsDouble());
        double ySpeed = Math.pow(
            MathUtil.applyDeadband(forwardStick.getAsDouble(),OIConstants.kDriveDeadband) * -1,
            OIConstants.kDriveExponent) * Math.signum(forwardStick.getAsDouble());
        double rot = Math.pow(
            MathUtil.applyDeadband(rotStick.getAsDouble(), OIConstants.kDriveRotationDeadband),
            2)*-Math.signum(rotStick.getAsDouble()) * OIConstants.kDriveRotationSpeed;
        driveSubsystem.drive(xSpeed, ySpeed, rot, true);
    }

    @Override
    public void initSendable(SendableBuilder sendableBuilder){
        super.initSendable(sendableBuilder);
        sendableBuilder.addDoubleProperty("X speed",()->sidewaysStick.getAsDouble(),null);
        sendableBuilder.addDoubleProperty("Y speed",()->forwardStick.getAsDouble(),null);
        sendableBuilder.addDoubleProperty("Rotation",()->rotStick.getAsDouble(),null);
    }
}
