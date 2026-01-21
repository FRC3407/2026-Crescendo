// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Meter;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.vision.VisionCamera;

/** Default command for driving the {@code DriveSubystem} using joysticks */
public class DriveCommand extends Command {

    private final DriveSubsystem driveSubsystem;
    private final VisionSubsystem visionSubsystem;
    private final DoubleSupplier forwardStick;
    private final DoubleSupplier sidewaysStick;
    private final DoubleSupplier rotStick;
    private final BooleanSupplier targeting_switch;
    private final VisionCamera lifeCamera;

    /**
     * Drive the robot using joysticks.
     * 
     * @param forwardStick Joystick for forward translation.
     * @param sidewaysStick Joystick for sideways translation.
     * @param rotStick Joystick axis for rotation.
     * @param drive DriveSubsystem
     */
    public DriveCommand(DoubleSupplier forwardStick, DoubleSupplier sidewaysStick, DoubleSupplier rotStick,
            BooleanSupplier targeting_switch, DriveSubsystem drive,  VisionSubsystem vision) {
        this.forwardStick = forwardStick;
        this.sidewaysStick = sidewaysStick;
        this.rotStick = rotStick;
        this.driveSubsystem = drive;
        this.visionSubsystem = vision;
        this.lifeCamera = vision.cameraList.get(1);
        this.targeting_switch = targeting_switch;
        addRequirements(this.driveSubsystem);
    }

    @Override
    public void execute() {
        double xSpeed = MathUtil.applyDeadband(sidewaysStick.getAsDouble(), OIConstants.kDriveDeadband) * -1;
        double ySpeed = MathUtil.applyDeadband(forwardStick.getAsDouble(), OIConstants.kDriveDeadband);
        double rot = MathUtil.applyDeadband(rotStick.getAsDouble(), OIConstants.kDriveDeadband) * -1;
        
        if (targeting_switch.getAsBoolean()) {
            AprilTag tag = lifeCamera.getBestTag(9);
            Pose2d tagpose = tag.pose.toPose2d();
            if (tag != null) {
                Pose2d currentpose = driveSubsystem.getPose();
                Rotation2d ang = currentpose.getRotation();
                Distance y = currentpose.getMeasureY();
                Distance x = currentpose.getMeasureX();
                Distance tagx = tagpose.getMeasureX();
                Distance tagy = tagpose.getMeasureY();
                Distance deltax = tagx.minus(x);
                Distance deltay = tagy.minus(y);
                Double ang_to_target = Math.atan2(deltay.in(Meter), deltax.in(Meter));
                Rotation2d angle_to_target_radians = new Rotation2d(ang_to_target);
                Rotation2d relative_rotation = ang.relativeTo(angle_to_target_radians);
                rot = -relative_rotation.getRadians()/Math.PI;
            }
        }
        driveSubsystem.drive(xSpeed, ySpeed, rot, true);
    }
}