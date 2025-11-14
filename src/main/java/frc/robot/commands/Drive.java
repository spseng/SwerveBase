package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class Drive extends Command {
    Drivetrain _drivetrain;

    public Drive(Drivetrain drivetrain) {
        _drivetrain = drivetrain;
        addRequirements(_drivetrain);
    }
    
    @Override
    public void execute() {
        double vx = -RobotContainer.driver.getLeftY();
        double vy = -RobotContainer.driver.getLeftX();

        double rot = -RobotContainer.driver.getRightX();

        _drivetrain.setFieldRelativeSpeeds(new ChassisSpeeds(vx, vy, rot));
    }
}
