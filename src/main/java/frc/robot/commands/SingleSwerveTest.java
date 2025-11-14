package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveModule;

public class SingleSwerveTest extends Command {
    SwerveModule _module;

    public SingleSwerveTest(SwerveModule module) {
        _module = module;
        addRequirements(_module);
    }
    
    @Override
    public void execute() {
        double vx = RobotContainer.driver.getLeftX();
        double vy = RobotContainer.driver.getLeftY();

        double vel = Math.sqrt(vx*vx + vy*vy);
        double rot = -Math.atan2(vx, -vy);

        Rotation2d angle = new Rotation2d(rot);

        SmartDashboard.putNumber("raw rot", angle.getRotations());

        _module.setState(vel, angle);
    }
}
