package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class SwerveModule extends SubsystemBase {
    private final TalonFX _steerMotor;
    private final TalonFX _driveMotor;

    private final TalonFXConfiguration _steerMotorConfig = new TalonFXConfiguration();

    private final Translation2d _moduleLocation;
    private SwerveModuleState _desiredModuleState;
    private double _steerAngularOffset;

    public SwerveModule(int steerPort, int drivePort, int encoderPort, ModuleConfiguration config) {
        _steerMotor = new TalonFX(steerPort, config.canBus);
        _driveMotor = new TalonFX(drivePort, config.canBus);
        
        _steerMotorConfig.Feedback.FeedbackRemoteSensorID = encoderPort;
        _steerMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        _steerMotorConfig.Feedback.SensorToMechanismRatio = 1.0;
        _steerMotorConfig.Feedback.RotorToSensorRatio = 1.0;

        _steerMotor.getConfigurator().apply(_steerMotorConfig);

        _moduleLocation = config.position;
        _desiredModuleState = new SwerveModuleState(0.0, new Rotation2d(0.0));
        _steerAngularOffset = config.encoderOffset;
    }

    public void setModuleState(SwerveModuleState state) {
        SwerveModuleState optimizedState = state;
        optimizedState.angle = optimizedState.angle.plus(Rotation2d.fromRadians(_steerAngularOffset));
        optimizedState.optimize(Rotation2d.fromRadians(_steerMotor.getPosition().getValue().in(Units.Radians)));
        PositionDutyCycle steerRequest = new PositionDutyCycle(optimizedState.angle.getRadians());
        _steerMotor.setControl(steerRequest);
        VelocityDutyCycle driveRequest = new VelocityDutyCycle(optimizedState.speedMetersPerSecond);
        _driveMotor.setControl(driveRequest);

        _desiredModuleState = state;
    }

    public SwerveModuleState getDesiredSwerveModuleState() {
        return _desiredModuleState;
    }


    public static class ModuleConfiguration {
        public String moduleName = "";

        public Translation2d position = new Translation2d();

        public double encoderOffset = 0.0;
        public boolean encoderInverted = false;

        public String canBus = "CANivore";
    }
}
