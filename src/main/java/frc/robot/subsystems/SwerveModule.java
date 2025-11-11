package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class SwerveModule extends SubsystemBase {

    private final TalonFX _steerMotor;
    private final TalonFX _driveMotor;

    private final TalonFXConfiguration _steerMotorConfig;
    private final TalonFXConfiguration _driveMotorConfig;

    private final VelocityVoltage _driveVelocityVoltage = new VelocityVoltage(0).withSlot(0);

    private final Translation2d _moduleLocation;
    private final int _moduleIndex;
    private SwerveModuleState _desiredModuleState;
    private Rotation2d _steerAngularOffset;

    // ======================================================================================

    public SwerveModule(int steerPort, int drivePort, int encoderPort, ModuleConfiguration config) {
        if(!config.canBus.equals("")) {
            _steerMotor = new TalonFX(steerPort, config.canBus);
            _driveMotor = new TalonFX(drivePort, config.canBus);
        } else {
            _steerMotor = new TalonFX(steerPort);
            _driveMotor = new TalonFX(drivePort);
        }

        // -------------------------------------------------------------------------------------

        _steerMotorConfig = new TalonFXConfiguration();
        _steerMotorConfig.Feedback.FeedbackRemoteSensorID = encoderPort;
        _steerMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        _steerMotorConfig.Feedback.SensorToMechanismRatio = Constants.Drivetrain.Hardware.STEER_SENSOR_TO_MECHANISM_RATIO;
        _steerMotorConfig.Feedback.RotorToSensorRatio = Constants.Drivetrain.Hardware.STEER_ROTOR_TO_SENSOR_RATIO;
        _steerMotorConfig.MotionMagic.withMotionMagicCruiseVelocity(RotationsPerSecond.of(5))
            .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(10))
            .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100));
        _steerMotorConfig.Slot0.kS = Constants.Drivetrain.STEER_KS;
        _steerMotorConfig.Slot0.kV = Constants.Drivetrain.STEER_KV;
        _steerMotorConfig.Slot0.kA = Constants.Drivetrain.STEER_KA;
        _steerMotorConfig.Slot0.kP = Constants.Drivetrain.STEER_KP;
        _steerMotorConfig.Slot0.kI = Constants.Drivetrain.STEER_KI;
        _steerMotorConfig.Slot0.kD = Constants.Drivetrain.STEER_KD;
        _steerMotorConfig.Voltage.withPeakForwardVoltage(Volts.of(Constants.Drivetrain.STEER_PEAK_VOLTAGE))
            .withPeakReverseVoltage(Volts.of(-Constants.Drivetrain.STEER_PEAK_VOLTAGE));

        _driveMotorConfig = new TalonFXConfiguration();
        _driveMotorConfig.Feedback.SensorToMechanismRatio = Constants.Drivetrain.Hardware.DRIVE_SENSOR_TO_MECHANISM_RATIO;
        _driveMotorConfig.Feedback.RotorToSensorRatio = Constants.Drivetrain.Hardware.DRIVE_ROTOR_TO_SENSOR_RATIO;
        _driveMotorConfig.Slot0.kS = Constants.Drivetrain.DRIVE_KS;
        _driveMotorConfig.Slot0.kV = Constants.Drivetrain.DRIVE_KV;
        _driveMotorConfig.Slot0.kP = Constants.Drivetrain.DRIVE_KP;
        _driveMotorConfig.Slot0.kI = Constants.Drivetrain.DRIVE_KI;
        _driveMotorConfig.Slot0.kD = Constants.Drivetrain.DRIVE_KD;
        _driveMotorConfig.Voltage.withPeakForwardVoltage(Volts.of(Constants.Drivetrain.DRIVE_PEAK_VOLTAGE))
            .withPeakReverseVoltage(Volts.of(-Constants.Drivetrain.DRIVE_PEAK_VOLTAGE));

        // -------------------------------------------------------------------------------------

        StatusCode steerMotorStatus = StatusCode.StatusCodeNotInitialized;
        for (int i = 0;i < 5;i++) {
            steerMotorStatus = _steerMotor.getConfigurator().apply(_steerMotorConfig);
            if (steerMotorStatus.isOK()) break;}
        if(!steerMotorStatus.isOK()) {System.out.println("Failed to apply steer motor config for module " + config.moduleName + " after 5 attempts. Status: " + steerMotorStatus);}
        StatusCode driveMotorStatus = StatusCode.StatusCodeNotInitialized;
        for (int i = 0;i < 5;i++) {
            driveMotorStatus = _driveMotor.getConfigurator().apply(_driveMotorConfig);
            if (driveMotorStatus.isOK()) break;}
        if(!driveMotorStatus.isOK()) {System.out.println("Failed to apply drive motor config for module " + config.moduleName + " after 5 attempts. Status: " + driveMotorStatus);}

        // -------------------------------------------------------------------------------------

        _moduleLocation = config.position;
        _moduleIndex = config.index;
        _desiredModuleState = new SwerveModuleState(0.0, new Rotation2d(0.0));
        _steerAngularOffset = config.encoderOffset;
    }

    // ======================================================================================

    public void setState(SwerveModuleState state) {
        SwerveModuleState optimizedState = new SwerveModuleState(
            state.speedMetersPerSecond, 
            state.angle.plus(_steerAngularOffset)
        );
        optimizedState.angle = optimizedState.angle.plus(_steerAngularOffset);
        optimizedState.optimize(getSteerAngle().minus(_steerAngularOffset));
        double driveRotationsPerSecond = optimizedState.speedMetersPerSecond 
            / (Constants.Drivetrain.Hardware.WHEEL_DIAMETER_METER * Math.PI);

        _steerMotor.setPosition(optimizedState.angle.getRotations());

        _driveMotor.setControl(_driveVelocityVoltage.withVelocity(driveRotationsPerSecond));

        _desiredModuleState = state;
    }

    public void setState(double speedMetersPerSecond, Rotation2d angle) {
        setState(new SwerveModuleState(speedMetersPerSecond, angle));
    }

    // ======================================================================================

    public Rotation2d getSteerAngle() {
        return Rotation2d.fromRadians(_steerMotor.getPosition().getValue().in(Units.Radians)).minus(_steerAngularOffset);
    }

    public SwerveModuleState getDesiredState() {
        return _desiredModuleState;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition (
            _driveMotor.getPosition().getValue().in(Units.Rotations) * Constants.Drivetrain.Hardware.WHEEL_DIAMETER_METER * Math.PI,
            getSteerAngle()
        );
    }

    public Translation2d getLocation() {
        return _moduleLocation;
    }

    public int getIndex() {
        return _moduleIndex;
    }
    
    // ======================================================================================

    public static class ModuleConfiguration {
        public String moduleName = "";
        public int index = 0;
        public Translation2d position = new Translation2d();
        public Rotation2d encoderOffset = new Rotation2d(0.0);
        public boolean encoderInverted = false;
        public String canBus = "";
    }
}
