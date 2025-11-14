package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import java.util.Optional;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Drivetrain extends SubsystemBase {
    public static final int FL_IDX = 0;
    public static final int FR_IDX = 1;
    public static final int BL_IDX = 2;
    public static final int BR_IDX = 3;

    private SwerveModule[] _modules;
    private final Pigeon2 _gyro;

    private ChassisSpeeds _desiredChassisSpeeds;
    private SwerveModulePosition[] _measuredPositions;
    private Rotation2d _yaw;

    private double _yawOffset;
    private Rotation2d _headingTarget;
    private final PIDController _headingController;

    private Pose2d _currentPose;
    private Pose2d _previousPose;

    private final SwerveDriveKinematics _kinematics;
    private final SwerveDrivePoseEstimator _odometry;

    private final boolean _isRedAlliance;

    private double _lastTime;

    StructArrayPublisher<SwerveModuleState> _desiredSwerveStatePublisher;
    StructArrayPublisher<Pose2d> _currentPosePublisher;

    public Drivetrain() {
        _modules = new SwerveModule[4];
        _modules[FL_IDX] = new SwerveModule(RobotMap.CAN.FL_STEER, RobotMap.CAN.FL_DRIVE, RobotMap.CAN.FL_ENCODER, Constants.Drivetrain.FL_CONFIG);
        _modules[FR_IDX] = new SwerveModule(RobotMap.CAN.FR_STEER, RobotMap.CAN.FR_DRIVE, RobotMap.CAN.FR_ENCODER, Constants.Drivetrain.FR_CONFIG);
        _modules[BL_IDX] = new SwerveModule(RobotMap.CAN.BL_STEER, RobotMap.CAN.BL_DRIVE, RobotMap.CAN.BL_ENCODER, Constants.Drivetrain.BL_CONFIG);
        _modules[BR_IDX] = new SwerveModule(RobotMap.CAN.BR_STEER, RobotMap.CAN.BR_DRIVE, RobotMap.CAN.BR_ENCODER, Constants.Drivetrain.BR_CONFIG);
        _gyro = new Pigeon2(RobotMap.CAN.PIGEON);

        // -------------------------------------------------------------------------------------

        _desiredChassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
        _measuredPositions = new SwerveModulePosition[4];
        _yaw = new Rotation2d(0.0);

        _yawOffset = _gyro.getYaw().getValueAsDouble() * Constants.Drivetrain.PIGEON_INVERTED;
        _headingTarget = Rotation2d.fromDegrees(_yawOffset);
        _headingController = new PIDController(Constants.Drivetrain.HEADING_KP, Constants.Drivetrain.HEADING_KI, Constants.Drivetrain.HEADING_KD);
        _headingController.enableContinuousInput(-180.0, 180.0);

        _currentPose = new Pose2d();
        _previousPose = new Pose2d();

        _kinematics = new SwerveDriveKinematics(
            _modules[FL_IDX].getLocation(),
            _modules[FR_IDX].getLocation(),
            _modules[BL_IDX].getLocation(),
            _modules[BR_IDX].getLocation());

        _odometry = new SwerveDrivePoseEstimator(_kinematics, _headingTarget, _measuredPositions, _currentPose);

        // -------------------------------------------------------------------------------------

        Optional<Alliance> alliance = DriverStation.getAlliance();
        _isRedAlliance = alliance.filter(value -> value == Alliance.Red).isPresent();

        _lastTime = Timer.getFPGATimestamp();

        _desiredSwerveStatePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("DesiredSwerveStates", SwerveModuleState.struct).publish();
        _currentPosePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("CurrentPose", Pose2d.struct).publish();
    }

    @Override
    public void periodic() {
        updateSpeeds(_desiredChassisSpeeds);
        updateOdometry();
        readIMU();
    }

    // =======================================================================================

    private void updateSpeeds(ChassisSpeeds speeds) {
        if(speeds == null) {
            speeds = new ChassisSpeeds();
        }
        SwerveModuleState desiredStates[] = _kinematics.toSwerveModuleStates(speeds);
        setModuleStates(desiredStates);
    }
    
    public void setRobotRelativeSpeeds(ChassisSpeeds speeds) {
        _desiredChassisSpeeds = speeds;
    }

    public void setRobotRelativeSpeeds(double vx, double vy, double omega) {
        setRobotRelativeSpeeds(new ChassisSpeeds(vx, vy, omega));
    }

    public void setFieldRelativeSpeeds(ChassisSpeeds fieldRelativeSpeeds) {
        _headingTarget = _headingTarget.plus(Rotation2d.fromRadians(fieldRelativeSpeeds.omegaRadiansPerSecond * getDeltaT()));
        double headingCorrectionDegrees = _headingController.calculate(getYaw().getDegrees(), _headingTarget.getDegrees());
        fieldRelativeSpeeds.omegaRadiansPerSecond += Radians.convertFrom(headingCorrectionDegrees, Degrees);
        ChassisSpeeds robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            fieldRelativeSpeeds,
            getYaw()
        );
        setRobotRelativeSpeeds(robotRelativeSpeeds);
    }

    public void setFieldRelativeSpeeds(double vx, double vy, double omega) {
        setFieldRelativeSpeeds(new ChassisSpeeds(vx, vy, omega));
    }

    private void setModuleStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Drivetrain.MAX_VELOCITY_METERS_PER_SECOND);
        for(SwerveModule module : _modules) {
            module.setState(states[module.getIndex()]);
        }
        _desiredSwerveStatePublisher.set(states);
    }

    public void stop() {
        setRobotRelativeSpeeds(new ChassisSpeeds(0.0, 0.0, 0.0));
    }

    // =======================================================================================

    public void zeroIMU() {
        _yawOffset = _gyro.getYaw().getValueAsDouble() * Constants.Drivetrain.PIGEON_INVERTED;
        readIMU();
    }

    public void readIMU() {
        double yawRobot = _gyro.getYaw().getValueAsDouble() * Constants.Drivetrain.PIGEON_INVERTED;
        double yawAllianceOffset = _isRedAlliance ? 180.0 : 0.0;
        _yaw = Rotation2d.fromDegrees(yawRobot - _yawOffset + yawAllianceOffset);
    }

    public Rotation2d getYaw() {
        return _yaw;
    }

    // =======================================================================================

    public void updateOdometry() {
        for (SwerveModule module : _modules) {
            _measuredPositions[module.getIndex()] = module.getPosition();
        }

        _odometry.updateWithTime(Timer.getFPGATimestamp(), getYaw(), _measuredPositions);

        _previousPose = _currentPose;
        _currentPose = _odometry.getEstimatedPosition();

        _currentPosePublisher.set(new Pose2d[]{_currentPose});
    }

    public double getVelocityMagnitude() {
        return (_currentPose.getTranslation().getDistance(_previousPose.getTranslation())) * (1.0 / getDeltaT());
    }

    public Pose2d getPose() {
        return _currentPose;
    }

    public void resetPose(Pose2d pose) {
        _odometry.resetPosition(getYaw(), new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
        }, pose);
        _currentPose = pose;
        _previousPose = pose;
    }

    private double getDeltaT() {
        double currentTime = Timer.getFPGATimestamp();
        double dt = currentTime - _lastTime;
        _lastTime = currentTime;
        return dt;
    }
}
