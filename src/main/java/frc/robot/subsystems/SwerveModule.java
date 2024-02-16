package frc.robot.subsystems;

import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import frc.lib.config.SwerveModuleConstants;
import frc.lib.math.OnboardModuleState;
import frc.lib.util.CANCoderUtil;
import frc.lib.util.CANCoderUtil.CCUsage;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;
import frc.robot.Robot;

public class SwerveModule {

    // public ADIS16470_IMU gyro = new ADIS16470_IMU();

    public int moduleNumber;
    private Rotation2d lastAngle;
    private Rotation2d angleOffset;

    private CANSparkMax angleMotor;
    private CANSparkMax driveMotor;

    private RelativeEncoder driveEncoder;
    private RelativeEncoder integratedAngleEncoder;
    private CANcoder angleEncoder;

    private final SparkPIDController driveController;
    private final SparkPIDController angleController;

    private final SimpleMotorFeedforward feedforward =
            new SimpleMotorFeedforward(
                    Constants.Swerve.DRIVE_KS, Constants.Swerve.DRIVE_KV, Constants.Swerve.DRIVE_KA);
 
    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants,double magoffset, AbsoluteSensorRangeValue withAbsoluteSensorRange) {

        //Causes the roboRIO to stay on this code for a second giving the swerve modules a chance to configure
        // new Thread(() -> {
        //     try {
        //         Thread.sleep(1000);
        //         // gyro.setGyroAngle(ADIS16470_IMU.IMUAxis.kZ, 0);
        //     } catch (Exception e){
        //     }
        // }).start();
           


        this.moduleNumber = moduleNumber;
        angleOffset = moduleConstants.angleOffset;
        

        


        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID);

        //CANcoderConfiguration tswerveCanCoderConfig = new CANcoderConfiguration();
        //tswerveCanCoderConfig.MagnetSensor.MagnetOffset = magoffset;
        //tswerveCanCoderConfig.MagnetSensor.AbsoluteSensorRange = withAbsoluteSensorRange;
        //angleEncoder.getConfigurator().apply(tswerveCanCoderConfig);

        configAngleEncoder();
        // angleEncoder.setPositionToAbsolute();
        // angleEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);



        /* Angle Motor Config */
        angleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        integratedAngleEncoder = angleMotor.getEncoder();
        angleController = angleMotor.getPIDController();
        configAngleMotor();

        /* Drive Motor Config */
        driveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();
        driveController = driveMotor.getPIDController();
        configDriveMotor();

        lastAngle = getState().angle;



        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }




    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        // Custom optimize command, since default WPILib optimize assumes continuous controller which
        // REV and CTRE are not
        desiredState = OnboardModuleState.optimize(desiredState, getState().angle);

        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void resetToAbsolute() {
        //double absolutePosition = getCanCoder().getDegrees() - angleOffset.getDegrees();
        double absolutePosition = getCanCoder().getRotations() - angleOffset.getRotations();
        //double absolutePosition = angleEncoder.getAbsolutePosition().getValueAsDouble();
        integratedAngleEncoder.setPosition(absolutePosition);
        System.out.println("Angle offset ------- " + absolutePosition);
    }

    private void configAngleEncoder() {
        angleEncoder.getConfigurator().apply(new CANcoderConfiguration());
        CANCoderUtil.setCANCoderBusUsage(angleEncoder, CCUsage.kMinimal);
        angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCanCoderConfig);

        
    }

    private void configAngleMotor() {
        angleMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(angleMotor, Usage.kPositionOnly);
        angleMotor.setSmartCurrentLimit(Constants.Swerve.ANGLE_CONTINUOUS_CURRENT_LIMIT);
        angleMotor.setInverted(Constants.Swerve.ANGLE_INVERT);
        angleMotor.setIdleMode(Constants.Swerve.ANGLE_NEUTRAL_MODE);
        integratedAngleEncoder.setPositionConversionFactor(Constants.Swerve.ANGLE_CONVERSION_FACTOR);
        angleController.setP(Constants.Swerve.ANGLE_KP);
        angleController.setI(Constants.Swerve.ANGLE_KI);
        angleController.setD(Constants.Swerve.ANGLE_KD);
        angleController.setFF(Constants.Swerve.ANGLE_KFF);
        angleMotor.enableVoltageCompensation(Constants.Swerve.VOLTAGE_COMP);
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        angleMotor.burnFlash();
        System.out.println("------ Reset to Absolute -------");
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        resetToAbsolute();
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        System.out.println("------ Done Reset to Absolute -------");
    }

    private void configDriveMotor() {
        driveMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(driveMotor, Usage.kAll);
        driveMotor.setSmartCurrentLimit(Constants.Swerve.DRIVE_CONTINUOUS_CURRENT_LIMIT);
        driveMotor.setInverted(Constants.Swerve.DRIVE_INVERT);
        driveMotor.setIdleMode(Constants.Swerve.DRIVE_NEUTRAL_MODE);
        driveEncoder.setVelocityConversionFactor(Constants.Swerve.DRIVE_CONVERSION_VELOCITY_FACTOR);
        driveEncoder.setPositionConversionFactor(Constants.Swerve.DRIVE_CONVERSION_POSITION_FACTOR);
        driveController.setP(Constants.Swerve.ANGLE_KP);
        driveController.setI(Constants.Swerve.ANGLE_KI);
        driveController.setD(Constants.Swerve.ANGLE_KD);
        driveController.setFF(Constants.Swerve.ANGLE_KFF);
        driveMotor.enableVoltageCompensation(Constants.Swerve.VOLTAGE_COMP);
        driveMotor.burnFlash();
        driveEncoder.setPosition(0.0);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.MAX_SPEED;
            driveMotor.set(percentOutput);
        } else {
            driveController.setReference(
                    desiredState.speedMetersPerSecond,
                    ControlType.kVelocity,
                    0,
                    feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    private void setAngle(SwerveModuleState desiredState) {
        // Prevent rotating module if speed is less then 1%. Prevents jittering.
        Rotation2d angle =
                (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.MAX_SPEED * 0.01))
                        ? lastAngle
                        : desiredState.angle;

        angleController.setReference(angle.getDegrees(), ControlType.kPosition);
        lastAngle = angle;
    }

    private Rotation2d getAngle() {
        return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition() - angleOffset.getRotations());
        //return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition() + getCanCoder().getRotations() - angleOffset.getRotations());
        //return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
    }

    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition().getValueAsDouble());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveEncoder.getVelocity(), getAngle());
    }

    public SwerveModulePosition getPosition() {
        //return new SwerveModulePosition(driveEncoder.getDistance(), getAngle())
        return new SwerveModulePosition(driveEncoder.getPosition(),getAngle());
    }
}
