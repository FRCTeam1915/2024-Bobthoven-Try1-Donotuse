package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Swerve extends SubsystemBase {
    private final Pigeon2 gyro;

    private SwerveDriveOdometry swerveOdometry;
    private SwerveModule[] mSwerveMods;

    private Field2d field;

    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.PIGEON_ID);
        gyro.configFactoryDefault();
        zeroGyro();

        mSwerveMods = new SwerveModule[] {
                new SwerveModule(0, Constants.Swerve.Mod0.CONSTANTS),
                new SwerveModule(1, Constants.Swerve.Mod1.CONSTANTS),
                new SwerveModule(2, Constants.Swerve.Mod2.CONSTANTS),
                new SwerveModule(3, Constants.Swerve.Mod3.CONSTANTS)
        };

        swerveOdometry = new SwerveDriveOdometry(
                Constants.Swerve.SWERVE_KINEMATICS,
                getYaw(),
                new SwerveModulePosition[] {
                        mSwerveMods[0].getPosition(),
                        mSwerveMods[1].getPosition(),
                        mSwerveMods[2].getPosition(),
                        mSwerveMods[3].getPosition()
                });

        field = new Field2d();
        SmartDashboard.putData("Field", field);
    }

    public void drive(
            Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
                Constants.Swerve.SWERVE_KINEMATICS.toSwerveModuleStates(
                        fieldRelative
                                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                        translation.getX(), translation.getY(), rotation, getYaw())
                                : new ChassisSpeeds(
                                        translation.getX(), translation.getY(), rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.MAX_SPEED);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.MAX_SPEED);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        //swerveOdometry.resetPosition(pose, getYaw());
        swerveOdometry.resetPosition(
                getYaw(),
                new SwerveModulePosition[] {
                        mSwerveMods[0].getPosition(),
                        mSwerveMods[1].getPosition(),
                        mSwerveMods[2].getPosition(),
                        mSwerveMods[3].getPosition()
                },
                pose
        );
    }

    public SwerveModuleState[] getStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public void zeroGyro() {
        gyro.setYaw(0);
    }

    public Rotation2d getYaw() {
        return (Constants.Swerve.INVERT_GYRO)
                ? Rotation2d.fromDegrees(360 - gyro.getYaw())
                : Rotation2d.fromDegrees(gyro.getYaw());
    }

    @Override
    public void periodic() {
        swerveOdometry.update(
                getYaw(),
                new SwerveModulePosition[] {
                        mSwerveMods[0].getPosition(),
                        mSwerveMods[1].getPosition(),
                        mSwerveMods[2].getPosition(),
                        mSwerveMods[3].getPosition()
                });

        field.setRobotPose(getPose());

        for (SwerveModule mod : mSwerveMods) {
            SmartDashboard.putNumber(
                    "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber(
                    "Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
            SmartDashboard.putNumber(
                    "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        }
    }
}
