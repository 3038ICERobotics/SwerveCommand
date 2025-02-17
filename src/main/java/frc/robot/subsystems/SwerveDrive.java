package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase {
    SparkMax[] steerMotors = new SparkMax[4];
    SparkMax[] driveMotors = new SparkMax[4];
     public RelativeEncoder[] steerEncoders = new RelativeEncoder[4];
    Translation2d FrontLeftDriveLocation = new Translation2d(0.285, 0.285);
    Translation2d FrontRightDriveLocation = new Translation2d(0.285, -0.285);
    Translation2d BackLeftDriveLocation = new Translation2d(-0.285, 0.285);
    Translation2d BackRightDriveLocation = new Translation2d(-0.285, -0.285);
    SwerveDriveKinematics Kinematics = new SwerveDriveKinematics(FrontLeftDriveLocation, BackLeftDriveLocation,
            FrontRightDriveLocation, BackRightDriveLocation);
    public final double MaxDriveSpeed = .5;
    private final double RotationsPerMeter = 27;
    private final int SecondsPerMinute = 60;
    public final double MaxTurnSpeed = .5;
    private final double GearRatio = 55;
    public AnalogContainer[] analogs = new AnalogContainer[4];
    private SparkClosedLoopController[] PIDDrive = new SparkClosedLoopController[4];
    private SparkClosedLoopController[] PIDSteer = new SparkClosedLoopController[4];
    SparkBaseConfig SteeringBaseConfig[] = new SparkMaxConfig[4];
    SparkBaseConfig VelocityBaseConfig[] = new SparkMaxConfig[4];
    public ClosedLoopConfig VelocityLoopConfig = new ClosedLoopConfig();
    public ClosedLoopConfig SteeringLoopConfig = new ClosedLoopConfig();

    public enum ModuleOrder {
        FL, BL, FR, BR
    };

    /** Creates a new ExampleSubsystem. */
    public SwerveDrive() {
        int index = 0;
        driveMotors[index++] = new SparkMax(10, MotorType.kBrushless);// FL
        driveMotors[index++] = new SparkMax(22, MotorType.kBrushless);// BL
        driveMotors[index++] = new SparkMax(3, MotorType.kBrushless);// FR
        driveMotors[index++] = new SparkMax(14, MotorType.kBrushless);// BR
        index = 0;
        steerMotors[index++] = new SparkMax(15, MotorType.kBrushless);// FL
        steerMotors[index++] = new SparkMax(8, MotorType.kBrushless);// BL
        steerMotors[index++] = new SparkMax(1, MotorType.kBrushless);// FR
        steerMotors[index++] = new SparkMax(2, MotorType.kBrushless);// BR

        analogs[0] = new AnalogContainer(steerMotors[0].getAnalog(), 2.23, 1.60);
        analogs[1] = new AnalogContainer(steerMotors[1].getAnalog(), 2.23, 2.18);
        analogs[2] = new AnalogContainer(steerMotors[2].getAnalog(), 2.27, 1.78);
        analogs[3] = new AnalogContainer(steerMotors[3].getAnalog(), 2.20, 0.12);

        for (int i = 0; i < 4; i++) {
            steerEncoders[i] = steerMotors[i].getEncoder();
        }
        PIDInit();
    }

    public void PIDInit() {

        VelocityLoopConfig.pidf(0.000170, 0.000001, 0.000020, 0.000001, ClosedLoopSlot.kSlot0);
        VelocityLoopConfig.outputRange(-1, 1, ClosedLoopSlot.kSlot0);

        SteeringLoopConfig.pidf(1, .5, .1, .000001, ClosedLoopSlot.kSlot0);
        SteeringLoopConfig.outputRange(-1, 1, ClosedLoopSlot.kSlot0);

        for (int i = 0; i < 4; i++) {
            VelocityBaseConfig[i] = new SparkMaxConfig();
            VelocityBaseConfig[i].apply(VelocityLoopConfig);
            if (i == 1) {
                VelocityBaseConfig[i].inverted(true);
            }
            SteeringBaseConfig[i] = new SparkMaxConfig();
            SteeringBaseConfig[i].apply(SteeringLoopConfig);
            SteeringBaseConfig[i].smartCurrentLimit(15); // Default limit is 80A - this limit is too high for a NEO 550
            SteeringBaseConfig[i].encoder
                    .positionConversionFactor(1)
                    .velocityConversionFactor(1);
            driveMotors[i].configure(VelocityBaseConfig[i], ResetMode.kResetSafeParameters,
                    PersistMode.kPersistParameters);
            steerMotors[i].configure(SteeringBaseConfig[i], ResetMode.kResetSafeParameters,
                    PersistMode.kPersistParameters);
            PIDDrive[i] = driveMotors[i].getClosedLoopController();
            PIDSteer[i] = steerMotors[i].getClosedLoopController();

        }
    }

    public void analogInit() {
        // initialize the analog offset
        for (int i = 0; i < 4; i++) {
            analogs[i].offset = analogs[i].getRotation() * -55;
            steerEncoders[i].setPosition(analogs[i].offset);
        }
    }

    private SwerveModuleState[] PerformKinematics(double TranslateX, double TranslateY, double TranslateRotation) {

        // Swerve Speed variables
        ChassisSpeeds speeds = new ChassisSpeeds(TranslateX, TranslateY, TranslateRotation);

        // Convert to module states
        SwerveModuleState[] moduleStates = Kinematics.toSwerveModuleStates(speeds);

        for (int i = 0; i < 4; i++) {
            double currentAngles = (180 + analogs[i].getDegrees()) % 360 - 180;
            moduleStates[i] = angleMinimize(currentAngles, moduleStates[i], i);
        }

        Kinematics.desaturateWheelSpeeds(moduleStates, MaxDriveSpeed);
        return moduleStates;
    }

    public void applyDrive(double distanceX, double distanceY, double angleRotation) {
        SmartDashboard.putNumber("distanceX", distanceX);
        SmartDashboard.putNumber("distanceY", distanceY);
        SmartDashboard.putNumber("angleRotation", angleRotation);

        SwerveModuleState[] moduleStates = PerformKinematics(distanceX, distanceY, angleRotation);
        // PIDInit();

        for (int i = 0; i < 4; i++) {
            // drive
            double setpoint = moduleStates[i].speedMetersPerSecond * RotationsPerMeter * SecondsPerMinute;
            SmartDashboard.putNumber("Drive " + ModuleOrder.values()[i], setpoint);
            PIDDrive[i]
                    .setReference(setpoint,
                            SparkMax.ControlType.kVelocity,
                            ClosedLoopSlot.kSlot0,0.000001);
            setpoint = moduleStates[i].angle.getRotations() * GearRatio;
            SmartDashboard.putNumber("Steer " + ModuleOrder.values()[i], setpoint);
            PIDSteer[i].setReference(setpoint,
                    SparkMax.ControlType.kPosition,
                    ClosedLoopSlot.kSlot0,0.000001);
        }
    }

    public SwerveModuleState angleMinimize(double CurrentAngle, SwerveModuleState TargetState, int ModuleIndex) {
        double deltaAngle = TargetState.angle.getDegrees() - (CurrentAngle);
        TargetState.angle = new Rotation2d(((CurrentAngle + deltaAngle) % 360) * Math.PI / 180);
        return TargetState;
    }

    /**
     * Example command factory method.
     *
     * @return a command
     */
    public Command exampleMethodCommand() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
                () -> {
                    /* one-time action goes here */
                });
    }

    /**
     * An example method querying a boolean state of the subsystem (for example, a
     * digital sensor).
     *
     * @return value of some boolean subsystem state, such as a digital sensor.
     */
    public boolean exampleCondition() {
        // Query some boolean state, such as a digital sensor.
        return false;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

}
