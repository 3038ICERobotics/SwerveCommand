package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase {
    SparkMax[] steerMotors = new SparkMax[4];
    SparkMax[] driveMotors = new SparkMax[4];

    /** Creates a new ExampleSubsystem. */
    public SwerveDrive() {
        int index = 0;
        driveMotors[index++]= new SparkMax(10, MotorType.kBrushless);//FL
        driveMotors[index++]= new SparkMax(10, MotorType.kBrushless);//BL
        driveMotors[index++]= new SparkMax(10, MotorType.kBrushless);//FR
        driveMotors[index++]= new SparkMax(10, MotorType.kBrushless);//BR
        index=0;
        steerMotors[index++]= new SparkMax(10, MotorType.kBrushless);//FL
        steerMotors[index++]= new SparkMax(10, MotorType.kBrushless);//BL
        steerMotors[index++]= new SparkMax(10, MotorType.kBrushless);//FR
        steerMotors[index++]= new SparkMax(10, MotorType.kBrushless);//BR
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
