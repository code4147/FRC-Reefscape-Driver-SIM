//Isaac Is Cool

package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class ElevatorConstants {
    public record ElevatorGains(
            double kP,
            double kI,
            double kD,
            double kMaxVelocityMetersPerSecond,
            double kMaxAccelerationMetersPerSecondSquared,
            double kJerkMetersPerSecondCubed,
            double kS,
            double kV,
            double kA,
            double kG) {}

    public record KrakenConfiguration(
            boolean kInvert,
            boolean kEnableStatorCurrentLimit,
            boolean kEnableSupplyCurrentLimit,
            double kStatorCurrentLimitAmps,
            double kSupplyCurrentLimitAmps,
            NeutralModeValue kNeutralMode) {}

    public record SimulationConfiguration(
            DCMotor kMotorType,
            double kCarriageMassKg,
            double kDrumRadiusMeters,
            boolean kSimulateGravity,
            double kStartingHeightMeters,
            double kMeasurementStdDevs) {}

    public static final int kMotorID = 50;

    public static final double kGearing = 9.0 / 1.0;
    public static final double kDrumCircumferenceMeters = 2.0 * Math.PI * Units.inchesToMeters(0.819);


    public static final double kMaxPositionMeters = 1.5494;
    public static final double kMinPositionMeters = 0.0;

    public static final double kStatusSignalUpdateFrequencyHz = 100.0;

    public static final ElevatorGains kElevatorGains =
            switch (Constants.currentMode) {
                case REAL -> new ElevatorGains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
                case SIM -> new ElevatorGains(1, 0.0, 0.0, 25.0, 25.0, 0.0, 0.0, 25.07, 0, 0.28);
                default -> new ElevatorGains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
            };

    public static final KrakenConfiguration kMotorConfiguration =
            new KrakenConfiguration(true, true, true, 60.0, 50.0, NeutralModeValue.Brake);

    public static final SimulationConfiguration kSimulationConfiguration =
            new SimulationConfiguration(DCMotor.getKrakenX60(1), 1.0, Units.inchesToMeters(0.994), true, 0.0, 0.0002);
}
