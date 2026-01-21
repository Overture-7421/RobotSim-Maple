package overture.sim.swerve;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;

public class Constants {
    private static SwerveModuleSimulationConfig Overture2024MK4i() {
        return new SwerveModuleSimulationConfig(
                DCMotor.getKrakenX60(1),
                DCMotor.getFalcon500(1),
                5.9027777,
                150.0 / 7.0,
                Volts.of(0.01),
                Volts.of(0.01),
                Inches.of(2),
                KilogramSquareMeters.of(0.03),
                // Use a conservative tire coefficient of friction to avoid bounding-check errors
                // The COTS value was flagged as abnormal by the simulation (e.g., ~2.106)
                // Set to a typical realistic value (~0.9) for rubber on carpet/asphalt.
                0.9);
    }

    public static DriveTrainSimulationConfig Swerve2024() {
        return DriveTrainSimulationConfig.Default()
                .withRobotMass(Kilograms.of(61.235))
                .withGyro(COTS.ofPigeon2())
                .withSwerveModule(Overture2024MK4i())
                .withCustomModuleTranslations(new Translation2d[] {
                    new Translation2d(Inches.of(9.144061), Inches.of(9.880374)), // Front Left
                    new Translation2d(Inches.of(9.144061), Inches.of(-9.880374)), // Front Right
                    new Translation2d(Inches.of(-9.144061), Inches.of(9.880374)), // Back Left
                    new Translation2d(Inches.of(-9.144061), Inches.of(-9.880374)) // Back Right
                })
                .withBumperSize(Inches.of(30.5), Inches.of(31.5));

    }
}
