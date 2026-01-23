package overture.sim.robots;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;

import java.util.List;

import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import overture.sim.mechanisms.SimMechanism;
import overture.sim.mechanisms.arm.Arm;
import overture.sim.mechanisms.elevator.Elevator;
import overture.sim.mechanisms.flywheel.Flywheel;
import overture.sim.swerve.Constants;
import overture.sim.swerve.SwerveChassis;

public class Rebuilt2026 extends SimBaseRobot {
    SwerveChassis driveTrain;
    Flywheel spindexer, intakeRollers;
    Arm intake;
    Elevator elevator;
    Transform3d originalRobotToSpindexer, originalRobotToIntake, originalRobotToElevator, originalRobotToIntakeRollers;

    List<SimMechanism> mechanisms;

    public Rebuilt2026(String name, Pose2d startingPose) {
        super(name, startingPose);

        // Drivertain
        driveTrain = new SwerveChassis(this, startingPose, Constants.Swerve2024());

        // Spindexer (Flywheel)
        originalRobotToSpindexer = new Transform3d(Meters.of(-0.1), Meters.of(0), Meters.of(0.15), new Rotation3d());
        spindexer = new Flywheel(this,
                new Transform3d(originalRobotToSpindexer.getMeasureX(), originalRobotToSpindexer.getMeasureY(), originalRobotToSpindexer.getMeasureZ(), originalRobotToSpindexer.getRotation()),
                new Rotation3d(0, 0, 1), // Flywheel rotates around this axis
                "spindexer",
                DCMotor.getKrakenX60(1),
                0.33,
                0.01,
                false,
                true);

        // Intake (Arm)
        originalRobotToIntake = new Transform3d(Meters.of(-0.35), Meters.of(0.0), Meters.of(0.3), new Rotation3d());
        intake = new Arm(this,
                new Transform3d(originalRobotToIntake.getMeasureX(), originalRobotToIntake.getMeasureY(), originalRobotToIntake.getMeasureZ(), originalRobotToIntake.getRotation()),
                new Rotation3d(0, 1, 0), // Arm rotations around this axis
                "intake",
                DCMotor.getKrakenX60(1),
                1,
                1.0,
                Meters.of(1),
                Degrees.of(-9999),
                Degrees.of(9999.0),
                Degrees.of(0.0),
                false,
                false);

        // Elevator Carrier (Elevator)
        originalRobotToElevator = new Transform3d(Meters.of(0.33), Meters.of(0.1), Meters.of(0.3), new Rotation3d());
        elevator = new Elevator(this,
                new Transform3d(originalRobotToElevator.getMeasureX(), originalRobotToElevator.getMeasureY(), originalRobotToElevator.getMeasureZ(), originalRobotToElevator.getRotation()),
                new Translation3d(0, 0, 1), // Elevator moves on this axis
                "elevator",
                DCMotor.getKrakenX60(2),
                1,
                Kilograms.of(0.5),
                Meters.of(0.1),
                Meters.of(0.0),
                Meters.of(0.33),
                Meters.of(0.0),
                1,
                false);

        // Intake Rollers (Flywheel)
        originalRobotToIntakeRollers = new Transform3d(Meters.of(0), Meters.of(0.0), Meters.of(0), new Rotation3d());
        intakeRollers = new Flywheel(this,
                new Transform3d(originalRobotToIntakeRollers.getMeasureX(), originalRobotToIntakeRollers.getMeasureY(), originalRobotToIntakeRollers.getMeasureZ(), originalRobotToIntakeRollers.getRotation()),
                new Rotation3d(0, 1, 0), // Flywheel rotates around this axis
                "intakeRollers",
                DCMotor.getKrakenX60(1),
                1,
                0.01,
                false,
                true);

        // List of mechanisms
        mechanisms = List.of(spindexer, intake, elevator, intakeRollers);
    }

    @Override
public void Update() {
    driveTrain.Update();
    mechanisms.forEach(mech -> mech.Update());

    // INTAKE ROLLERS CONTROL WITH INTAKE
    //Copiar los movimientos de translación de X y Z del intake a los rolers, con coordenadas polares
            
        // Update the wheels position based on the arm's position
        double intakeAngle = intake.GetAngle();
        double intakeLength = 0.15; // Assuming this is the arm's length (r)

        // Convert polar to rectangular
        double wheelsX = intakeLength * Math.sin(intakeAngle); // x = r * cos(θ)
        double wheelsZ = intakeLength * Math.cos(intakeAngle); // y = r * sin(θ)
        // Update the wheels rotation based on the arm's rotator
        double intakeAngleX = intake.GetPoses3d().get(0).getRotation().getX(); // Angle in radians
        double intakeAngleY = intake.GetPoses3d().get(0).getRotation().getY(); // Angle in radians
        double intakeAngleZ = intake.GetPoses3d().get(0).getRotation().getZ(); // Angle in radians
    
        // Create new Pose3d for wheels
        Pose3d armRotatorPoseWheels = new Pose3d(
            new Translation3d(wheelsX - 0.25, 0, wheelsZ + 0.3), 
            new Rotation3d(intakeAngleX, intakeAngleY, intakeAngleZ)
        );

        // Update the wheels position
        intakeRollers.SetRobotToMechanism(
            originalRobotToIntakeRollers.plus(new Transform3d(armRotatorPoseWheels.getTranslation(), armRotatorPoseWheels.getRotation())));
    


}

    @Override
    public AbstractDriveTrainSimulation GetDriveTrain() {
        return driveTrain;
    }

    @Override
    public List<SimMechanism> GetMechanisms() {
        return mechanisms;
    }
}