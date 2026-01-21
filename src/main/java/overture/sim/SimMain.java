package overture.sim;

import static edu.wpi.first.units.Units.Seconds;

import java.util.EnumSet;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.Arena2025Reefscape;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableListener;
import edu.wpi.first.wpilibj.DriverStation;
import overture.sim.robots.DaytonaOffseason;
import overture.sim.robots.Reefscape2025;
import overture.sim.robots.Rebuilt2026;
import overture.sim.robots.SimBaseRobot;

public class SimMain {
	static SimulatedArena arena = null;
	static SimBaseRobot robot = null;

	NetworkTableListener autoLoad = null;

	public void Initialize(LoggedRobot loggedRobot) {
		// Mechanisms
		//robot = new Reefscape2025("Reefscape2025", new Pose2d(3, 3, new Rotation2d()));
		//robot = new DaytonaOffseason("DaytonaOffseason", new Pose2d(3, 3, new Rotation2d()));
		robot = new Rebuilt2026("Rebuilt2026", new Pose2d(3, 3, new Rotation2d()));



		// Arena
		SimulatedArena.overrideInstance(new Arena2025Reefscape());
		SimulatedArena.overrideSimulationTimings(Seconds.of(loggedRobot.getPeriod()), 50);
		arena = SimulatedArena.getInstance();

		arena.resetFieldForAuto();

		// Add mechanisms to Arena
		arena.addDriveTrainSimulation(robot.GetDriveTrain());

		autoLoad = NetworkTableListener.createListener(
				NetworkTableInstance.getDefault().getStructTopic("/PathPlanner/ResetPose", Pose2d.struct),
				EnumSet.of(NetworkTableEvent.Kind.kValueRemote), SimMain::handlePathPlannerPoseReset);
	}

	static public void handlePathPlannerPoseReset(NetworkTableEvent event) {
		try {
			System.out.println("Reset robot pose!");
			Pose2d resetPose = NetworkTableInstance.getDefault().getStructTopic("/PathPlanner/ResetPose", Pose2d.struct)
					.subscribe(new Pose2d()).get();

			Logger.recordOutput("FieldSimulation/AutoPose", resetPose);
			robot.GetDriveTrain().setSimulationWorldPose(resetPose);
			arena.resetFieldForAuto();
			DriverStation.reportWarning("Auto Loaded!", false);

		} catch (Exception e) {
			DriverStation.reportError("Can't update robot Pose2d", false);
			DriverStation.reportError(e.getMessage(), true);
		}
	}

	public void Periodic() {
		arena.simulationPeriodic();
		robot.Update();

		Logger.recordOutput("FieldSimulation/RobotPosition", robot.GetDriveTrain().getSimulatedDriveTrainPose());
		Logger.recordOutput("FieldSimulation/ZeroRobotPosition", new Pose3d());
		Logger.recordOutput("FieldSimulation/FinalComponentPoses", robot.GetMechanismPoses());
		Logger.recordOutput("FieldSimulation/ZeroedComponentPoses", robot.GetZeroedMechanismPoses());

		Logger.recordOutput("FieldSimulation/Algae",
				SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
		Logger.recordOutput("FieldSimulation/Coral",
				SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
	}
}
