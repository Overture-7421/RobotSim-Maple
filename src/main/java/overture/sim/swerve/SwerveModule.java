package overture.sim.swerve;

import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import overture.sim.NTCANCoder;
import overture.sim.NTMotor;

public class SwerveModule {
    NTMotor driveNtMotor;
    NTMotor steerNtMotor;
    NTCANCoder ntCANCoder;

    public String robotName = null;
    public String moduleName = null;
    public SwerveModuleSimulation moduleSimulation = null;

    public SwerveModule(String robotName, String moduleName, SwerveModuleSimulation moduleSimulation) {
        this.robotName = robotName;
        this.moduleName = moduleName;
        this.moduleSimulation = moduleSimulation;

        var driveMotor = moduleSimulation.useGenericMotorControllerForDrive();
        driveNtMotor = new NTMotor(new NTMotor.Config() {
            {
                Name = robotName + "/motors/" + moduleName + "_drive";
                VoltageApplied = (voltage) -> driveMotor.requestVoltage(voltage);
                EncoderPosition = () -> moduleSimulation.getDriveEncoderUnGearedPosition();
                EncoderSpeed = () -> moduleSimulation.getDriveEncoderUnGearedSpeed();
                Current = () -> moduleSimulation.getDriveMotorSupplyCurrent();
                Inverted = false;
            }
        });

        var steerMotor = moduleSimulation.useGenericControllerForSteer();
        steerNtMotor = new NTMotor(new NTMotor.Config() {
            {
                Name = robotName + "/motors/" + moduleName + "_rotation";
                VoltageApplied = (voltage) -> steerMotor.requestVoltage(voltage.times(0.85));
                EncoderPosition = () -> moduleSimulation.getSteerRelativeEncoderPosition();
                EncoderSpeed = () -> moduleSimulation.getSteerRelativeEncoderVelocity();
                Current = () -> moduleSimulation.getSteerMotorSupplyCurrent();
                Inverted = false;
            }
        });

        ntCANCoder = new NTCANCoder(new NTCANCoder.Config() {
            {
                Name = robotName + "/cancoders/" + moduleName + "_cancoder";
                EncoderPosition =
                        () -> moduleSimulation.getSteerAbsoluteFacing().getMeasure();
                EncoderSpeed = () -> moduleSimulation.getSteerAbsoluteEncoderSpeed();
                Inverted = false;
            }
        });
    }

    public void Update() {
        driveNtMotor.Update();
        steerNtMotor.Update();
        ntCANCoder.Update();
    }
}
