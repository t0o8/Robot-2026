package frc.robot.commands;

import static edu.wpi.first.units.Units.Volt;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.turret.TurretSubsystem.TurretState;

public class ToggleManualCommand extends Command {
    
    private final DoubleSupplier turretYawSupplier;
    private final DoubleSupplier turretPitchSupplier;

    public ToggleManualCommand(DoubleSupplier turretYawSupplier, DoubleSupplier turretPitchSupplier) {
        this.turretYawSupplier = turretYawSupplier;
        this.turretPitchSupplier = turretPitchSupplier;

        addRequirements(RobotContainer.turretSubsystem);
    }

    @Override
    public void initialize() {
        RobotContainer.turretSubsystem.setOverrideState(TurretState.MANUAL);
    }

    @Override
    public void execute() {
        RobotContainer.turretSubsystem.setOverrideVoltages(
            Volt.of(turretYawSupplier.getAsDouble() * 6),
            Volt.of(turretPitchSupplier.getAsDouble() * 6)
        );
    }

    @Override
    public boolean isFinished() {
        
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.turretSubsystem.setOverrideState(null);

        RobotContainer.turretSubsystem.setOverrideVoltages(Volt.of(0), Volt.of(0));
    }
}
