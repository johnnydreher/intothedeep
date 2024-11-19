package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Arm;


public class ArmUp extends CommandBase {
    private final Arm arm;
    public ArmUp(Arm arm) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.arm = arm;
        addRequirements(arm);
    }
    @Override
    public void initialize() {

        arm.setArm(1);
        arm.setElevator(1);
    }
    @Override
    public void execute() {
    }
    @Override
    public void end(boolean interrupted) {


    }
    @Override
    public boolean isFinished() {

        return (arm.isArmUp() && arm.isElevatorUp());
    }
}
