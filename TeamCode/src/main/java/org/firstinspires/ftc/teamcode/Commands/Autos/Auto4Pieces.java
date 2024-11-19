package org.firstinspires.ftc.teamcode.Commands.Autos;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Commands.ArmUp;
import org.firstinspires.ftc.teamcode.Commands.Grab;
import org.firstinspires.ftc.teamcode.Commands.Outtake;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;

public class Auto4Pieces extends SequentialCommandGroup {
    private final Drivetrain dt;
    private final Arm arm;
    private final Intake intake;

    public Auto4Pieces(Drivetrain dt, Arm arm, Intake intake) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.dt = dt;
        this.arm = arm;
        this.intake = intake;
        addCommands( new Grab(intake),
                    new ArmUp(arm),
                    new Outtake(intake),
                    new ArmUp(arm) );
        addRequirements(dt, arm, intake);
    }

}
