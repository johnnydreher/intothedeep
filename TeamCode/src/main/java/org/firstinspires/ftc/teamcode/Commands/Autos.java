package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.WaitCommand;


import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;

import java.util.ArrayList;
import java.util.List;

public class Autos  {

    public static List<Command> Auto4Pieces(Drivetrain dt, Arm arm, Intake intake) {
        // Use addRequirements() here to declare subsystem dependencies.
        final List<Command> m_commands = new ArrayList<>();
        m_commands.add(new InstantCommand(intake::up));
        m_commands.add(new ArmUp(arm));
        m_commands.add(new InstantCommand(dt::resetEncoders));
        m_commands.add(new DriveToXY(-50,-20 ,dt));
        m_commands.add(new InstantCommand(dt::resetEncoders));
        /*m_commands.add(new DriveTo(-50 ,dt));
        m_commands.add(new InstantCommand(dt::resetEncoders));
        m_commands.add(new DriveTo(20 ,dt));*/
        m_commands.add(new InstantCommand(dt::resetEncoders));
        m_commands.add(new TurnTo(45,dt));
        m_commands.add(new Outtake(intake));
        m_commands.add(new InstantCommand(dt::resetEncoders));
        m_commands.add(new TurnTo(45,dt));
        m_commands.add(new ElevatorDown(arm));
        m_commands.add(new InstantCommand(intake::down));
        m_commands.add(new InstantCommand(intake::push));
        m_commands.add(new InstantCommand(arm::setArmZero));
        m_commands.add(new ElevatorUp(arm));
        m_commands.add(new Grab(intake));
        m_commands.add(new ArmUp(arm));
        m_commands.add(new InstantCommand(dt::resetEncoders));
        m_commands.add(new TurnTo(-45,dt));
        m_commands.add(new Outtake(intake));
        m_commands.add(new InstantCommand(dt::resetEncoders));
        m_commands.add(new TurnTo(30,dt));
        m_commands.add(new ElevatorDown(arm));
        m_commands.add(new InstantCommand(intake::down));
        m_commands.add(new InstantCommand(intake::push));
        m_commands.add(new InstantCommand(arm::setArmZero));
        m_commands.add(new ElevatorUp(arm));
        m_commands.add(new Grab(intake));
        m_commands.add(new ArmUp(arm));
        m_commands.add(new InstantCommand(dt::resetEncoders));
        m_commands.add(new TurnTo(-30,dt));
        m_commands.add(new Outtake(intake));
        m_commands.add(new ArmDown(arm));
        m_commands.add(new InstantCommand(arm::setArmZero,arm));
        m_commands.add(new InstantCommand(arm::setElevatorZero,arm));
        m_commands.add(new WaitCommand(15000));
        return m_commands;
    }

}
