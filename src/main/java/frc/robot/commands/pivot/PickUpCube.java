// package frc.robot.commands.pivot;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.subsystems.pivot.Intake;
// import frc.robot.subsystems.pivot.Pivot;

// public class PickUpCube extends CommandBase{
//     private Intake intakeSubsystem;
//     private Pivot pivotSubsystem;

//     public PickUpCube(Intake intakeSubsystem, Pivot pivotSubsystem) {
//         this.intakeSubsystem = intakeSubsystem;
//         this.pivotSubsystem = pivotSubsystem;
//     }
//     @Override
//     public void initialize() {
//         SequentialCommandGroup group = new SequentialCommandGroup(
//             new ParallelRaceGroup(new RollIntake(intakeSubsystem), new PivotToCube(pivotSubsystem)),
//             new Turtle(pivotSubsystem)
//         );
//         group.schedule();
//     }
//     @Override
//     public boolean isFinished() {
//         return true;
//     }
// }
