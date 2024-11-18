// package frc.robot.commands;

// import frc.robot.Constants.DriveConstants;
// import frc.robot.subsystems.DriveSubsystem;
// import edu.wpi.first.wpilibj2.command.Command;

// import com.ctre.phoenix6.hardware.TalonFX;

// import edu.wpi.first.wpilibj.MotorSafety;
// /**
//  * This command allows the drivetrain to run at a certain
//  * percent output for a certain time.
//  * 
//  * This is only used in autonomous.
//  */
// public class DriveForTimeCmd extends Command {

//     private final DriveSubsystem driveBase;
//     private final double speed;
//     private final double xSpeed;
//     private final double zRotation;
//     private int counter = 0;
//     private int target = 0;

//     public DriveForTimeCmd(DriveSubsystem driveBase, double speed, double seconds, double RightLeft, double Rotation) {
//         this.driveBase = driveBase;
//         this.speed = speed;
//         this.xSpeed = RightLeft;
//         this.zRotation = Rotation;

//         // Convert time in seconds to robot cycles (50 cycles/s)
//         target = (int)(seconds * 50);

//         addRequirements(driveBase);
//     }

    
//     @Override
//     public void execute() {
//         if(counter < target)
//             counter++;
//             //driveBase.feedCheck();
//             System.out.print("GoofyChec1");
//         driveBase.lineardrive(-speed, xSpeed);
//     }

//     @Override
//     public boolean isFinished() {
//         System.out.print("GoofyChec2");
//         return counter >= target;
//     }

//     @Override
//     public void end(boolean interrupted) {
//         System.out.print("GoofyChec3");
//         driveBase.lineardrive(0, 0);
//     }

// }


package frc.robot.commands;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.MotorSafety;
/**
 * This command allows the drivetrain to run at a certain
 * percent output for a certain time.
 * 
 * This is only used in autonomous.
 */
public class DriveForTimeCmd extends Command {

    private final DriveSubsystem driveBase;
    private final double speed;
    private final double xSpeed;
    private final double zRotation;
    private int counter = 0;
    private int target = 0;

    public DriveForTimeCmd(DriveSubsystem driveBase, double speed, double seconds, double RightLeft, double Rotation) {
        this.driveBase = driveBase;
        this.speed = speed;
        this.xSpeed = RightLeft;
        this.zRotation = Rotation;

        // Convert time in seconds to robot cycles (50 cycles/s)
        target = (int)(seconds * 50);

        addRequirements(driveBase);
    }

    
    @Override
    public void execute() {
        if(counter < target)
            counter++;
            //driveBase.feedCheck();
            System.out.print("GoofyChec1");
        driveBase.driveCartesian(-speed, xSpeed, zRotation);
    }

    @Override
    public boolean isFinished() {
        System.out.print("GoofyChec2");
        return counter >= target;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.print("GoofyChec3");
        driveBase.driveCartesian(0, 0, 0);
    }

}
