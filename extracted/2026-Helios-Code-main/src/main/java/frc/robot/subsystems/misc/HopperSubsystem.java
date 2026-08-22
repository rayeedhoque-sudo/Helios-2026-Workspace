package frc.robot.subsystems.misc;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SubsystemConstants.HopperSubsystemConstants;
import frc.robot.Constants.SubsystemConstants.ShooterSubsystemConstants;
import frc.robot.subsystems.utility.Sensors;

public class HopperSubsystem extends SubsystemBase{

    //Hopper Motors
        private final SparkMax hopperMotorA = new SparkMax( HopperSubsystemConstants.HOPPER_ID_A, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
        private final SparkMax hopperMotorB = new SparkMax( HopperSubsystemConstants.HOPPER_ID_B, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    
    //Index Motor
        private final TalonFX kickerMotor = new TalonFX(HopperSubsystemConstants.KICKER_MOTOR_ID);

    // STATE MACHINES
        public enum HOPPERSTATE {
            RUN, STOW
        }
    //Tracker Variables
        private boolean fuelDetectedIndexer;
        private HOPPERSTATE kickState;
        private boolean kickManually;
        private boolean desiredVelReached;
        private boolean desiredAngleReached;
    
    //Data
        private ShuffleboardTab HopperSubsystemTab = Shuffleboard.getTab("Hopper Subsystem Tab");
        private GenericEntry fuelDetectedIndexerEntry;
        private GenericEntry indexStateEntry;
        private GenericEntry desiredVelReachedEntry;
        private GenericEntry desiredAngleReachedEntry;
    
    //Sensors 
        Sensors sensors = new Sensors();
    
    public HopperSubsystem(){
        //Tracker Variables
            fuelDetectedIndexer = false;
            kickState = HOPPERSTATE.STOW;
            desiredVelReached = ShooterSubsystemConstants.desiredVelReached;
            desiredAngleReached = ShooterSubsystemConstants.desiredAngleReached;
    
        //Data
            fuelDetectedIndexerEntry = HopperSubsystemTab.add("Fuel Detected Indexer", false).getEntry();
            indexStateEntry = HopperSubsystemTab.add("Current Index State", kickState.name()).getEntry();
            desiredAngleReachedEntry = HopperSubsystemTab.add("Desired Angle Reached", desiredAngleReached).getEntry();
            desiredVelReachedEntry = HopperSubsystemTab.add("desiredVelocityReached", desiredVelReached).getEntry();
    }

    public void indexFuel(){
        hopperMotorA.set(HopperSubsystemConstants.HOPPER_SPEED);
        hopperMotorB.set(HopperSubsystemConstants.HOPPER_SPEED);
    }

    public void kickFuel(){
        kickerMotor.set(HopperSubsystemConstants.INDEXER_SPEED);
    }
    
    public void stopIndex(){
        hopperMotorA.set(0);
        hopperMotorB.set(0);
    }

    public void stopKickFuel(){
        kickerMotor.set(0);
    }

    public Command setHopperState(HOPPERSTATE state){
        return Commands.runOnce(()->{
            this.kickState = state;
            if(kickState == HOPPERSTATE.STOW){
                kickManually = true;
            }
        });
    }


    @Override
    public void periodic(){
        //Update Tracker Variables
            fuelDetectedIndexer = sensors.getIndexSensorA() && sensors.getIndexSensorB();
            desiredVelReached = ShooterSubsystemConstants.desiredVelReached;
            desiredAngleReached = ShooterSubsystemConstants.desiredAngleReached;
        //Data
            fuelDetectedIndexerEntry.setBoolean(fuelDetectedIndexer);
            indexStateEntry.setString(kickState.name());
            desiredVelReachedEntry.setBoolean(desiredVelReached);
            desiredAngleReachedEntry.setBoolean(desiredAngleReached);

        //Index Based On State Input
        if(kickState == HOPPERSTATE.RUN){
            indexFuel();
            if(desiredVelReached){
                kickFuel();
            } else {
                stopKickFuel();
            }
        } else if (kickState == HOPPERSTATE.STOW){
            stopIndex();
            stopKickFuel();
        }
    }
}
