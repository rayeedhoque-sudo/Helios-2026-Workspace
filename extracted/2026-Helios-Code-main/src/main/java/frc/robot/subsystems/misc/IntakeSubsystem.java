package frc.robot.subsystems.misc;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SubsystemConstants.IntakeSubsystemConstants;
import frc.robot.subsystems.utility.Sensors;

public class IntakeSubsystem extends SubsystemBase {

    //Type
        boolean enableComp;
        boolean enableTest = false;
    
    //Intake Motor
        private final TalonFX intakeMotor = new TalonFX(IntakeSubsystemConstants.INTAKE_MOTOR_ID);
        private final SparkMax intakeSliderMotor = new SparkMax(IntakeSubsystemConstants.INTAKE_PIVOT_MOTOR_ID, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    
    //State Machine
        public enum IntakeSSTATE{
            INTAKE_STATE, OUTTAKE_STATE, STOW_STATE;
        }
    
    //Tracker Variables
        private IntakeSSTATE currentState;
        private IntakeSSTATE desiredState;

    //Data
        private ShuffleboardTab IntakeSubsystemTab = Shuffleboard.getTab("Intake Subsystem Tab");
        private GenericEntry currentStateEntry;
        private GenericEntry desiredStateEntry;
    
    // //Sensors
        Sensors sensors = new Sensors();

    public IntakeSubsystem(){

        //Initializing Tracker Variables
            currentState = IntakeSSTATE.STOW_STATE;
            desiredState = IntakeSSTATE.STOW_STATE;
        
        // Initializing Shuffleboard Entries
            desiredStateEntry = IntakeSubsystemTab.add("Desired Intake State", desiredState.name()).getEntry();
            currentStateEntry = IntakeSubsystemTab.add("Current Intake State", currentState.name()).getEntry();

    }

    //Subsystem Methods
        public void intake(){
                intakeMotor.set(IntakeSubsystemConstants.INTAKE_SPEED);
        }

        public void outtake(){
                intakeMotor.set(-IntakeSubsystemConstants.OUTTAKE_SPEED);
        }

        public boolean isIntakeSliderStall(){
            return Math.abs(intakeSliderMotor.get()) < IntakeSubsystemConstants.STALL_SPEED && (intakeSliderMotor.get() > 0);
        }

    //Command Based methods

        public Command intakeCommand(){
            return Commands.sequence(
                    Commands.runOnce(()->{
                        intakeSliderMotor.set(-0.5);
                    }),
                    Commands.race(
                        Commands.waitUntil(()->
                            isIntakeSliderStall()
                        ), 
                        Commands.waitSeconds(1)
                    ),
                    Commands.runOnce(()->{
                        intakeSliderMotor.set(0);
                    }),
                    Commands.runOnce(()->{
                        desiredState = IntakeSSTATE.INTAKE_STATE;
                    })
            );
        }

        public Command outtakeCommand(){
            return Commands.sequence(
                    Commands.runOnce(()->{
                        intakeSliderMotor.set(-0.5);
                    }),
                    Commands.race(
                        Commands.waitUntil(()->
                            isIntakeSliderStall()
                        ), 
                        Commands.waitSeconds(1)
                    ),
                    Commands.runOnce(()->{
                        intakeSliderMotor.set(0);
                    }),
                    Commands.runOnce(()->{
                        desiredState = IntakeSSTATE.OUTTAKE_STATE;
                    })
            );
        }

        public Command stowCommand(){
            return Commands.parallel(
                Commands.runOnce(()->{
                    desiredState = IntakeSSTATE.STOW_STATE;
                }),
                Commands.sequence(
                    Commands.runOnce(()->{
                        intakeSliderMotor.set(0.5);
                    }),
                    Commands.race(
                        Commands.waitUntil(()->
                            isIntakeSliderStall()
                        ), 
                        Commands.waitSeconds(1)
                    ),
                    Commands.runOnce(()->{
                        intakeSliderMotor.set(0);
                    })
                )
            );
        }
        public Command stowSliderCommand(){
            return Commands.sequence(
              Commands.runOnce(()->{
                intakeSliderMotor.set(-0.25);
               }),
               Commands.waitUntil(()->
                isIntakeSliderStall()
               ),
               Commands.runOnce(()->{
                intakeSliderMotor.set(0);
               })
            );
        }
        

    @Override
        public void periodic(){
            //STATE MACHINE
                if(desiredState == IntakeSSTATE.INTAKE_STATE){
                    intake();
                }
                else if(desiredState == IntakeSSTATE.OUTTAKE_STATE){
                    outtake();
                } else {
                    intakeMotor.stopMotor();
                }
            //Data
                currentStateEntry.setString(currentState.name());
                desiredStateEntry.setString(desiredState.name());
        }
}
