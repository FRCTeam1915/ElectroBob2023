package frc.robot.commands;



import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class BBUTTON extends CommandBase {
    boolean bbutton;
     static double addSpeed;

   
     //creates new bbbbbutton
    public BBUTTON(boolean tbbutton) {
       
            
        //if bbbbbutton is true sets speed to 0.5 otherwise set to 0
        if(bbutton == true){
            addSpeed = .5;
            SmartDashboard.putBoolean("B button", bbutton);
          }
          else{
            addSpeed = 0;
            SmartDashboard.putBoolean("B button", bbutton);
          }
           
            
        //creates class that just returns speed
    } 
     public double speed() {
        return addSpeed;
    }

}