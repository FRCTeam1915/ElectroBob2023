package frc.robot.commands;



import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class BBUTTON extends CommandBase {
    boolean bbutton;
     static double addSpeed;

   
     
    public BBUTTON(boolean tbbutton) {
       
            
        
        if(bbutton == true){
            addSpeed = .5;
            SmartDashboard.putBoolean("B button", bbutton);
          }
          else{
            addSpeed = 0;
            SmartDashboard.putBoolean("B button", bbutton);
          }
           
            
        
    } 
     public double speed() {
        return addSpeed;
    }

}