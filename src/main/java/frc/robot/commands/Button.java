package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

// This will not work because we never initialized the variable button,
// and the variable button is going to be null forever
// So the speed will always be 0

public class Button extends CommandBase {
    boolean button;
    static double speed; // FIXME: Remove the `static` keyword

    // FIXME: Unused variable?
    public Button(boolean tbbutton) {
        // if button == true -> speed set to 0.5 else set to 0
        speed = button ? 0.5 : 0;
        SmartDashboard.putBoolean("B button", button);
    }
     public double getSpeed() {
        return speed;
    }
}