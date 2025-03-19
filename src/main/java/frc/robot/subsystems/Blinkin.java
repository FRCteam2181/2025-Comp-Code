package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.AddressableLED;
// import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Colors;

public class Blinkin extends SubsystemBase {

    public static Spark blinkin;
    public static Colors.solidColors SOLID_COLORS = new Colors.solidColors();
    public static Colors.fixedPalettePattern FIXED_PALETTE_PATTERN = new Colors.fixedPalettePattern();

    public Blinkin(int channel) {
        blinkin = new Spark(channel);
        
    }
    

    public static Command setBlack() {
        return new InstantCommand(() -> blinkin.set(SOLID_COLORS.black));
    }

    public static Command setRedChase() {
        return new InstantCommand(() -> blinkin.set(FIXED_PALETTE_PATTERN.chase_red));
    }

    public static Command setBlueChase() {
        return new InstantCommand(() -> blinkin.set(FIXED_PALETTE_PATTERN.chase_blue)); 
    }

    public void setDefault() {
        
        var alliance = DriverStation.getAlliance();
        
        if (alliance.get() == DriverStation.Alliance.Red) {
           setRedChase(); 
        } else {
           setBlueChase(); 
        }
    }

    @Override
    public void periodic() {
    }
}

