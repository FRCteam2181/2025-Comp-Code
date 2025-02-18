package frc.robot.systems;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ButtonBoard {
    
    public Trigger button0;
    public Trigger button1;
    public Trigger button2;
    public Trigger button3;
    public Trigger button4;
    public Trigger button5;
    public Trigger button6;
    public Trigger button7;
    public Trigger button8;
    public Trigger button9;
    public Trigger button10;
    public Trigger button11;
    public Trigger button12;
    public Trigger button13;
    public Trigger button14;
    public Trigger button15;
    public Trigger button16;
    public Trigger button17;

    private GenericHID buttonBoard;

    public ButtonBoard(int port) {
        
        buttonBoard = new GenericHID(port);

        button0 = new Trigger(buttonBoard.button(0, null));
        button1 = new Trigger(buttonBoard.button(1, null));
        button2 = new Trigger(buttonBoard.button(2, null));
        button3 = new Trigger(buttonBoard.button(3, null));
        button4 = new Trigger(buttonBoard.button(4, null));
        button5 = new Trigger(buttonBoard.button(5, null));
        button6 = new Trigger(buttonBoard.button(6, null));
        button7 = new Trigger(buttonBoard.button(7, null));
        button8 = new Trigger(buttonBoard.button(8, null));
        button9 = new Trigger(buttonBoard.button(9, null));
        button10 = new Trigger(buttonBoard.button(10, null));
        button11 = new Trigger(buttonBoard.button(11, null));
        button12 = new Trigger(buttonBoard.button(12, null));
        button13 = new Trigger(buttonBoard.button(13, null));
        button14 = new Trigger(buttonBoard.button(14, null));
        button15 = new Trigger(buttonBoard.button(15, null));
        button16 = new Trigger(buttonBoard.button(16, null));
        button17 = new Trigger(buttonBoard.button(17, null));
    }
}
