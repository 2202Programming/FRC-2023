package frc.robot.subsystems.util;

import java.util.function.Consumer;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


public class WebCommands {

  private NetworkTable table;
  
  public WebCommands() {
  
    table = NetworkTableInstance.getDefault().getTable("Commands");



  }

  void  ListenerBoolean(String entryName, boolean init_value, Consumer<Boolean> cmd) {
    //create entry in our table and set initial value to false
    NetworkTableEntry nte = table.getEntry(entryName);
    nte.setBoolean(init_value);

    // now construct the command listener, lambda called on value changes 
    table.addEntryListener(entryName, (table, key, entry, value, flags)  -> 
      {
        boolean b = value.getBoolean();
        cmd.accept(b);
        System.out.println("***Web Boolean " + entryName + " = " + b);
      }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
  }




  void  ListenerCmdOnTrue(String entryName, Command cmd) {
    //create entry in our table and set initial value to false
    NetworkTableEntry nte = table.getEntry(entryName);
    nte.setBoolean(false);

    // now construct the command listener, lambda called on value changes 
    table.addEntryListener(entryName, (table, key, entry, value, flags)  -> 
      {
        if (value.getBoolean()) {
          System.out.println("***Web Command - Executing " + entryName);
          CommandScheduler.getInstance().schedule(cmd);
          // just ack the scheduling, in perfect world the cmd would handle this 
          // by taking an NTE or entry string
         nte.setBoolean(false);    
        }
      }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
  }

}