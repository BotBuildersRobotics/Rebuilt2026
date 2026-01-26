package frc.robot;

import frc.robot.lib.io.CanDeviceId;

public class Ports {

    //each CAN device will have a name and a unique ID.
   
    public static final CanDeviceId INTAKE = new CanDeviceId(19, "canivore");
    public static final CanDeviceId TURRET = new CanDeviceId(31, "canivore");
   
    public static final CanDeviceId SHOOTER = new CanDeviceId(18, "canivore");
    public static final CanDeviceId HOOD = new CanDeviceId(33, "canivore");

    public static final CanDeviceId SHUFFLA = new CanDeviceId(36, "canivore");
    public static final CanDeviceId CHUTE = new CanDeviceId(37, "canivore");
   

    public static final int PIGEON = 13;


}
