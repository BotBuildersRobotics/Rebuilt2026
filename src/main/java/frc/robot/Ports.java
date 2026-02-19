package frc.robot;

import frc.robot.lib.io.CanDeviceId;

public class Ports {

    //each CAN device will have a name and a unique ID.
   
    public static final CanDeviceId INTAKE = new CanDeviceId(18, "canivore");
    public static final CanDeviceId INTAKE_2 = new CanDeviceId(19, "canivore");
    public static final CanDeviceId TURRET = new CanDeviceId(30, "canivore");
   
    public static final CanDeviceId SHOOTER = new CanDeviceId(21, "canivore");
    public static final CanDeviceId HOOD = new CanDeviceId(33, "canivore");

    public static final CanDeviceId SHUFFLA = new CanDeviceId(16, "canivore");
    public static final CanDeviceId CHUTE = new CanDeviceId(37, "canivore");
   
    public static final CanDeviceId PIVOT = new CanDeviceId(15, "canivore");

    public static final int PIGEON = 13;


}
