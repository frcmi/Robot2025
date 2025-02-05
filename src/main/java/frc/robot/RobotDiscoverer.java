package frc.robot;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Enumeration;

import frc.robot.Constants.BotType;
import frc.robot.Constants.RobotDetectionConstants;

public class RobotDiscoverer {
    public static BotType getRobot() {
        if (Robot.isSimulation()) return BotType.SIM_BOT;

        try {
            Enumeration<NetworkInterface> netInterfaces = NetworkInterface.getNetworkInterfaces();
            while (netInterfaces.hasMoreElements()) {
                NetworkInterface netInterface = netInterfaces.nextElement();
                if (netInterface != null) {
                    byte[] macAddress = netInterface.getHardwareAddress();
                    String addressStr = "";
                    for (int i = 0; i < macAddress.length; i++) {
                        addressStr = addressStr.concat(
                            String.format("%02X", macAddress[i])
                            .concat((i < macAddress.length-1) ? ":" : ""));
                    }
                    switch (addressStr) {
                        case RobotDetectionConstants.mainBotMacAddress:
                            return BotType.MAIN_BOT;
                        case RobotDetectionConstants.alphaBotMacAddress:
                            return BotType.ALPHA_BOT;
                    }
                }
            }
        } catch (SocketException e) {
            System.out.println("Error discovering robot type\n" + e.getMessage());
        }


        return BotType.MAIN_BOT;
    }
}
