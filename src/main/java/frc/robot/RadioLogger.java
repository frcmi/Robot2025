package frc.robot;

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.net.HttpURLConnection;
import java.net.URI;
import java.net.URL;

import static edu.wpi.first.units.Units.Millisecond;
import static edu.wpi.first.units.Units.Seconds;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotController.RadioLEDState;

import frc.lib.ultralogger.UltraBooleanLog;
import frc.lib.ultralogger.UltraJSONLog;

class RadioLogger {
    private static final Time requestPeriodSecs = Seconds.of(5.0);
    private static final Time timeout = Seconds.of(0.5);

    private URL statusURL;
    private Notifier notifier;
    private final UltraJSONLog statusJsonPublisher = new UltraJSONLog("Radio/Status");
    private final UltraBooleanLog isConnectedPublisher = new UltraBooleanLog("Radio/Connected");

    private void periodic() {
        StringBuilder response = new StringBuilder();
        try {
            HttpURLConnection connection = (HttpURLConnection) statusURL.openConnection();
            connection.setRequestMethod("GET");
            connection.setConnectTimeout((int) timeout.in(Millisecond));
            connection.setReadTimeout((int) timeout.in(Millisecond));

            try (BufferedReader reader = new BufferedReader(new InputStreamReader(connection.getInputStream()))) {
                for (String line; (line = reader.readLine()) != null;) {
                    response.append(line);
                }
            }
        } catch (Exception e) {
            System.err.println(e);
        }

        String responseStr = response.toString().replaceAll("\\s+", "");
        boolean isConnected = !responseStr.isEmpty();

        isConnectedPublisher.update(isConnected);
        if (isConnected) {
            RobotController.setRadioLEDState(RadioLEDState.kGreen);
            statusJsonPublisher.update(responseStr);
        } else {
            RobotController.setRadioLEDState(RadioLEDState.kRed);
            statusJsonPublisher.update("{}");
        }
    }

    public RadioLogger() {
        if (RobotBase.isReal()) {
            // TODO: configure dynamically or make alert when team number != 5937
            try {
                statusURL = new URI("http://10.59.37.1/status").toURL();
            } catch (Exception e) {
                System.err.println(e);
                return;
            }

            notifier = new Notifier(this::periodic);
            notifier.setName("Radio Logger");
            notifier.startPeriodic(requestPeriodSecs.in(Seconds));
        }
    }
}