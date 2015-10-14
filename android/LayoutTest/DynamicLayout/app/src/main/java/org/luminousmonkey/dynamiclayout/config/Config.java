package org.luminousmonkey.dynamiclayout.config;

import android.content.Context;
import android.content.res.AssetManager;
import android.util.Log;

import org.yaml.snakeyaml.Yaml;

import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Map;

import org.luminousmonkey.dynamiclayout.config.Motor;

public class Config {

    private final String CONFIG_FILENAME = "config.yaml";

    // YAML config field names.
    private final String CONFIG_INPUTS = "inputs";
    private final String CONFIG_DISPLAY_TEXT = "display_name";
    private final String CONFIG_NAME = "name";

    private Map configMap;

    private ArrayList<Motor> motors;

    public Config(Context context) throws IOException {
        AssetManager am = context.getAssets();
        InputStream is = am.open(CONFIG_FILENAME);

        motors = new ArrayList<Motor>();

        Yaml config = new Yaml();

        configMap  = (Map) config.load(is);

        Log.i("ConfigTest", configMap.toString());

        Map configInputs = (Map) configMap.get(CONFIG_NAME);

        Log.i("ConfigTest", "ConfigInputs: " + configInputs.toString());

    }

    /*
     * Warning, this leaks out the references to the motors in a list
     * and makes them editable by anything with the list.
     */

    public List getMotors() {
        return motors;
    }

    /*
     * Given a motor map entry, return a Motor object that represents
     * the config.
     */
    private Motor getMotorConfig(Map motorMap) {
        Motor newMotor = new Motor();
//
//        newMotor.setUsesAbsolutePosition(
//            ((String) motorMap.get("type")).equals(CONFIG_USES_ABS));
//
//        newMotor.setDisplayText((String) motorMap.get(CONFIG_DISPLAY_TEXT));
//        newMotor.setName((String) motorMap.get(CONFIG_NAME));
//
//        if (motorMap.get(CONFIG_MOTOR) instanceof Map) {
//            Map motorDetails = (Map) motorMap.get(CONFIG_MOTOR);
//            newMotor.setId((int) motorDetails.get(CONFIG_ID));
//
//            // This should really read the current position of the motor.
//            newMotor.setInitPosition((int) motorDetails.get(CONFIG_INIT));
//            newMotor.setMinPosition((int) motorDetails.get(CONFIG_MIN));
//            newMotor.setMaxPosition((int) motorDetails.get(CONFIG_MAX));
//        }

        return newMotor;
    }

}
