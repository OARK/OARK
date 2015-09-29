package org.luminousmonkey.dynamiclayout.config;

import android.content.Context;
import android.content.res.AssetManager;
import android.util.Log;

import org.yaml.snakeyaml.Yaml;

import java.io.IOException;
import java.io.InputStream;
import java.util.Iterator;
import java.util.Map;

public class Config {

    private final String configFilename = "config.yaml";
    private Map configMap;

    public Config(Context context) throws IOException {
        AssetManager am = context.getAssets();
        InputStream is = am.open(configFilename);

        Yaml config = new Yaml();

        configMap  = (Map) config.load(is);

        Log.i("ConfigTest", configMap.toString());

        Iterator it = configMap.entrySet().iterator();

        while (it.hasNext()) {
            Map.Entry pair = (Map.Entry) it.next();
            Log.i("ConfigTest", "Key: " + pair.getKey());
            Log.i("ConfigTest", "Value: " + pair.getValue());
        }
    }

    /*
     * Return a map of only the wheel types.
     */
    public Map getWheelTypes() {
        return null;
    }
}
