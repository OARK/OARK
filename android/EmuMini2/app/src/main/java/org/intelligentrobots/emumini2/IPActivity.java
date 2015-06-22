package org.intelligentrobots.emumini2;

import android.content.Intent;
import android.support.v7.app.ActionBarActivity;
import android.os.Bundle;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;

import org.emumini2.R;


public class IPActivity extends ActionBarActivity {

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_ip);

        final TextView ipTextView = (TextView)findViewById(R.id.ipText);
        Button ipButton = (Button)findViewById(R.id.ipButton);

        ipButton.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                Intent activityStarter = new Intent(getBaseContext(), ControlActivity.class);
                activityStarter.putExtra("IP_ADDRESS", ipTextView.getText().toString());
                startActivity(activityStarter);
            }
        });
    }
}
