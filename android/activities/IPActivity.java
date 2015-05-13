package sep_17.fourwwcontrol;

import android.content.Intent;
import android.support.v7.app.ActionBarActivity;
import android.os.Bundle;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;


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
                activityStarter.putExtra("tim.is.a.legend.IP_ADDRESS", ipTextView.getText().toString());
                startActivity(activityStarter);
            }
        });
    }


    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        // Inflate the menu; this adds items to the action bar if it is present.
        getMenuInflater().inflate(R.menu.menu_i, menu);
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        // Handle action bar item clicks here. The action bar will
        // automatically handle clicks on the Home/Up button, so long
        // as you specify a parent activity in AndroidManifest.xml.
        int id = item.getItemId();

        //noinspection SimplifiableIfStatement
        if (id == R.id.action_settings) {
            return true;
        }

        return super.onOptionsItemSelected(item);
    }
}
