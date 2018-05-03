package com.example.nigelleong.quantum;

import android.bluetooth.BluetoothSocket;
import android.os.Bundle;
import android.support.v7.app.AppCompatActivity;
import android.util.Log;
import android.view.View;
import android.widget.Button;

/**
 * Created by nigelleong on 2/5/18.
 */

public class analogController extends AppCompatActivity implements View.OnClickListener{


    BluetoothSocket btSocket = null;
    BluetoothSocketHelper bluetoothSocketHelper;
//    public static String EXTRA_BT_SOCKET = "bluetooth_socket";

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_analog);

        bluetoothSocketHelper = ((BluetoothSocketHelper) getApplicationContext());
        btSocket = bluetoothSocketHelper.getBluetoothSocket();
        Log.d("analogController",btSocket.toString());

    }

    @Override
    public void onClick(View view) {
        switch(view.getId()) {
            default:
                break;
        }
    }
}
