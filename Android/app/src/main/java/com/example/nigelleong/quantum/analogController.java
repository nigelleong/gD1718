package com.example.nigelleong.quantum;

/**
 * Created by nigelleong on 28/4/18.
 */

import android.content.Intent;
import android.os.Bundle;
import android.support.v7.app.AppCompatActivity;


import android.bluetooth.BluetoothSocket;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.Toast;

import java.io.IOException;

public class analogController extends AppCompatActivity implements View.OnClickListener {

    Button btnStandby;

    BluetoothSocket btSocket;
    BluetoothSocketHelper bluetoothSocketHelper;

    byte[] buffer = new byte[1024];  // buffer store for the stream
    int bytes; // bytes returned from read()


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_analog);

        bluetoothSocketHelper = ((BluetoothSocketHelper) getApplicationContext());
        btSocket = bluetoothSocketHelper.getBluetoothSocket();
        Log.d("analogController",btSocket.toString());

        btnStandby = (Button)findViewById(R.id.btn_analog_standby);


        btnStandby.setOnClickListener(this);


        //Switch to Analog remote control (state = 3);
        if (btSocket!=null) {
            try {
                btSocket.getOutputStream().write("S|3|0|0!".getBytes());
            } catch (IOException e) {
                toastMsg("Error");
            }
        }
    }

    @Override
    public void onClick(View view) {
        switch(view.getId()) {
            case R.id.btn_analog_standby:
                Intent drivingIntent = new Intent(analogController.this, standbyController.class);
                startActivity(drivingIntent);
                break;
            default:
                break;
        }
    }

    /* Exemple:
    private void setToOdoIMUNFC() {
        if (btSocket!=null) {
            try {
                btSocket.getOutputStream().write("L|2|0|0!".getBytes());
                toastMsg("Localization method: Odometry + IMU + NFC");
            } catch (IOException e) {
                toastMsg("Error");
            }
        }
    }*/


    private void toastMsg(String s) {
        Toast.makeText(getApplicationContext(),s,Toast.LENGTH_LONG).show();
    }
}
