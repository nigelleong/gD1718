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

public class PIDController extends AppCompatActivity implements View.OnClickListener {

    Button btnGoTo5050;

    BluetoothSocket btSocket;
    BluetoothSocketHelper bluetoothSocketHelper;

    byte[] buffer = new byte[1024];  // buffer store for the stream
    int bytes; // bytes returned from read()


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_pid);

        bluetoothSocketHelper = ((BluetoothSocketHelper) getApplicationContext());
        btSocket = bluetoothSocketHelper.getBluetoothSocket();
        Log.d("PIDController",btSocket.toString());

//        btnStandby = (Button)findViewById(R.id.btn_pid_standby);
//        btnStandby.setOnClickListener(this);

        btnGoTo5050 = (Button)findViewById(R.id.btn_goto5050);
        btnGoTo5050.setOnClickListener(this);

        //Switch to PID position control (state = 5);
        if (btSocket!=null) {
            try {
                btSocket.getOutputStream().write("S|5|0|0!".getBytes());
            } catch (IOException e) {
                toastMsg("Error");
            }
        }
        Log.d("STATE", "5");
    }

    @Override
    protected void onDestroy() {
        if (btSocket!=null) {
            try {
                btSocket.getOutputStream().write("S|0|0|0!".getBytes());
            } catch (IOException e) {
                toastMsg("Error");
            }
        }
        Log.d("STATE", "0");
        super.onDestroy();
    }

    @Override
    public void onClick(View view) {
        switch(view.getId()) {
//            case R.id.btn_pid_standby:
//                Intent drivingIntent = new Intent(PIDController.this, standbyController.class);
//                startActivity(drivingIntent);
//                break;
            case R.id.btn_goto5050:
                moveTo5050();
                break;
            default:
                break;
        }
    }

    private void moveTo5050() {
        if (btSocket!=null) {
            try {
//                btSocket.getOutputStream().write("M|2000|2000|90!".getBytes());
                btSocket.getOutputStream().write("T|500|500|0!".getBytes());
                toastMsg("Command 'Go to Position 50 50' sent");
            } catch (IOException e) {
                toastMsg("Error");
            }
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
