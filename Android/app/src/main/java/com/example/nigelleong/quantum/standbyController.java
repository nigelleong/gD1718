package com.example.nigelleong.quantum;

/**
 * Created by nigelleong on 28/4/18.
 */

import android.content.Intent;
import android.os.Bundle;
import android.support.v7.app.AppCompatActivity;
import android.util.Log;


import android.bluetooth.BluetoothSocket;
import android.view.View;
import android.widget.Button;
import android.widget.Toast;
import android.app.ProgressDialog;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.os.AsyncTask;

import java.io.IOException;
import java.util.UUID;

public class standbyController extends AppCompatActivity implements View.OnClickListener {

    Button btnDriving, btnFolding, btnAnalog, btnLayouts, btnPID;

    BluetoothSocket btSocket = null;
    BluetoothSocketHelper bluetoothSocketHelper;

    byte[] buffer = new byte[1024];  // buffer store for the stream
    int bytes; // bytes returned from read()


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_standby);


        bluetoothSocketHelper = ((BluetoothSocketHelper) getApplicationContext());
        btSocket = bluetoothSocketHelper.getBluetoothSocket();
        Log.d("standbyController", btSocket.toString());

<<<<<<< HEAD
        btnDriving = (Button)findViewById(R.id.btn_driving);
        btnFolding = (Button)findViewById(R.id.btn_folding);
        btnAnalog = (Button)findViewById(R.id.btn_analog);
        btnLayouts = (Button)findViewById(R.id.btn_layouts);
        btnPID = (Button)findViewById(R.id.btn_PID);
=======
        btnDriving = (Button) findViewById(R.id.btn_driving);
        btnFolding = (Button) findViewById(R.id.btn_folding);
        btnAnalog = (Button) findViewById(R.id.btn_analog);
        btnLayouts = (Button) findViewById(R.id.btn_layouts);
        btnLayouts = (Button) findViewById(R.id.btn_PID);
>>>>>>> e995c3abdbe4a4fde201c7e117d340e22417ad1c


        btnDriving.setOnClickListener(this);
        btnFolding.setOnClickListener(this);
        btnAnalog.setOnClickListener(this);
        btnLayouts.setOnClickListener(this);
        btnPID.setOnClickListener(this);

        //Switch to STANDBY (state = 0);
        if (btSocket != null) {
            try {
                btSocket.getOutputStream().write("S|0|0|0!".getBytes());
            } catch (IOException e) {
                toastMsg("Error");
            }
        }
        Log.d("STATE", "0");
    }

    @Override
    public void onClick(View view) {
        switch (view.getId()) {
            case R.id.btn_driving:
                Intent drivingIntent = new Intent(standbyController.this, drivingController.class);
                startActivity(drivingIntent);
                break;
            case R.id.btn_folding:
                Intent foldingIntent = new Intent(standbyController.this, foldingController.class);
                startActivity(foldingIntent);
                break;
            case R.id.btn_analog:
                Intent analogIntent = new Intent(standbyController.this, analogController.class);
                startActivity(analogIntent);
                break;
            case R.id.btn_layouts:
                Intent layoutsIntent = new Intent(standbyController.this, layoutController.class);
                startActivity(layoutsIntent);
                break;
            case R.id.btn_PID:
                Intent PIDIntent = new Intent(standbyController.this, PIDController.class);
                startActivity(PIDIntent);
                break;
            default:
                break;
        }
    }

    private void toastMsg(String s) {
        Toast.makeText(getApplicationContext(), s, Toast.LENGTH_LONG).show();
    }
}
