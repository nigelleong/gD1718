package com.example.nigelleong.quantum;

/**
 * Created by nigelleong on 28/4/18.
 */

import android.os.Bundle;
import android.support.v7.app.AppCompatActivity;


import android.bluetooth.BluetoothSocket;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.Toast;

import com.example.nigelleong.quantum.helper.GlobalState;

import java.io.IOException;

public class layoutController extends AppCompatActivity implements View.OnClickListener {

    Button btnPrivacy, btnEfficiency, btnCommunication;

    BluetoothSocket btSocket;
    GlobalState globalState;

    byte[] buffer = new byte[1024];  // buffer store for the stream
    int bytes; // bytes returned from read()


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_layout);

        globalState = ((GlobalState) getApplicationContext());
        btSocket = globalState.getBluetoothSocket();
        Log.d("layoutController",btSocket.toString());

        btnPrivacy = (Button)findViewById(R.id.btn_privacy);
        btnEfficiency = (Button)findViewById(R.id.btn_efficiency);
        btnCommunication = (Button)findViewById(R.id.btn_communication);

        btnPrivacy.setOnClickListener(this);
        btnEfficiency.setOnClickListener(this);
        btnCommunication.setOnClickListener(this);

        //Switch to LAYOUTS (state = 4);
        if (btSocket!=null) {
            try {
                btSocket.getOutputStream().write("S|4|0|0!".getBytes());
            } catch (IOException e) {
                toastMsg("Error");
            }
        }
        Log.d("STATE", "4");
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
            case R.id.btn_privacy:
                request_Privacy();
                break;
            case R.id.btn_efficiency:
                request_Efficiency();
                break;
            case R.id.btn_communication:
                request_Communication();
                break;
            default:
                break;
        }
    }

    private void request_Privacy() {
        if (btSocket!=null) {
            try {
                btSocket.getOutputStream().write("K|0|0|0!".getBytes());
                toastMsg("Command Privacy Mode sent");
            } catch (IOException e) {
                toastMsg("Error");
            }
        }
    }

    private void request_Efficiency() {
        if (btSocket!=null) {
            try {
                btSocket.getOutputStream().write("K|1|0|0!".getBytes());
                toastMsg("Command Efficiency Mode sent");
            } catch (IOException e) {
                toastMsg("Error");
            }
        }
    }

    private void request_Communication() {
        if (btSocket!=null) {
            try {
                btSocket.getOutputStream().write("K|2|0|0!".getBytes());
                toastMsg("Command Communication Mode sent");
            } catch (IOException e) {
                toastMsg("Error");
            }
        }
    }


    private void toastMsg(String s) {
        Toast.makeText(getApplicationContext(),s,Toast.LENGTH_LONG).show();
    }
}
