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

public class PIDController extends AppCompatActivity implements View.OnClickListener {

    Button btnGoTo505090, btnGoTo100125180, btnGoTo2510090, btnGoTo000;

    BluetoothSocket btSocket;
    GlobalState globalState;

    byte[] buffer = new byte[1024];  // buffer store for the stream
    int bytes; // bytes returned from read()


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_pid);

        globalState = ((GlobalState) getApplicationContext());
        btSocket = globalState.getBluetoothSocket();
        Log.d("PIDController",btSocket.toString());

//        btnStandby = (Button)findViewById(R.id.btn_pid_standby);
//        btnStandby.setOnClickListener(this);

        btnGoTo505090 = (Button)findViewById(R.id.btn_goto505090);
        btnGoTo100125180 = (Button)findViewById(R.id.btn_goto_100125180);
        btnGoTo2510090 = (Button)findViewById(R.id.btn_goto_2510090);
        btnGoTo000 = (Button)findViewById(R.id.btn_goto000);

        btnGoTo505090.setOnClickListener(this);
        btnGoTo100125180.setOnClickListener(this);
        btnGoTo2510090.setOnClickListener(this);
        btnGoTo000 .setOnClickListener(this);


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
            case R.id.btn_goto505090:
                moveTo505090();
                break;
            case R.id.btn_goto_100125180:
                moveTo100125180();
                break;
            case R.id.btn_goto_2510090:
                moveTo2510090();
                break;
            case R.id.btn_goto000:
                moveTo000();
                break;
            default:
                break;
        }
    }

    private void moveTo505090() {
        if (btSocket!=null) {
            try {
                btSocket.getOutputStream().write("T|500|500|90!".getBytes());
                toastMsg("Command 'Go to Position 50 50 90' sent");
            } catch (IOException e) {
                toastMsg("Error");
            }
        }
    }

    private void moveTo100125180() {
        if (btSocket!=null) {
            try {
                btSocket.getOutputStream().write("T|1000|1250|180!".getBytes());
                toastMsg("Command 'Go to Position 100 125 180' sent");
            } catch (IOException e) {
                toastMsg("Error");
            }
        }
    }

    private void moveTo2510090() {
        if (btSocket!=null) {
            try {
                btSocket.getOutputStream().write("T|250|1000|-90!".getBytes());
                toastMsg("Command 'Go to Position 25 100 -90' sent");
            } catch (IOException e) {
                toastMsg("Error");
            }
        }
    }

    private void moveTo000() {
        if (btSocket!=null) {
            try {
                btSocket.getOutputStream().write("T|0|0|0!".getBytes());
                toastMsg("Command 'Go to Position 0 0 0' sent");
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
