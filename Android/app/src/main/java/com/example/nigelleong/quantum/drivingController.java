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
import android.app.ProgressDialog;
import android.bluetooth.BluetoothAdapter;

import java.io.IOException;
import java.util.UUID;

public class drivingController extends AppCompatActivity implements View.OnClickListener {

    Button btnUp, btnDown, btnLeft, btnRight, btnPeak, btnOffPeak, btnMove;

    BluetoothSocket btSocket;
    BluetoothSocketHelper bluetoothSocketHelper;

    byte[] buffer = new byte[1024];  // buffer store for the stream
    int bytes; // bytes returned from read()


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_driving);

        bluetoothSocketHelper = ((BluetoothSocketHelper) getApplicationContext());
        btSocket = bluetoothSocketHelper.getBluetoothSocket();
        Log.d("drivingController",btSocket.toString());

        btnUp = (Button)findViewById(R.id.btn_up);
        btnDown = (Button)findViewById(R.id.btn_down);
        btnLeft = (Button)findViewById(R.id.btn_left);
        btnRight = (Button)findViewById(R.id.btn_right);
        btnPeak = (Button)findViewById(R.id.btn_peak);
        btnOffPeak = (Button)findViewById(R.id.btn_offpeak);
        btnMove = (Button)findViewById(R.id.btn_move);

        btnUp.setOnClickListener(this);
        btnDown.setOnClickListener(this);
        btnLeft.setOnClickListener(this);
        btnRight.setOnClickListener(this);
        btnPeak.setOnClickListener(this);
        btnOffPeak.setOnClickListener(this);
        btnMove.setOnClickListener(this);
    }

    @Override
    public void onClick(View view) {
        switch(view.getId()) {
            case R.id.btn_up:
                robotGoUp();
                break;
            case R.id.btn_down:
                robotGoDown();
                break;
            case R.id.btn_left:
                robotGoLeft();
                break;
            case R.id.btn_right:
                robotGoRight();
                break;
            case R.id.btn_peak:
                configPeak();
                break;
            case R.id.btn_offpeak:
                configOffPeak();
                break;
            case R.id.btn_move:
                move();
                break;
            default:
                break;
        }
    }

    private void move() {
        if (btSocket!=null) {
            try {
//                btSocket.getOutputStream().write("M|2000|2000|90!".getBytes());
                btSocket.getOutputStream().write("M|2000|2000|270!".getBytes());
            } catch (IOException e) {
                toastMsg("Error");
            }
        }
    }

    private void robotGoUp() {
        if (btSocket!=null) {
            try {
//                btSocket.getOutputStream().write("M|2000|2000|90!".getBytes());
                btSocket.getOutputStream().write("M|2000|2000|270!".getBytes());
            } catch (IOException e) {
                toastMsg("Error");
            }
        }
    }

    private void robotGoDown() {
        if (btSocket!=null) {
            try {
                btSocket.getOutputStream().write("M|2000|2000|270!".getBytes());
                toastMsg("Command 'down' sent");
            } catch (IOException e) {
                toastMsg("Error");
            }
        }
    }

    private void robotGoLeft() {
        if (btSocket!=null) {
            try {
                btSocket.getOutputStream().write("M|2000|2000|180!".getBytes());
                toastMsg("Command 'left' sent");
            } catch (IOException e) {
                toastMsg("Error");
            }
        }
    }

    private void robotGoRight() {
        if (btSocket!=null) {
            try {
                btSocket.getOutputStream().write("M|2000|2000|0!".getBytes());
                toastMsg("Command 'right' sent");
            } catch (IOException e) {
                toastMsg("Error");
            }
        }
    }

    private void configPeak() {
        if (btSocket!=null) {
            try {
                btSocket.getOutputStream().write("W|200|200|90!".getBytes());
                bytes = btSocket.getInputStream().read(buffer);
                btSocket.getOutputStream().write("W|200|200|180!".getBytes());
                bytes = btSocket.getInputStream().read(buffer);
                btSocket.getOutputStream().write("W|200|200|270!".getBytes());
                bytes = btSocket.getInputStream().read(buffer);
                btSocket.getOutputStream().write("W|200|200|0!".getBytes());
                toastMsg("Config Peak completed");
            } catch (IOException e) {
                toastMsg("Error");
            }
        }
    }

    private void configOffPeak() {
        if (btSocket!=null) {
            try {
                btSocket.getOutputStream().write("W|200|200|90!".getBytes());
                bytes = btSocket.getInputStream().read(buffer);
                btSocket.getOutputStream().write("W|200|200|180!".getBytes());
                bytes = btSocket.getInputStream().read(buffer);
                btSocket.getOutputStream().write("W|200|200|270!".getBytes());
                bytes = btSocket.getInputStream().read(buffer);
                btSocket.getOutputStream().write("W|200|200|0!".getBytes());
                toastMsg("Config Peak completed");
            } catch (IOException e) {
                toastMsg("Error");
            }
        }
    }

    private void toastMsg(String s) {
        Toast.makeText(getApplicationContext(),s,Toast.LENGTH_LONG).show();
    }
}
