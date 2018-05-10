package com.example.nigelleong.quantum;

/**
 * Created by nigelleong on 28/4/18.
 */

import android.os.Bundle;
import android.support.v7.app.AppCompatActivity;


import android.bluetooth.BluetoothSocket;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;
import android.os.SystemClock;

import java.io.IOException;

import io.github.controlwear.virtual.joystick.android.JoystickView;

public class analogController extends AppCompatActivity implements View.OnClickListener {

    Button btnOdo, btnOdoIMU;

    BluetoothSocket btSocket;
    BluetoothSocketHelper bluetoothSocketHelper;
    SystemClock time;

    byte[] buffer = new byte[1024];  // buffer store for the stream
    int bytes; // bytes returned from read()

    private TextView txtXspeed;
    private TextView txtYspeed;
    private TextView txtRotation;

    private double X_Speed, Y_Speed, Rot_Speed;
    private long time_prev;

    private boolean updateBrake;


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_analog);

        bluetoothSocketHelper = ((BluetoothSocketHelper) getApplicationContext());
        btSocket = bluetoothSocketHelper.getBluetoothSocket();
        Log.d("analogController",btSocket.toString());

        btnOdo = (Button)findViewById(R.id.btn_odo);
        btnOdoIMU = (Button)findViewById(R.id.btn_odo_imu);

        btnOdo.setOnClickListener(this);
        btnOdoIMU.setOnClickListener(this);

        txtXspeed = (TextView) findViewById(R.id.txt_x_speed);
        txtYspeed = (TextView) findViewById(R.id.txt_y_speed);
        txtRotation = (TextView) findViewById(R.id.txt_rotation);
        X_Speed = 0;
        Y_Speed = 0;
        Rot_Speed = 0;

        time_prev = time.uptimeMillis();

        updateBrake = false;

        JoystickView joystickLeft = (JoystickView) findViewById(R.id.joystick_left);
        joystickLeft.setOnMoveListener(new JoystickView.OnMoveListener() {
            @Override
            public void onMove(int angle, int strength) {
                X_Speed = strength*Math.sin(angle*Math.PI/180);
                Y_Speed = -strength*Math.cos(angle*Math.PI/180);
                txtXspeed.setText(Math.round(X_Speed) + "%");
                txtYspeed.setText(Math.round(Y_Speed) + "%");
                if (X_Speed == 0 && Y_Speed == 0) {
                    updateBrake = true;
                    toastMsg("Stopped");
                }
                send_Speeds();
            }
        });

        JoystickView joystickRight = (JoystickView) findViewById(R.id.joystick_right);
        joystickRight.setOnMoveListener(new JoystickView.OnMoveListener() {
            @Override
            public void onMove(int angle, int strength) {
                Rot_Speed = -strength*Math.cos(angle*Math.PI/180);
                txtRotation.setText(Math.round(Rot_Speed) + "%");
                if (Rot_Speed == 0) {
                    updateBrake = true;
                    toastMsg("Stopped");
                }
                send_Speeds();
            }
        });


        //Switch to Analog remote control (state = 3);
        if (btSocket!=null) {
            try {
                btSocket.getOutputStream().write("S|3|0|0!".getBytes());
            } catch (IOException e) {
                toastMsg("Error");
            }
        }
        Log.d("STATE", "3");
    }

    public void onClick(View view) {
        switch(view.getId()) {
            case R.id.btn_odo:
                setToOdo();
                break;
            case R.id.btn_odo_imu:
                setToOdoIMU();
                break;
            default:
                break;
        }
    }
    private void send_Speeds() {
        if((time.uptimeMillis()-time_prev >50) || updateBrake) {
            if (btSocket != null) {
                try {
                    btSocket.getOutputStream().write(("M|"+Integer.toString((int)X_Speed)+"|"+Integer.toString((int)Y_Speed)+"|"+Integer.toString((int)Rot_Speed)+"!").getBytes());
                    //toastMsg("Sending velocities");
                } catch (IOException e) {
                    toastMsg("Error");
                }
            }
            time_prev = time.uptimeMillis();
            updateBrake = false;
        }
    }
    private void setToOdo() {
        if (btSocket!=null) {
            try {
                btSocket.getOutputStream().write("L|0|0|0!".getBytes());
                toastMsg("Localization method: odometry");
            } catch (IOException e) {
                toastMsg("Error");
            }
        }
    }

    private void setToOdoIMU() {
        if (btSocket!=null) {
            try {
                btSocket.getOutputStream().write("L|1|0|0!".getBytes());
                toastMsg("Localization method: Odometry + IMU");
            } catch (IOException e) {
                toastMsg("Error");
            }
        }
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
