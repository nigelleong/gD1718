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

public class drivingController extends AppCompatActivity implements View.OnClickListener {

    Button btnUp, btnDown, btnLeft, btnRight, btnStandby, btnReset, btnOdo, btnOdoIMU, btnOdoIMUNFC;

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
//        btnStandby = (Button)findViewById(R.id.btn_driving_standby);
        btnReset = (Button)findViewById(R.id.btn_reset);
        btnOdo = (Button)findViewById(R.id.btn_odo);
        btnOdoIMU = (Button)findViewById(R.id.btn_odo_imu);
        btnOdoIMUNFC = (Button)findViewById(R.id.btn_odo_imu_nfc);


        btnUp.setOnClickListener(this);
        btnDown.setOnClickListener(this);
        btnLeft.setOnClickListener(this);
        btnRight.setOnClickListener(this);
//        btnStandby.setOnClickListener(this);
        btnReset.setOnClickListener(this);
        btnOdo.setOnClickListener(this);
        btnOdoIMU.setOnClickListener(this);
        btnOdoIMUNFC.setOnClickListener(this);


        //Switch to DRIVING (state = 1);
        if (btSocket!=null) {
            try {
                btSocket.getOutputStream().write("S|1|0|0!".getBytes());
            } catch (IOException e) {
                toastMsg("Error");
            }
        }
        Log.d("STATE", "1");
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
//            case R.id.btn_driving_standby:
//                Intent drivingIntent = new Intent(drivingController.this, standbyController.class);
//                startActivity(drivingIntent);
//                break;
            case R.id.btn_reset:
                resetPose();
                break;
            case R.id.btn_odo:
                setToOdo();
                break;
            case R.id.btn_odo_imu:
                setToOdoIMU();
                break;
            case R.id.btn_odo_imu_nfc:
                setToOdoIMUNFC();
                break;
            default:
                break;
        }
    }


    private void robotGoUp() {
        if (btSocket!=null) {
            try {
//                btSocket.getOutputStream().write("M|2000|2000|90!".getBytes());
                btSocket.getOutputStream().write("M|100|0|0!".getBytes());
                toastMsg("Command 'up' sent");
            } catch (IOException e) {
                toastMsg("Error");
            }
        }
    }

    private void robotGoDown() {
        if (btSocket!=null) {
            try {
                btSocket.getOutputStream().write("M|-100|0|0!".getBytes());
                toastMsg("Command 'down' sent");
            } catch (IOException e) {
                toastMsg("Error");
            }
        }
    }

    private void robotGoLeft() {
        if (btSocket!=null) {
            try {
                btSocket.getOutputStream().write("M|0|100|0!".getBytes());
                toastMsg("Command 'left' sent");
            } catch (IOException e) {
                toastMsg("Error");
            }
        }
    }

    private void robotGoRight() {
        if (btSocket!=null) {
            try {
                btSocket.getOutputStream().write("M|0|-100|0!".getBytes());
                toastMsg("Command 'right' sent");
            } catch (IOException e) {
                toastMsg("Error");
            }
        }
    }

    private void resetPose() {
        if (btSocket!=null) {
            try {
                btSocket.getOutputStream().write("P|0|0|0!".getBytes());
                toastMsg("Pose set to (0,0,0)");
            } catch (IOException e) {
                toastMsg("Error");
            }
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

    private void setToOdoIMUNFC() {
        if (btSocket!=null) {
            try {
                btSocket.getOutputStream().write("L|2|0|0!".getBytes());
                toastMsg("Localization method: Odometry + IMU + NFC");
            } catch (IOException e) {
                toastMsg("Error");
            }
        }
    }


    private void toastMsg(String s) {
        Toast.makeText(getApplicationContext(),s,Toast.LENGTH_LONG).show();
    }
}
