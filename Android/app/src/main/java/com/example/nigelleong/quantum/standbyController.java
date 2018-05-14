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
import android.widget.EditText;
import android.widget.Toast;

import com.example.nigelleong.quantum.helper.GlobalState;

import java.io.IOException;

public class standbyController extends AppCompatActivity implements View.OnClickListener {

    Button btnDriving, btnFolding, btnAnalog, btnLayouts, btnPID, btnSubmitPose;
    EditText txtPoseX, txtPoseY, txtPoseRot;

    String pose_x, pose_y, pose_rot;

    BluetoothSocket btSocket = null;
    GlobalState globalState;

    byte[] buffer = new byte[1024];  // buffer store for the stream
    int bytes; // bytes returned from read()


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_standby);


        globalState = ((GlobalState) getApplicationContext());
        btSocket = globalState.getBluetoothSocket();
        Log.d("standbyController", btSocket.toString());

        btnDriving = (Button)findViewById(R.id.btn_driving);
        btnFolding = (Button)findViewById(R.id.btn_folding);
        btnAnalog = (Button)findViewById(R.id.btn_analog);
        btnLayouts = (Button)findViewById(R.id.btn_layouts);
        btnPID = (Button)findViewById(R.id.btn_PID);
        btnSubmitPose = (Button) findViewById(R.id.btn_submit_pose);

        txtPoseX = (EditText) findViewById(R.id.txt_pose_x_value);
        txtPoseY = (EditText) findViewById(R.id.txt_pose_y_value);
        txtPoseRot = (EditText) findViewById(R.id.txt_pose_rot_value);

        btnDriving.setOnClickListener(this);
        btnFolding.setOnClickListener(this);
        btnAnalog.setOnClickListener(this);
        btnLayouts.setOnClickListener(this);
        btnPID.setOnClickListener(this);
        btnSubmitPose.setOnClickListener(this);

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
                if (btSocket != null) {
                    try {
                        btSocket.getOutputStream().write("S|1|0|0!".getBytes());
                    } catch (IOException e) {
                        toastMsg("Error");
                    }
                }
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
            case R.id.btn_submit_pose:
                if (btSocket != null) {
                    try {
                        pose_x = txtPoseX.getText().toString();
                        pose_y = txtPoseY.getText().toString();
                        pose_rot = txtPoseRot.getText().toString();
                        if (checkPoseValidity(pose_x, pose_y, pose_rot)){
                            Log.d("pose", "P|" + pose_x + "|" + pose_y + "|" + pose_rot + "!");
                            btSocket.getOutputStream().write(("P|" + pose_x + "|" + pose_y + "|" + pose_rot + "!").getBytes());
                        }
                    } catch (IOException e) {
                        toastMsg("Error");
                    }
                }
                break;
            default:
                break;
        }
    }

    private void toastMsg(String s) {
        Toast.makeText(getApplicationContext(), s, Toast.LENGTH_LONG).show();
    }

    private boolean checkPoseValidity(String pose_x, String pose_y, String pose_rot){

        if ( pose_x.matches("") || Integer.valueOf(pose_x) < 0 || Integer.valueOf(pose_x) > 2500 ){
            toastMsg("Pose x must be in the range of 0 to 2500");
            return false;
        }
        if (pose_y.matches("") ||Integer.valueOf(pose_y) < 0 || Integer.valueOf(pose_y) > 2500){
            toastMsg("Pose y must be in the range of 0 to 2500");
            return false;
        }
        if (pose_rot.matches("") ||Integer.valueOf(pose_rot) < -180 || Integer.valueOf(pose_rot) > 180){
            toastMsg("Pose Rotation must be in the range of -180 to 180");
            return false;
        }
        return true;
    }

}
