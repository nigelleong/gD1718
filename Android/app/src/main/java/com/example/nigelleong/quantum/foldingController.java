package com.example.nigelleong.quantum;

import android.bluetooth.BluetoothSocket;
import android.os.Bundle;
import android.support.v7.app.AppCompatActivity;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.Toast;

import java.io.IOException;

/**
 * Created by nigelleong on 2/5/18.
 */

public class foldingController extends AppCompatActivity implements View.OnClickListener {

    Button btnFoldSeats, btnFoldWings;

    BluetoothSocket btSocket;
    BluetoothSocketHelper bluetoothSocketHelper;

    byte[] buffer = new byte[1024];  // buffer store for the stream
    int bytes; // bytes returned from read()

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_folding);

        bluetoothSocketHelper = ((BluetoothSocketHelper) getApplicationContext());
        btSocket = bluetoothSocketHelper.getBluetoothSocket();
        Log.d("foldingController",btSocket.toString());

        btnFoldSeats = (Button)findViewById(R.id.btn_foldSeats);
        btnFoldWings = (Button)findViewById(R.id.btn_foldWings);

        btnFoldSeats.setOnClickListener(this);
        btnFoldWings.setOnClickListener(this);
    }

    @Override
    public void onClick(View view) {
        switch(view.getId()) {
            case R.id.btn_foldSeats:
                foldSeats();
                break;
            case R.id.btn_foldWings:
                robotWings();
                break;
            default:
                break;
        }
    }

    private void foldSeats() {
        if (btSocket!=null) {
            try {
                btSocket.getOutputStream().write("M|2000|2000|270!".getBytes());
            } catch (IOException e) {
                toastMsg("Error");
            }
        }
    }

    private void robotWings() {
        if (btSocket!=null) {
            try {
                btSocket.getOutputStream().write("M|2000|2000|270!".getBytes());
            } catch (IOException e) {
                toastMsg("Error");
            }
        }
    }

    private void toastMsg(String s) {
        Toast.makeText(getApplicationContext(),s,Toast.LENGTH_LONG).show();
    }
}
