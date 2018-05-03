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

    Button btnDriving, btnFolding, btnAnalog;
    String address = null;
    private ProgressDialog progress;
    BluetoothAdapter myBluetooth = null;

    BluetoothSocket btSocket = null;
    BluetoothSocketHelper bluetoothSocketHelper;

    private boolean isBtConnected = false;
    public static String EXTRA_BT_SOCKET = "bluetooth_socket";

    //SPP UUID. Look for it
    static final UUID myUUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");
//    static final UUID myUUID = UUID.fromString("0983cb76-a059-40c1-be70-7687f86fff56");


    byte[] buffer = new byte[1024];  // buffer store for the stream
    int bytes; // bytes returned from read()


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_standby);

        address = getIntent().getStringExtra(deviceListController.EXTRA_ADDRESS);
        bluetoothSocketHelper = ((BluetoothSocketHelper) getApplicationContext());

        btnDriving = (Button)findViewById(R.id.btn_driving);
        btnFolding = (Button)findViewById(R.id.btn_folding);
        btnAnalog = (Button)findViewById(R.id.btn_analog);

        new ConnectBT().execute();

        btnDriving.setOnClickListener(this);
        btnFolding.setOnClickListener(this);
        btnAnalog.setOnClickListener(this);
    }

    @Override
    public void onClick(View view) {
        switch(view.getId()) {
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
            default:
                break;
        }
    }

    private void toastMsg(String s) {
        Toast.makeText(getApplicationContext(),s,Toast.LENGTH_LONG).show();
    }

    private class ConnectBT extends AsyncTask<Void, Void, Void> {
        private boolean ConnectSuccess = true;

        @Override
        protected void onPreExecute()
        {
            progress = ProgressDialog.show(standbyController.this, "Connecting...", "Please wait");  //show a progress dialog
        }

        @Override
        protected Void doInBackground(Void... devices) //while the progress dialog is shown, the connection is done in background
        {
            try
            {
                if (btSocket == null || !isBtConnected) {
                    myBluetooth = BluetoothAdapter.getDefaultAdapter();//get the mobile bluetooth adapter
                    BluetoothDevice myBt = myBluetooth.getRemoteDevice(address);//connects to the device's address and checks if it's available
                    btSocket = myBt.createInsecureRfcommSocketToServiceRecord(myUUID);//create a RFCOMM (SPP) connection
                    BluetoothAdapter.getDefaultAdapter().cancelDiscovery();
                    btSocket.connect();//start connection
                }
            }
            catch (IOException e)
            {
                Log.d("e","fails to connect");
                ConnectSuccess = false;
            }
            return null;
        }
        @Override
        protected void onPostExecute(Void result)
        {
            super.onPostExecute(result);

            if (!ConnectSuccess)
            {
                toastMsg("Connection Failed");
                try {
                    btSocket.close();
                } catch (IOException e) {
                    Log.d("standbyController","Could not close connection:" + e.toString());
                    e.printStackTrace();
                }
                finish();
            }
            else
            {
                toastMsg("Connected!");
                isBtConnected = true;
                Log.d("standbyController",btSocket.toString());
                bluetoothSocketHelper.setBluetoothSocket(btSocket);
            }
            progress.dismiss();
        }
    }

}
