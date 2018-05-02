package com.example.nigelleong.quantum;

/**
 * Created by nigelleong on 28/4/18.
 */

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

public class robotControl extends AppCompatActivity implements View.OnClickListener {

    Button btnUp, btnDown, btnLeft, btnRight, btnFoldSeat, btnOdoIMU, btnBlinkLED;
    Button btnPeak, btnOffPeak, btn1;
    String address = null;
    private ProgressDialog progress;
    BluetoothAdapter myBluetooth = null;
    BluetoothSocket btSocket = null;
    private boolean isBtConnected = false;
    //SPP UUID. Look for it
    static final UUID myUUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");

    byte[] buffer = new byte[1024];  // buffer store for the stream
    int bytes; // bytes returned from read()


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_robot_control);

        address = getIntent().getStringExtra(DeviceList.EXTRA_ADDRESS);

        btnUp = (Button)findViewById(R.id.btn_up);
        btnDown = (Button)findViewById(R.id.btn_down);
        btnLeft = (Button)findViewById(R.id.btn_left);
        btnRight = (Button)findViewById(R.id.btn_right);
        btnFoldSeat = (Button)findViewById(R.id.btn_fold_seat);
        btnOdoIMU = (Button)findViewById(R.id.btn_odometry_IMU);
        btnBlinkLED = (Button)findViewById(R.id.btn_blink_led);
        btnPeak = (Button)findViewById(R.id.btn_peak);
        btn1 = (Button)findViewById(R.id.btn1);

        new ConnectBT().execute();

        btnUp.setOnClickListener(this);
        btnDown.setOnClickListener(this);
        btnLeft.setOnClickListener(this);
        btnRight.setOnClickListener(this);
        btnFoldSeat.setOnClickListener(this);
        btnOdoIMU.setOnClickListener(this);
        btnBlinkLED.setOnClickListener(this);
        btnPeak.setOnClickListener(this);
        btn1.setOnClickListener(this);


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
            case R.id.btn_fold_seat:
                robotFoldSeat();
                break;
            case R.id.btn_odometry_IMU:
                localizationOdoIMU();
                break;
            case R.id.btn_blink_led:
                arduinoBlinkLED();
                break;
            case R.id.btn_peak:
                configPeak();
                break;
            case R.id.btn1:

                break;
            default:
                break;
        }
    }

    private void robotFoldSeat() {
        if (btSocket!=null) {
            try {
                btSocket.getOutputStream().write("S|1||!".getBytes());
                toastMsg("Command 'fold seat' sent");
            } catch (IOException e) {
                toastMsg("Error");
            }
        }
//        toastMsg("Test command 'fold seat' sent");
    }

    private void localizationOdoIMU() {
        if (btSocket!=null) {
            try {
                btSocket.getOutputStream().write("L|1||!".getBytes());
                toastMsg("Command 'btnOdoIMU' sent");
            } catch (IOException e) {
                toastMsg("Error");
            }
        }
//        toastMsg("Test command 'fold wings' sent");
    }

    private void robotGoUp() {
        if (btSocket!=null) {
            try {
//                btSocket.getOutputStream().write("M|2000|2000|90!".getBytes());
                btSocket.getOutputStream().write("M|100|0|0!".getBytes());
//                btSocket.getOutputStream().write("s".getBytes(Charset.forName("UTF-8")));
//                BufferedOutputStream buffStream = new BufferedOutputStream(btSocket.getOutputStream());
//                buffStream.write("M".getBytes());
 //               buffStream.flush();
            } catch (IOException e) {
                toastMsg("Error");
            }
        }
//        toastMsg("Test command 'up' sent");
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
//        toastMsg("Test command 'down' sent");
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
//        toastMsg("Test command 'left' sent");
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
//        toastMsg("Test command 'right' sent");
    }

    private void arduinoBlinkLED() {
        if (btSocket!=null) {
            try {
                btSocket.getOutputStream().write("B|2||!".getBytes());
                toastMsg("Command 'blink led 2 times' sent");
            } catch (IOException e) {
                toastMsg("Error");
            }
        }
//        toastMsg("Test command 'blink led 2 times' sent");
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
//        toastMsg("Test command 'Config Peak completed' sent");
    }

    private void toastMsg(String s) {
        Toast.makeText(getApplicationContext(),s,Toast.LENGTH_LONG).show();
    }

    private class ConnectBT extends AsyncTask<Void, Void, Void> {
        private boolean ConnectSuccess = true;

        @Override
        protected void onPreExecute()
        {
            progress = ProgressDialog.show(robotControl.this, "Connecting...", "Please wait");  //show a progress dialog
        }

        @Override
        protected Void doInBackground(Void... devices) //while the progress dialog is shown, the connection is done in background
        {
            try
            {
                if (btSocket == null || !isBtConnected) {
                    myBluetooth = BluetoothAdapter.getDefaultAdapter();//get the mobile bluetooth device
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
                finish();
            }
            else
            {
                toastMsg("Connected!");
                isBtConnected = true;
            }
            progress.dismiss();
        }
    }

}