package com.example.nigelleong.quantum;

import android.app.Application;
import android.bluetooth.BluetoothSocket;
import android.util.Log;

import java.io.Serializable;

/**
 * Created by nigelleong on 2/5/18.
 */

public class BluetoothSocketHelper extends Application{

    public BluetoothSocket btSocket;

    public BluetoothSocketHelper(){}

    public BluetoothSocket getBluetoothSocket(){
        return btSocket;
    }

    public void setBluetoothSocket(BluetoothSocket btSocket)
    {
        Log.d("BluetoothSocketHelper",btSocket.toString());
        this.btSocket = btSocket;
    }


}
