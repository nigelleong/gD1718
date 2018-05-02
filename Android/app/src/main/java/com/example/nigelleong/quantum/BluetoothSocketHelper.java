package com.example.nigelleong.quantum;

import android.bluetooth.BluetoothSocket;

import java.io.Serializable;

/**
 * Created by nigelleong on 2/5/18.
 */

public class BluetoothSocketHelper implements Serializable{

    public BluetoothSocket btSocket;

    public BluetoothSocket getBluetoothSocket(){
        return btSocket;
    }

    public void setBluetoothSocket(BluetoothSocket btSocket){
        this.btSocket = btSocket;
    }


}
