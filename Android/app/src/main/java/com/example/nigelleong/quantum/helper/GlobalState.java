package com.example.nigelleong.quantum.helper;

import android.app.Application;
import android.bluetooth.BluetoothSocket;
import android.util.Log;

import com.google.android.gms.maps.model.Marker;

import java.io.Serializable;

/**
 * Created by nigelleong on 2/5/18.
 */

public class GlobalState extends Application{

    public BluetoothSocket btSocket;
    public String destination, start;
    public String eta;
    Marker markerStart = null, markerDestination = null;

    public GlobalState(){}

    public Marker getMarkerStart() {
        return markerStart;
    }

    public void setMarkerStart(Marker markerStart) {
        this.markerStart = markerStart;
    }

    public Marker getMarkerDestination() {
        return markerDestination;
    }

    public void setMarkerDestination(Marker markerDestination) {
        this.markerDestination = markerDestination;
    }

    public BluetoothSocket getBluetoothSocket(){
        return btSocket;
    }

    public void setBluetoothSocket(BluetoothSocket btSocket)
    {
        Log.d("GlobalState",btSocket.toString());
        this.btSocket = btSocket;
    }

    public void setDestination(String destination) {
        this.destination = destination;
    }

    public void setStart(String start) {
        this.start = start;
    }

    public void setEta(String eta) {
        this.eta = eta;
    }

    public String getDestination() {
        return destination;
    }

    public String getStart() {
        return start;
    }

    public String getEta() {
        return eta;
    }


}
