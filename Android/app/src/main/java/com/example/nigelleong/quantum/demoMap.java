package com.example.nigelleong.quantum;

import android.bluetooth.BluetoothSocket;
import android.content.Context;
import android.content.Intent;
import android.content.res.Resources;
import android.graphics.Point;
import android.support.v4.app.FragmentActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.view.inputmethod.InputMethodManager;
import android.widget.Button;
import android.widget.EditText;
import android.widget.Toast;

import com.example.nigelleong.quantum.helper.GlobalState;
import com.google.android.gms.maps.CameraUpdateFactory;
import com.google.android.gms.maps.GoogleMap;
import com.google.android.gms.maps.OnMapReadyCallback;
import com.google.android.gms.maps.Projection;
import com.google.android.gms.maps.SupportMapFragment;
import com.google.android.gms.maps.model.BitmapDescriptorFactory;
import com.google.android.gms.maps.model.LatLng;
import com.google.android.gms.maps.model.MapStyleOptions;
import com.google.android.gms.maps.model.Marker;
import com.google.android.gms.maps.model.MarkerOptions;

import java.io.IOException;

public class demoMap extends FragmentActivity implements View.OnTouchListener, View.OnClickListener,OnMapReadyCallback, GoogleMap.OnInfoWindowClickListener{

    BluetoothSocket btSocket;

    int evalue = 1;
    EditText txtStarting, txtDestination;
    Button btnHire;
    GlobalState globalState;

    private GoogleMap mMap;
    CustomInfoWindowGoogleMap customInfoWindow;
    private static final LatLng mapCenter = new LatLng(1.4383022,103.8184172);
    private static final LatLng mapSchool = new LatLng(1.3202532,103.7435057);
    private static final LatLng mapNTU = new LatLng(1.3487729,103.6790481);
    private static final LatLng mapChangi = new LatLng(1.3640377,103.9881359);
    private static final LatLng mapMBS = new LatLng(1.2833808,103.8585377);
    private static final LatLng mapWoodlandTrainCkpt = new LatLng(1.4437066,103.7674462);
    private static final LatLng mapBotanicGarden = new LatLng(1.314804,103.8121228);
    private static final LatLng mapUSS = new LatLng(1.2541727,103.820856);
    private static final LatLng mapSingaporeZoo = new LatLng(1.4043539,103.7908343);
    private static final LatLng mapBedokMall = new LatLng(1.3323294,103.9184953);


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_demomaps);
        // Obtain the SupportMapFragment and get notified when the map is ready to be used.
        SupportMapFragment mapFragment = (SupportMapFragment) getSupportFragmentManager()
                .findFragmentById(R.id.map);
        mapFragment.getMapAsync(this);

        globalState = (GlobalState) getApplicationContext();
        btSocket = globalState.getBluetoothSocket();

        txtStarting = (EditText)findViewById(R.id.txt_starting);
        txtDestination = (EditText)findViewById(R.id.txt_destination);
        btnHire = (Button)findViewById(R.id.btn_hire);

        txtStarting.setOnTouchListener(this);
        txtDestination.setOnTouchListener(this);
        btnHire.setOnClickListener(this);
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
    public boolean onTouch(View view, MotionEvent arg1) {
        switch(view.getId()) {
            case R.id.txt_starting:
                evalue = 1;
                Log.d("evalue", "1");
//                btnSetPoint.setText("Set as starting point");
                return false;
            case R.id.txt_destination:
                evalue = 2;
                Log.d("evalue", "2");
//                btnSetPoint.setText("Set as destination");
                return false;
            default:
                return false;
        }
    }

    @Override
    public void onClick(View view) {
        switch(view.getId()) {
            case R.id.btn_hire:
                if (txtStarting.getText().toString().trim().length() == 0) {
                    toastMsg("Select a starting point to continue");
                    break;
                }
                if (txtDestination.getText().toString().trim().length() == 0) {
                    toastMsg("Select a destination to continue");
                    break;
                }
                if (txtDestination.getText().toString().matches(txtStarting.getText().toString())) {
                    toastMsg("Starting point and destination can't be the same");
                    break;
                }
                Intent demoIntent = new Intent(this, demoLayoutController.class);
                startActivity(demoIntent);
                break;
            default:
                break;
        }
    }


    @Override
    public void onMapReady(GoogleMap googleMap){
        mMap = googleMap;

        // Construct markers
        mMap.addMarker(new MarkerOptions()
                .position(mapSchool)
                .title("Commonwealth Secondary School"));

        mMap.addMarker(new MarkerOptions()
                .position(mapNTU)
                .title("Nanyang Technological University"));

        mMap.addMarker(new MarkerOptions()
                .position(mapChangi)
                .title("Changi Airport"));

        mMap.addMarker(new MarkerOptions()
                .position(mapMBS)
                .title("Marina Bay Sands"));

        mMap.addMarker(new MarkerOptions()
                .position(mapWoodlandTrainCkpt)
                .title("Woodland Train Checkpoint"));

        mMap.addMarker(new MarkerOptions()
                .position(mapBotanicGarden)
                .title("Botanic Gardens"));

        mMap.addMarker(new MarkerOptions()
                .position(mapUSS)
                .title("Universal Studios Singapore"));

        mMap.addMarker(new MarkerOptions()
                .position(mapSingaporeZoo)
                .title("Singapore Zoo"));

        mMap.addMarker(new MarkerOptions()
                .position(mapBedokMall)
                .title("Bedok Mall"));



//        CustomInfoWindowGoogleMap customInfoWindow = new CustomInfoWindowGoogleMap(this);
        customInfoWindow = new CustomInfoWindowGoogleMap(this);
        mMap.setInfoWindowAdapter(customInfoWindow);

        // Customize Map Style
        try {
            // Customise the styling of the base map using a JSON object defined
            // in a raw resource file.
            boolean success = googleMap.setMapStyle(
                    MapStyleOptions.loadRawResourceStyle(
                            this, R.raw.style_json));

            if (!success) {
                Log.e("Maps", "Style parsing failed.");
            }
        } catch (Resources.NotFoundException e) {
            Log.e("Maps", "Can't find style. Error: ", e);
        }

        mMap.setOnMarkerClickListener(new GoogleMap.OnMarkerClickListener() {
            @Override
            public boolean onMarkerClick(Marker marker) {
                // Calculate required horizontal shift for current screen density
                final int dX = getResources().getDimensionPixelSize(R.dimen.map_dx);
                // Calculate required vertical shift for current screen density
                final int dY = getResources().getDimensionPixelSize(R.dimen.map_dy);
                final Projection projection = mMap.getProjection();
                final Point markerPoint = projection.toScreenLocation(
                        marker.getPosition()
                );
                // Shift the point we will use to center the map
                markerPoint.offset(dX, dY);
                final LatLng newLatLng = projection.fromScreenLocation(markerPoint);
                mMap.animateCamera(CameraUpdateFactory.newLatLng(newLatLng), 300, null);

                // Show the info window (as the overloaded method would)
                marker.showInfoWindow();

                // hide soft keyboard
                View view = getCurrentFocus();
                InputMethodManager imm = (InputMethodManager) getSystemService(Context.INPUT_METHOD_SERVICE);
                imm.hideSoftInputFromWindow(view.getWindowToken(), 0);

                return true; // Consume the event since it was dealt with

            }
        });


        // Move camera
        mMap.moveCamera(CameraUpdateFactory.newLatLngZoom(mapCenter, (float)10.5));

        mMap.setOnInfoWindowClickListener(this);
    }

    @Override
    public void onInfoWindowClick(Marker marker) {
//        Toast.makeText(this, "Info window clicked, jump to navigation/layout selection",
//                Toast.LENGTH_SHORT).show();
//        Intent demoIntent = new Intent(this, demoLayoutController.class);
//        demoIntent.putExtra("destination", marker.getTitle());
//        startActivity(demoIntent);
        switch(evalue) {
            case 1:
                if (globalState.getMarkerDestination() != null && marker.getTitle().matches(globalState.getMarkerDestination().getTitle())){
                    toastMsg("Starting point and destination can't be the same");
                    return;
                }
                txtStarting.setText(marker.getTitle());
                if (globalState.getStart() != null){
                    globalState.getMarkerStart().setIcon(BitmapDescriptorFactory.defaultMarker(BitmapDescriptorFactory.HUE_RED));
                }
                globalState.setMarkerStart(marker);
                globalState.setStart(marker.getTitle());
                txtDestination.requestFocus();
                evalue = 2;
                break;
            case 2:
                if (globalState.getMarkerStart() != null && marker.getTitle().matches(globalState.getMarkerStart().getTitle())){
                    toastMsg("Starting point and destination can't be the same");
                    return;
                }
                txtDestination.setText(marker.getTitle());
                if (globalState.getMarkerDestination() != null){
                    globalState.getMarkerDestination().setIcon(BitmapDescriptorFactory.defaultMarker(BitmapDescriptorFactory.HUE_RED));
                }
                globalState.setMarkerDestination(marker);
                globalState.setDestination(marker.getTitle());
                break;
            default:
                break;
        }
        marker.hideInfoWindow();
        marker.setIcon(BitmapDescriptorFactory.defaultMarker(180));
        mMap.animateCamera(CameraUpdateFactory.newLatLngZoom(mapCenter, (float)10.5),300,null);
    }

    private void toastMsg(String s) {
        Toast.makeText(getApplicationContext(),s,Toast.LENGTH_LONG).show();
    }
}


