package com.example.nigelleong.quantum;

import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.view.View;
import android.widget.Button;
import android.widget.ListView;

import java.util.Set;
import java.util.ArrayList;
import android.widget.Toast;
import android.widget.ArrayAdapter;
import android.widget.AdapterView;
//import android.widget.AdapterView.OnClickListener;
import android.widget.TextView;
import android.content.Intent;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;

public class deviceListController extends AppCompatActivity implements View.OnClickListener {

    Button btnPaired;
    ListView devicelist;

    private BluetoothAdapter myBluetoothAdapter = null;
    private Set<BluetoothDevice> pairedDevices;
    public static String EXTRA_ADDRESS = "device_address";

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_device_list);

        btnPaired = (Button)findViewById(R.id.button);
        devicelist = (ListView)findViewById(R.id.listView);

        //if the device has bluetooth
        myBluetoothAdapter = BluetoothAdapter.getDefaultAdapter();

        if(myBluetoothAdapter == null) {
            //Show a mensag. that the device has no bluetooth adapter
            Toast.makeText(getApplicationContext(), "Bluetooth Device Not Available", Toast.LENGTH_LONG).show();
            finish();
        } else if(!myBluetoothAdapter.isEnabled()) {
            //Ask to turn on bluetooth
            Intent turnBTon = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
            startActivityForResult(turnBTon,1);
        }

        btnPaired.setOnClickListener(this);
    }

    @Override
    public void onClick(View view) {
        switch(view.getId()) {
            case R.id.button:
                pairedDevicesList();
            default:
                break;
        }
    }

    private void pairedDevicesList()
    {
        pairedDevices = myBluetoothAdapter.getBondedDevices();
        ArrayList list = new ArrayList();

        if (pairedDevices.size()>0) {
            for(BluetoothDevice bt : pairedDevices) {
                list.add(bt.getName() + "\n" + bt.getAddress()); //Get the device's name and the address
            }
        } else {
            Toast.makeText(getApplicationContext(), "No Paired Bluetooth Devices Found.", Toast.LENGTH_LONG).show();
        }

        final ArrayAdapter adapter = new ArrayAdapter(this,android.R.layout.simple_list_item_1, list);
        devicelist.setAdapter(adapter);
        devicelist.setOnItemClickListener(myListClickListener); //Method called when the device from the list is clicked

    }

    private AdapterView.OnItemClickListener myListClickListener = new AdapterView.OnItemClickListener()
    {
        public void onItemClick (AdapterView<?> av, View v, int arg2, long arg3)
        {
            // Get device MAC address, the last 17 chars in the View
            String info = ((TextView) v).getText().toString();
            String address = info.substring(info.length() - 17);

            // Make an intent to start next activity.
            Intent intent = new Intent(deviceListController.this, afterPairing.class);
            intent.putExtra(EXTRA_ADDRESS, address);
            startActivity(intent);
        }
    };


}
