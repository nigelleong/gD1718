package com.example.nigelleong.quantum.model;

import android.bluetooth.BluetoothSocket;
import android.content.Intent;
import android.os.Bundle;
import android.support.v4.app.Fragment;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;

import com.example.nigelleong.quantum.R;
import com.example.nigelleong.quantum.demoDispatchController;
import com.example.nigelleong.quantum.helper.GlobalState;

import java.io.IOException;

public class PrivateModuleInterface extends Fragment implements View.OnClickListener{

    Button btnBook;
    GlobalState globalState;
    BluetoothSocket btSocket;
    TextView txt_eta;

    public PrivateModuleInterface() {
        // Required empty public constructor
    }

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

    }

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,
                             Bundle savedInstanceState) {
        // Inflate the layout for this fragment
        View view =  inflater.inflate(R.layout.activity_privatemodule, container, false);
        globalState = ((GlobalState) getActivity().getApplicationContext());
        btSocket = globalState.getBluetoothSocket();

        txt_eta = (TextView)view.findViewById(R.id.txt_priv_mod_eta);
        btnBook = (Button)view.findViewById(R.id.btn_book_priv_mod);
        btnBook.setOnClickListener(this);

        return view;
    }

    @Override
    public void onClick(View view){
        switch(view.getId()) {
            case R.id.btn_book_priv_mod:
                Intent demoDispatchIntent = new Intent(getActivity(), demoDispatchController.class);
                startActivity(demoDispatchIntent);
                globalState.setEta(txt_eta.getText().toString());
                getActivity().finish();

                // Send layout choice to Arduino
                if (btSocket!=null) {
                    try {
                        btSocket.getOutputStream().write("K|2|0|0!".getBytes());
                    } catch (IOException e) {
                        toastMsg("Error");
                    }
                }
                Log.d("Layout", "Private 2");
                break;
            default:
                break;
        }
    }

    private void toastMsg(String s) {
        Toast.makeText(getActivity().getApplicationContext(),s,Toast.LENGTH_LONG).show();
    }
}
