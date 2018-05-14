package com.example.nigelleong.quantum.model;

import com.example.nigelleong.quantum.R;

import android.content.Intent;
import android.os.Bundle;
import android.support.v4.app.Fragment;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;

import java.util.ArrayList;
import java.util.List;


public class EfficiencyModuleInferface extends Fragment implements View.OnClickListener{

    Button btn_book_eff_mod;

    public EfficiencyModuleInferface() {
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
        View view =  inflater.inflate(R.layout.activity_efficiencymodule, container, false);
        btn_book_eff_mod = view.findViewById(R.id.btn_book_eff_mod);
        btn_book_eff_mod.setOnClickListener(this);

        return view;
    }

    @Override
    public void onClick(View view){
        switch(view.getId()) {
//            case R.id.btn_book_eff_mod:
//                Intent intent = new Intent(getActivity(), ListViewInterface.class);
//                intent.putExtras(information);
//                startActivity(intent);
//                break;
            default:
                break;
        }

    }



}
