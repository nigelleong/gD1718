package com.example.nigelleong.quantum;

import android.content.Intent;
import android.os.Bundle;
import android.support.v7.app.AppCompatActivity;
import android.view.View;
import android.widget.Button;
import android.widget.Toast;

import com.example.nigelleong.quantum.helper.GlobalState;
import com.google.android.gms.maps.model.BitmapDescriptorFactory;

public class demoDispatchController extends AppCompatActivity implements View.OnClickListener{

    Button btnAcknowledge;
    GlobalState globalState;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_demodispatch);

        btnAcknowledge = (Button)findViewById(R.id.btn_acknowledge);
        btnAcknowledge.setOnClickListener(this);

        globalState = ((GlobalState) getApplicationContext());
    }

    @Override
    public void onClick(View view){
        switch(view.getId()){
            case(R.id.btn_acknowledge):
                Intent demoIntent = new Intent(this, demoMap.class);
                demoIntent.addFlags(Intent.FLAG_ACTIVITY_CLEAR_TOP);
                startActivity(demoIntent);
                finish();
            default:
                break;
        }
    }

    @Override
    protected void onDestroy(){
        toastMsg("Your ride is arriving in " + globalState.getEta());
        super.onDestroy();
    }

    private void toastMsg(String s) {
        Toast.makeText(getApplicationContext(),s,Toast.LENGTH_LONG).show();
    }
}
