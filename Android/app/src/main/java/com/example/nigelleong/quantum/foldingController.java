package com.example.nigelleong.quantum;

import android.bluetooth.BluetoothSocket;
import android.os.Bundle;
import android.support.v7.app.AppCompatActivity;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.Toast;

import java.io.IOException;

/**
 * Created by nigelleong on 2/5/18.
 */

public class foldingController extends AppCompatActivity implements View.OnClickListener {

    Button btnOpenWingLeft, btnOpenWingBoth, btnOpenWingRight, btnCloseWingLeft, btnCloseWingBoth, btnCloseWingRight;
    Button btnFoldUpLeft, btnFoldUpMiddle, btnFoldUpRight, btnFoldDownLeft, btnFoldDownMiddle, btnFoldDownRight;

    BluetoothSocket btSocket;
    BluetoothSocketHelper bluetoothSocketHelper;

    byte[] buffer = new byte[1024];  // buffer store for the stream
    int bytes; // bytes returned from read()

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_folding);

        bluetoothSocketHelper = ((BluetoothSocketHelper) getApplicationContext());
        btSocket = bluetoothSocketHelper.getBluetoothSocket();
        Log.d("foldingController",btSocket.toString());

        btnOpenWingLeft = (Button)findViewById(R.id.btn_open_wing_left);
        btnOpenWingBoth = (Button)findViewById(R.id.btn_open_wing_both);
        btnOpenWingRight = (Button)findViewById(R.id.btn_open_wing_right);
        btnCloseWingLeft = (Button)findViewById(R.id.btn_close_wing_left);
        btnCloseWingBoth = (Button)findViewById(R.id.btn_close_wing_both);
        btnCloseWingRight = (Button)findViewById(R.id.btn_close_wing_right);

        btnFoldUpLeft = (Button)findViewById(R.id.btn_seat_left_up);
        btnFoldUpMiddle = (Button)findViewById(R.id.btn_seat_middle_up);
        btnFoldUpRight = (Button)findViewById(R.id.btn_seat_right_up);
        btnFoldDownLeft = (Button)findViewById(R.id.btn_seat_left_down);
        btnFoldDownMiddle = (Button)findViewById(R.id.btn_seat_middle_down);
        btnFoldDownRight = (Button)findViewById(R.id.btn_seat_right_down);

        btnOpenWingLeft.setOnClickListener(this);
        btnOpenWingBoth.setOnClickListener(this);
        btnOpenWingRight.setOnClickListener(this);
        btnCloseWingLeft.setOnClickListener(this);
        btnCloseWingBoth.setOnClickListener(this);
        btnCloseWingRight.setOnClickListener(this);

        btnFoldUpLeft.setOnClickListener(this);
        btnFoldUpMiddle.setOnClickListener(this);
        btnFoldUpRight.setOnClickListener(this);
        btnFoldDownLeft.setOnClickListener(this);
        btnFoldDownMiddle.setOnClickListener(this);
        btnFoldDownRight.setOnClickListener(this);

        Log.d("STATE", "2?");
    }

    @Override
    protected void onDestroy() {
        if (btSocket!=null) {
            try {
                btSocket.getOutputStream().write("S|2|0|0!".getBytes());
            } catch (IOException e) {
                toastMsg("Error");
            }
        }
        Log.d("STATE", "0");
        super.onDestroy();
    }

    @Override
    public void onClick(View view) {
        switch(view.getId()) {
            case R.id.btn_open_wing_left:
                 open_wing_left();
                 break;
            case R.id.btn_open_wing_both:
                open_wing_both();
                break;
            case R.id.btn_open_wing_right:
                open_wing_right();
                break;
            case R.id.btn_close_wing_left:
                close_wing_left();
                break;
            case R.id.btn_close_wing_both:
                close_wing_both();
                break;
            case R.id.btn_close_wing_right:
                close_wing_right();
                break;
            case R.id.btn_seat_left_up:
                seat_left_up();
                break;
            case R.id.btn_seat_middle_up:
                seat_middle_up();
                break;
            case R.id.btn_seat_right_up:
                seat_right_up();
                break;
            case R.id.btn_seat_left_down:
                seat_left_down();
                break;
            case R.id.btn_seat_middle_down:
                seat_middle_down();
                break;
            case R.id.btn_seat_right_down:
                seat_right_down();
                break;
            default:
                break;
        }
    }

    private void open_wing_left() {
        if (btSocket!=null) {
            try {
                btSocket.getOutputStream().write("W|1|1|0!".getBytes());
            } catch (IOException e) {
                toastMsg("Error");
            }
        }
    }

    private void open_wing_both() {
        if (btSocket!=null) {
            try {
                btSocket.getOutputStream().write("W|2|1|0!".getBytes());
            } catch (IOException e) {
                toastMsg("Error");
            }
        }
    }

    private void open_wing_right() {
        if (btSocket!=null) {
            try {
                btSocket.getOutputStream().write("W|0|1|0!".getBytes());
            } catch (IOException e) {
                toastMsg("Error");
            }
        }
    }

    private void close_wing_left() {
        if (btSocket!=null) {
            try {
                btSocket.getOutputStream().write("W|1|0|0!".getBytes());
            } catch (IOException e) {
                toastMsg("Error");
            }
        }
    }

    private void close_wing_both() {
        if (btSocket!=null) {
            try {
                btSocket.getOutputStream().write("W|2|0|0!".getBytes());
            } catch (IOException e) {
                toastMsg("Error");
            }
        }
    }

    private void close_wing_right() {
        if (btSocket!=null) {
            try {
                btSocket.getOutputStream().write("W|0|0|0!".getBytes());
            } catch (IOException e) {
                toastMsg("Error");
            }
        }
    }

    private void seat_left_up() {
        if (btSocket!=null) {
            try {
                btSocket.getOutputStream().write("C|2|1|0!".getBytes());
            } catch (IOException e) {
                toastMsg("Error");
            }
        }
    }

    private void seat_middle_up() {
        if (btSocket!=null) {
            try {
                btSocket.getOutputStream().write("C|1|1|0!".getBytes());
            } catch (IOException e) {
                toastMsg("Error");
            }
        }
    }

    private void seat_right_up() {
        if (btSocket!=null) {
            try {
                btSocket.getOutputStream().write("C|0|1|0!".getBytes());
            } catch (IOException e) {
                toastMsg("Error");
            }
        }
    }

    private void seat_left_down() {
        if (btSocket!=null) {
            try {
                btSocket.getOutputStream().write("C|2|0|0!".getBytes());
            } catch (IOException e) {
                toastMsg("Error");
            }
        }
    }

    private void seat_middle_down() {
        if (btSocket!=null) {
            try {
                btSocket.getOutputStream().write("C|1|0|0!".getBytes());
            } catch (IOException e) {
                toastMsg("Error");
            }
        }
    }

    private void seat_right_down() {
        if (btSocket!=null) {
            try {
                btSocket.getOutputStream().write("C|0|0|0!".getBytes());
            } catch (IOException e) {
                toastMsg("Error");
            }
        }
    }

    private void toastMsg(String s) {
        Toast.makeText(getApplicationContext(),s,Toast.LENGTH_LONG).show();
    }
}
