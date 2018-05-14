package com.example.nigelleong.quantum;
import android.support.design.widget.TabLayout;
import android.support.v4.app.Fragment;
import android.support.v4.app.FragmentManager;
import android.support.v4.app.FragmentPagerAdapter;
import android.support.v4.app.NavUtils;
import android.support.v4.view.ViewPager;
import android.support.v7.app.AppCompatActivity;

import android.bluetooth.BluetoothSocket;
import android.os.Bundle;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.MenuItem;
import android.view.View;
import android.widget.EditText;
import android.widget.TextView;
import android.widget.Toast;

import com.example.nigelleong.quantum.model.CommunicationModuleInterface;
import com.example.nigelleong.quantum.model.PrivateModuleInterface;
import com.example.nigelleong.quantum.model.EfficiencyModuleInferface;

import java.util.ArrayList;
import java.util.List;

public class demoLayoutController extends AppCompatActivity {

//    implements View.OnClickListener

    BluetoothSocket btSocket;
    BluetoothSocketHelper bluetoothSocketHelper;
    String destination;

    private TabLayout tabLayout;
    private ViewPager viewPager;

    EditText txtDestination;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_demolayout);

        bluetoothSocketHelper = ((BluetoothSocketHelper) getApplicationContext());
        btSocket = bluetoothSocketHelper.getBluetoothSocket();
        Log.d("demoLayoutController",btSocket.toString());

        viewPager = (ViewPager) findViewById(R.id.viewpager);
        ViewPagerAdapter adapter = new ViewPagerAdapter(getSupportFragmentManager());
        adapter.addFragment(new EfficiencyModuleInferface(), "Efficiency");
        adapter.addFragment(new CommunicationModuleInterface(), "Communication");
        adapter.addFragment(new PrivateModuleInterface(), "Private");
        viewPager.setAdapter(adapter);

        tabLayout = (TabLayout) findViewById(R.id.tabs);
        tabLayout.setupWithViewPager(viewPager);

        for (int i = 0; i < tabLayout.getTabCount(); i++) {
            TabLayout.Tab tab = tabLayout.getTabAt(i);
            tab.setCustomView(adapter.getTabView(i));
        }

        //Switch to DRIVING (state = 1);
//        if (btSocket!=null) {
//            try {
//                btSocket.getOutputStream().write("S|1|0|0!".getBytes());
//            } catch (IOException e) {
//                toastMsg("Error");
//            }
//        }
//        Log.d("STATE", "1");

    }

//    @Override
//    public void onClick(View view) {
//        switch(view.getId()) {
////            case R.id.:
////                robotGoUp();
////                break;
//            default:
//                break;
//        }
//    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        switch (item.getItemId()) {
            // Respond to the action bar's Up/Home button
            case android.R.id.home:
                NavUtils.navigateUpFromSameTask(this);
                return true;
        }
        return super.onOptionsItemSelected(item);
    }

    class ViewPagerAdapter extends FragmentPagerAdapter {
        private final List<Fragment> mFragmentList = new ArrayList<>();
        private final List<String> mFragmentTitleList = new ArrayList<>();

        String tabTitles[] = new String[] { "Efficiency", "Communication", "Private" };

        public ViewPagerAdapter(FragmentManager manager) {
            super(manager);
        }

        @Override
        public Fragment getItem(int position) {
            return mFragmentList.get(position);
        }

        @Override
        public int getCount() {
            return mFragmentList.size();
        }

        public void addFragment(Fragment fragment, String title) {
            mFragmentList.add(fragment);
            mFragmentTitleList.add(title);
        }

        @Override
        public CharSequence getPageTitle(int position) {
            return mFragmentTitleList.get(position);
        }


        public View getTabView(int position) {
            View tab = LayoutInflater.from(demoLayoutController.this).inflate(R.layout.custom_tab, null);
            TextView tv = (TextView) tab.findViewById(R.id.custom_text);
            tv.setText(tabTitles[position]);
            return tab;
        }
    }

    private void toastMsg(String s) {
        Toast.makeText(getApplicationContext(),s,Toast.LENGTH_LONG).show();
    }
}
