package com.example.nigelleong.quantum;

import android.app.Activity;
import android.content.Context;
import android.view.View;
import android.widget.ImageView;
import android.widget.TextView;

import com.google.android.gms.maps.GoogleMap;
import com.google.android.gms.maps.model.Marker;

public class CustomInfoWindowGoogleMap implements GoogleMap.InfoWindowAdapter {

    private Context context;

    public CustomInfoWindowGoogleMap(Context context){
        this.context = context;
    }

    @Override
    public View getInfoWindow(Marker marker) {
        return null;
    }

    @Override
    public View getInfoContents(Marker marker) {
        View view = ((Activity)context).getLayoutInflater()
                .inflate(R.layout.map_information_window, null);

        TextView name = view.findViewById(R.id.name);
//        TextView details = view.findViewById(R.id.details);

        name.setText(marker.getTitle());
//        details.setText(marker.getSnippet());

//        InfoWindowData infoWindowData = (InfoWindowData) marker.getTag();

        return view;
    }
}