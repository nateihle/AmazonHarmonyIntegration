package com.microchip.spp_voice_control;


import android.app.Activity;
import android.os.Bundle;
import android.os.Handler;
import android.support.v4.app.Fragment;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.TextView;


public class DataRateFragment extends Fragment {
	

	private Handler mHandler;
	private TextView dataRateText;
	
	// Callback functions
    // These callbacks allow a fragment to call a function defined in its parent Activity.

    // This callback allows this fragment to send a string over Bluetooth, using its parent Activity's Bluetooth 
    // connection. This is used to send the user-entered string over Bluetooth when the user presses the Send button.
    public interface OnSendRequestListener {
        public void onSendRequest(String s);
    }
    OnSendRequestListener mSendRequestCallback;
    
    // Lifecycle callbacks
    // These are called by the system when starting, closing, or navigating through the app
    
    // Called when this fragment is attached to its container activity (when its tab is selected)
    @Override
    public void onAttach(Activity activity) {
        super.onAttach(activity);
        // This checks that the container activity (TabHostActivity) has implemented
        // the OnSendRequestListener interface. If not, it throws an exception. If it
        // has been implemented, sets mSendRequestCallback to the implementation in the
        // container activity.
//        try {
//            mSendRequestCallback = (OnSendRequestListener) activity;
//        } catch (ClassCastException e) {
//            throw new ClassCastException(activity.toString() + " must implement OnSendRequestListener");
//        }

        // Get the parent avtivity's handler
        // for using Toast to show hint message
        mHandler = ((TabHostActivity)activity).getHandler();
        return;
    }

    // Called when this fragment is created
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
    }
    
    
	@Override
	public View onCreateView(LayoutInflater inflater, ViewGroup container,
	        Bundle savedInstanceState)
	{
		
		ViewGroup rootView =  (ViewGroup) inflater.inflate(R.layout.fragment_datarate, container, false);
        dataRateText = (TextView)rootView.findViewById(R.id.dataRateId);
		return rootView;
	}
	
	
	public void updateDataRateText(String text)
	{
        dataRateText.setText(text);
	}
	
	

}
