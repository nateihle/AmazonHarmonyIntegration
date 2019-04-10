/*
 * Bluetooth Starter Kit Android app. For use with the Microchip Bluetooth 
 * Starter Kit DM320018
 *
 * Copyright (C) 2014  Microchip Technology Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
package com.mchp.android.PIC32_BTSK;

import android.app.AlertDialog;
import android.app.Dialog;
import android.content.DialogInterface;
import android.content.pm.PackageInfo;
import android.os.Bundle;
import android.support.v4.app.DialogFragment;
import com.mchp.android.PIC32_BTSK.R;

/*
 * This is the About dialog.
 */
public class AboutDialogFragment extends DialogFragment {
	private String mVersionName = "1.0";
    final String mAbout = "\n" +
    		"For use with Microchip Bluetooth Starter Kit DM320018" +
            "\n\n" +
    		"Copyright \u00a9 2014 Microchip Technology Inc. All Rights Reserved" +
            "\n\n" +
            "Visit www.microchip.com" +            
            "\n\n" +
            "License:" + 
            "\n" +
            "   GNU General Public License v3" +
            "\n" +
            "   www.gnu.org/licenses/" +
            "\n\n" +
    		"Credits:" +
            "\n" +
            "   The Android Open Source Project" +
            "\n" +
            "   GraphView: Jonas Gehring" +
            "\n" +
            "   ColorPicker: Sergey Margaritov & Daniel Nilsson" +
            "\n" +
            "   Android Holo Colors: Jérôme Van Der Linden" +            
    		"\n";
    
    
    
    public AboutDialogFragment(String versionName){
    	mVersionName = versionName;
    }
    
    @Override
    public Dialog onCreateDialog(Bundle savedInstanceState) {
        // Use the Builder class for convenient dialog construction
        AlertDialog.Builder builder = new AlertDialog.Builder(getActivity());
        
        String mTitle = this.getActivity().getApplicationContext().getString(R.string.app_name)
        					+ " " + mVersionName;
        builder.setTitle(mTitle);
        builder.setIcon(android.R.drawable.ic_dialog_info);
        builder.setMessage(mAbout)
               .setPositiveButton("OK", new DialogInterface.OnClickListener() {
                   public void onClick(DialogInterface dialog, int id) {
                   }
               });

        // Create the AlertDialog object and return it
        return builder.create();
    }
}
