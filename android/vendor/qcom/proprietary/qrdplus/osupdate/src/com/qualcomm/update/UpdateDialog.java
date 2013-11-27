/**
 * Copyright (c) 2011-2012, QUALCOMM Technologies Incorporated.
 * All Rights Reserved.
 * QUALCOMM Proprietary and Confidential.
 * Developed by QRD Engineering team.
 */

package com.qualcomm.update;

import android.app.Activity;
import android.app.AlertDialog;
import android.app.Dialog;
import android.content.ActivityNotFoundException;
import android.content.DialogInterface;
import android.content.DialogInterface.OnCancelListener;
import android.content.DialogInterface.OnClickListener;
import android.content.Intent;
import android.os.Bundle;
import android.widget.Toast;
//adupsfota
import android.content.Context;
import android.content.pm.PackageManager;
import android.content.pm.PackageInfo;
import android.content.pm.ResolveInfo;
import java.util.List;
//adupsfota

public class UpdateDialog extends Activity implements OnClickListener, OnCancelListener {

    private static final int DIALOG_CHOOSE = 1;

    private static final int INDEX_REMOTE = 0;

    private static final int INDEX_LOCAL = 1;

    private static final int REQUEST_CODE_FILE = 1;
    //adupsfota
    private boolean mApkExist = false;
    //adupsfota
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        this.showDialog(DIALOG_CHOOSE);
		//adupsfota
        if(isApkExist(this, "com.adups.fota") == true){
            mApkExist = true;
		}
		//adupsfota
    }
	
    //adupsfota start
    boolean isApkExist(Context ctx, String packageName){
		try{
			List<PackageInfo> packages = ctx.getPackageManager().getInstalledPackages(0);
			for(int i=0; i<packages.size(); i++){
				if(packages.get(i).packageName.equals(packageName)){
					return true;
				}
			}
		}catch(Exception e){
		//	Log.e("AdupsFota", "isApkExist error", e);
		}

		return false;
	}
	//adupsfota end
	
    private class ChooseFromListener implements OnClickListener {

        public void onClick(DialogInterface dialog, int which) {
            Intent intent = null;
            switch (which) {
                case INDEX_REMOTE:
                    //adupsfota
                    if(mApkExist == true){
                        intent = new Intent();
                        intent.setClassName("com.adups.fota", "com.adups.fota.GoogleOtaClient");   
                    }
                    else
                    {
                        intent = new Intent(UpdateDialog.this, RemoteActivity.class);
                    }
					//adupsfota
					//intent = new Intent(UpdateDialog.this, RemoteActivity.class);
                    startActivity(intent);
                    finish();
                    break;
                case INDEX_LOCAL:
                    try {
                        intent = new Intent("com.android.fileexplorer.action.FILE_SINGLE_SEL");
                        startActivityForResult(intent, REQUEST_CODE_FILE);
                        Toast.makeText(UpdateDialog.this, R.string.msg_pick_update,
                                Toast.LENGTH_SHORT).show();
                    } catch (ActivityNotFoundException e) {
                        Toast.makeText(UpdateDialog.this, R.string.msg_no_file_explore,
                                Toast.LENGTH_SHORT).show();
                    }
                    break;
            }
        }

    }

    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        if (resultCode == RESULT_OK) {
            switch (requestCode) {
                case REQUEST_CODE_FILE:
                    Intent intent = new Intent(InstallReceiver.ACTION_REBOOT);
                    intent.setData(data.getData());
                    startActivity(intent);
                    break;
            }
        }
        finish();
    }

    protected Dialog onCreateDialog(int id) {
        switch (id) {
            case DIALOG_CHOOSE:
                return new AlertDialog.Builder(this)
                        .setSingleChoiceItems(
                                new MenuAdapter(this, getResources().getStringArray(
                                        R.array.remote_entries), R.drawable.remote_update,
                                        R.drawable.local_update), 0, new ChooseFromListener())
                        .setTitle(R.string.title_choose_update_from).setOnCancelListener(this)
                        .setNegativeButton(android.R.string.cancel, this).create();
        }
        return null;
    }

    public void onClick(DialogInterface dialog, int which) {
        finish();
    }

    public void onCancel(DialogInterface dialog) {
        finish();
    }
}
