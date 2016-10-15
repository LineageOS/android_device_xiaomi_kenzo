/*
 * Copyright (C) 2015 The CyanogenMod Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package com.cyanogenmod.settings.doze;

import android.app.ActionBar;
import android.app.Activity;
import android.app.AlertDialog;
import android.app.Dialog;
import android.app.DialogFragment;
import android.content.Context;
import android.content.DialogInterface;
import android.content.SharedPreferences;
import android.os.Bundle;
import android.preference.Preference;
import android.preference.Preference.OnPreferenceChangeListener;
import android.preference.PreferenceActivity;
import android.preference.PreferenceScreen;
import android.preference.SwitchPreference;
import android.provider.Settings;
import android.view.Menu;
import android.view.MenuItem;

public class DozeSettings extends PreferenceActivity implements OnPreferenceChangeListener {

    private Context mContext;
    private SharedPreferences mPreferences;

    private SwitchPreference mAmbientDisplayPreference;
    private SwitchPreference mPickUpPreference;
    private SwitchPreference mHandwavePreference;
    private SwitchPreference mPocketPreference;

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        addPreferencesFromResource(R.xml.doze_settings);
        mContext = getApplicationContext();
        boolean dozeEnabled = Utils.isDozeEnabled(mContext);

        // get shared preference
        mPreferences = mContext.getSharedPreferences("doze_settings", Activity.MODE_PRIVATE);
        if (savedInstanceState == null && !mPreferences.getBoolean("first_help_shown", false)) {
            showHelp();
        }

        mAmbientDisplayPreference =
            (SwitchPreference) findPreference(Utils.AMBIENT_DISPLAY_KEY);
        // Read from DOZE_ENABLED secure setting
        mAmbientDisplayPreference.setChecked(dozeEnabled);
        mAmbientDisplayPreference.setOnPreferenceChangeListener(this);

        mPickUpPreference =
            (SwitchPreference) findPreference(Utils.PICK_UP_KEY);
        mPickUpPreference.setOnPreferenceChangeListener(this);

        mHandwavePreference =
            (SwitchPreference) findPreference(Utils.GESTURE_HAND_WAVE_KEY);
        mHandwavePreference.setOnPreferenceChangeListener(this);

        mPocketPreference =
            (SwitchPreference) findPreference(Utils.GESTURE_POCKET_KEY);
        mPocketPreference.setOnPreferenceChangeListener(this);

        final ActionBar actionBar = getActionBar();
        actionBar.setDisplayHomeAsUpEnabled(true);
    }

    @Override
    protected void onResume() {
        super.onResume();
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        if (item.getItemId() == android.R.id.home) {
            onBackPressed();
            return true;
        }
        return false;
    }

    @Override
    public boolean onPreferenceChange(Preference preference, Object newValue) {
        final String key = preference.getKey();
        final boolean value = (Boolean) newValue;
        if (Utils.AMBIENT_DISPLAY_KEY.equals(key)) {
            mAmbientDisplayPreference.setChecked(value);
            Utils.enableDoze(value, mContext);
            return true;
        } else if (Utils.PICK_UP_KEY.equals(key)) {
            mPickUpPreference.setChecked(value);
            Utils.startService(mContext);
            return true;
        } else if (Utils.GESTURE_HAND_WAVE_KEY.equals(key)) {
            mHandwavePreference.setChecked(value);
            Utils.startService(mContext);
            return true;
        } else if (Utils.GESTURE_POCKET_KEY.equals(key)) {
            mPocketPreference.setChecked(value);
            Utils.startService(mContext);
            return true;
        }
        return false;
    }

    public static class HelpDialogFragment extends DialogFragment {
        @Override
        public Dialog onCreateDialog(Bundle savedInstanceState) {
            return new AlertDialog.Builder(getActivity())
                    .setTitle(R.string.doze_settings_help_title)
                    .setMessage(R.string.doze_settings_help_text)
                    .setNegativeButton(R.string.dlg_ok, new DialogInterface.OnClickListener() {
                        @Override
                        public void onClick(DialogInterface dialog, int which) {
                            dialog.cancel();
                        }
                    })
                    .create();
        }

        @Override
        public void onCancel(DialogInterface dialog) {
            getActivity().getSharedPreferences("doze_settings", Activity.MODE_PRIVATE)
                    .edit()
                    .putBoolean("first_help_shown", true)
                    .commit();
        }
    }

    private void showHelp() {
        HelpDialogFragment fragment = new HelpDialogFragment();
        fragment.show(getFragmentManager(), "help_dialog");
    }
}
