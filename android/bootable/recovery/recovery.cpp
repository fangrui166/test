/*
 * Copyright (C) 2007 The Android Open Source Project
 * Copyright (c) 2012 The Linux Foundation. All Rights Reserved.
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

#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <limits.h>
#include <linux/input.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/reboot.h>
#include <time.h>
#include <unistd.h>
#include <dirent.h>

#include "bootloader.h"
#include "common.h"
#include "cutils/properties.h"
#include "cutils/android_reboot.h"
#include "install.h"
#include "minui/minui.h"
#include "minzip/DirUtil.h"
#include "roots.h"
#include "ui.h"
#include "screen_ui.h"
#include "device.h"
#include "adb_install.h"
extern "C" {
#include "minadbd/adb.h"
}

//adupsfota start
#ifdef ADUPS_FOTA_SUPPORT
extern "C"{
extern void finish_adupsfota();
extern void adupsfota_install_finish(const char* path, int status);
extern int prepare_install_firmwave();
}
#endif
//adupsfota end

#define UI_REBOOT_TIMEOUT_SEC    5

struct selabel_handle *sehandle;

#include "deltaupdate_config.h"

static const deltaupdate_config_st DELTA_UPDATE_STATUS_DB[] = {
        {NO_DELTA_UPDATE, "IP_NO_UPDATE"},
        {START_DELTA_UPDATE, "IP_START_UPDATE"},
        {DELTA_UPDATE_IN_PROGRESS, "IP_PREVIOUS_UPDATE_IN_PROGRESS"},
        {DELTA_UPDATE_SUCCESSFUL, "IP_PREVIOUS_UPDATE_SUCCESSFUL"},
        {DELTA_UPDATE_FAILED, "IP_PREVIOUS_UPDATE_FAILED"}
};

static char diff_pkg_path_name[PATH_MAX];

static const struct option OPTIONS[] = {
  { "send_intent", required_argument, NULL, 's' },
  { "update_package", required_argument, NULL, 'u' },
  { "wipe_data", no_argument, NULL, 'w' },
  { "wipe_cache", no_argument, NULL, 'c' },
  { "show_text", no_argument, NULL, 't' },
  { "just_exit", no_argument, NULL, 'x' },
  { NULL, 0, NULL, 0 },
};

static const char *COMMAND_FILE = "/cache/recovery/command";
static const char *INTENT_FILE = "/cache/recovery/intent";
static const char *LOG_FILE = "/cache/recovery/log";
static const char *LAST_LOG_FILE = "/cache/recovery/last_log";
static const char *LAST_INSTALL_FILE = "/cache/recovery/last_install";
static const char *CACHE_ROOT = "/cache";
static const char *RADIO_DIR = "/sdcard/radio";
static const char *SDCARD_ROOT = "/sdcard";
static const char *INTERNAL_SDCARD_ROOT = "/sdcard1";
static const char *TEMPORARY_LOG_FILE = "/tmp/recovery.log";
static const char *SDCARD_LOG_DIR = "/sdcard/logs/recovery";
static const char *SDCARD1_LOG_DIR = "/sdcard1/logs/recovery";
static const char *TEMPORARY_INSTALL_FILE = "/tmp/last_install";
static const char *SIDELOAD_TEMP_DIR = "/tmp/sideload";
static const char *SYSFS_F_MASS_STORAGE = "/sys/devices/virtual/android_usb/android0/f_mass_storage/lun1/file";

RecoveryUI* ui = NULL;
Device* gDevice = NULL;
static int verify_signature = 1;
static void apply_pre_installed_apps();
#define BUFFER_SIZE 1024

static const char* MOUNT_STORAGE_ITEMS[] = {
    "../",
    "toggle USB mass storage",
    NULL
};
static char* ADVANCE_ITEMS[] = {
    "../",
    "dummp toggle signature",
    NULL
};

enum MountStorageItem {MOUNT_STORAGE_LAST_MENU, TOGGLE_MASS_STORAGE};
enum AdvanceItem {ADVANCE_LAST_MENU, TOGGLE_SIGNATURE_VERIFICATION};

/*
 * The recovery tool communicates with the main system through /cache files.
 *   /cache/recovery/command - INPUT - command line for tool, one arg per line
 *   /cache/recovery/log - OUTPUT - combined log file from recovery run(s)
 *   /cache/recovery/intent - OUTPUT - intent that was passed in
 *
 * The arguments which may be supplied in the recovery.command file:
 *   --send_intent=anystring - write the text out to recovery.intent
 *   --update_package=path - verify install an OTA package file
 *   --wipe_data - erase user data (and cache), then reboot
 *   --wipe_cache - wipe cache (but not user data), then reboot
 *   --set_encrypted_filesystem=on|off - enables / diasables encrypted fs
 *   --just_exit - do nothing; exit and reboot
 *
 * After completing, we remove /cache/recovery/command and reboot.
 * Arguments may also be supplied in the bootloader control block (BCB).
 * These important scenarios must be safely restartable at any point:
 *
 * FACTORY RESET
 * 1. user selects "factory reset"
 * 2. main system writes "--wipe_data" to /cache/recovery/command
 * 3. main system reboots into recovery
 * 4. get_args() writes BCB with "boot-recovery" and "--wipe_data"
 *    -- after this, rebooting will restart the erase --
 * 5. erase_volume() reformats /data
 * 6. erase_volume() reformats /cache
 * 7. finish_recovery() erases BCB
 *    -- after this, rebooting will restart the main system --
 * 8. main() calls reboot() to boot main system
 *
 * OTA INSTALL
 * 1. main system downloads OTA package to /cache/some-filename.zip
 * 2. main system writes "--update_package=/cache/some-filename.zip"
 * 3. main system reboots into recovery
 * 4. get_args() writes BCB with "boot-recovery" and "--update_package=..."
 *    -- after this, rebooting will attempt to reinstall the update --
 * 5. install_package() attempts to install the update
 *    NOTE: the package install must itself be restartable from any point
 * 6. finish_recovery() erases BCB
 *    -- after this, rebooting will (try to) restart the main system --
 * 7. ** if install failed **
 *    7a. prompt_and_wait() shows an error icon and waits for the user
 *    7b; the user reboots (pulling the battery, etc) into the main system
 * 8. main() calls maybe_install_firmware_update()
 *    ** if the update contained radio/hboot firmware **:
 *    8a. m_i_f_u() writes BCB with "boot-recovery" and "--wipe_cache"
 *        -- after this, rebooting will reformat cache & restart main system --
 *    8b. m_i_f_u() writes firmware image into raw cache partition
 *    8c. m_i_f_u() writes BCB with "update-radio/hboot" and "--wipe_cache"
 *        -- after this, rebooting will attempt to reinstall firmware --
 *    8d. bootloader tries to flash firmware
 *    8e. bootloader writes BCB with "boot-recovery" (keeping "--wipe_cache")
 *        -- after this, rebooting will reformat cache & restart main system --
 *    8f. erase_volume() reformats /cache
 *    8g. finish_recovery() erases BCB
 *        -- after this, rebooting will (try to) restart the main system --
 * 9. main() calls reboot() to boot main system
 */

static const int MAX_ARG_LENGTH = 4096;
static const int MAX_ARGS = 100;

// open a given path, mounting partitions as necessary
FILE*
fopen_path(const char *path, const char *mode) {
    if (ensure_path_mounted(path) != 0) {
        LOGE("Can't mount %s\n", path);
        return NULL;
    }

    // When writing, try to create the containing directory, if necessary.
    // Use generous permissions, the system (init.rc) will reset them.
    if (strchr("wa", mode[0])) dirCreateHierarchy(path, 0777, NULL, 1, sehandle);

    FILE *fp = fopen(path, mode);
    return fp;
}

// close a file, log an error if the error indicator is set
static void
check_and_fclose(FILE *fp, const char *name) {
    fflush(fp);
    if (ferror(fp)) LOGE("Error in %s\n(%s)\n", name, strerror(errno));
    fclose(fp);
}

// command line args come from, in decreasing precedence:
//   - the actual command line
//   - the bootloader control block (one per line, after "recovery")
//   - the contents of COMMAND_FILE (one per line)
static void
get_args(int *argc, char ***argv) {
    struct bootloader_message boot;
    memset(&boot, 0, sizeof(boot));
    get_bootloader_message(&boot);  // this may fail, leaving a zeroed structure

    if (boot.command[0] != 0 && boot.command[0] != 255) {
        LOGI("Boot command: %.*s\n", sizeof(boot.command), boot.command);
    }

    if (boot.status[0] != 0 && boot.status[0] != 255) {
        LOGI("Boot status: %.*s\n", sizeof(boot.status), boot.status);
    }

    // --- if arguments weren't supplied, look in the bootloader control block
    if (*argc <= 1) {
        boot.recovery[sizeof(boot.recovery) - 1] = '\0';  // Ensure termination
        const char *arg = strtok(boot.recovery, "\n");
        if (arg != NULL && !strcmp(arg, "recovery")) {
            *argv = (char **) malloc(sizeof(char *) * MAX_ARGS);
            (*argv)[0] = strdup(arg);
            for (*argc = 1; *argc < MAX_ARGS; ++*argc) {
                if ((arg = strtok(NULL, "\n")) == NULL) break;
                (*argv)[*argc] = strdup(arg);
            }
            LOGI("Got arguments from boot message\n");
        } else if (boot.recovery[0] != 0 && boot.recovery[0] != 255) {
            LOGE("Bad boot message\n\"%.20s\"\n", boot.recovery);
        }
    }

    // --- if that doesn't work, try the command file
    if (*argc <= 1) {
        FILE *fp = fopen_path(COMMAND_FILE, "r");
        if (fp != NULL) {
            char *argv0 = (*argv)[0];
            *argv = (char **) malloc(sizeof(char *) * MAX_ARGS);
            (*argv)[0] = argv0;  // use the same program name

            char buf[MAX_ARG_LENGTH];
            for (*argc = 1; *argc < MAX_ARGS; ++*argc) {
                if (!fgets(buf, sizeof(buf), fp)) break;
                (*argv)[*argc] = strdup(strtok(buf, "\r\n"));  // Strip newline.
            }

            check_and_fclose(fp, COMMAND_FILE);
            LOGI("Got arguments from %s\n", COMMAND_FILE);
        }
    }

    // --> write the arguments we have back into the bootloader control block
    // always boot into recovery after this (until finish_recovery() is called)
    strlcpy(boot.command, "boot-recovery", sizeof(boot.command));
    strlcpy(boot.recovery, "recovery\n", sizeof(boot.recovery));
    int i;
    for (i = 1; i < *argc; ++i) {
        strlcat(boot.recovery, (*argv)[i], sizeof(boot.recovery));
        strlcat(boot.recovery, "\n", sizeof(boot.recovery));
    }
    //adupsfota start
    #ifdef ADUPS_FOTA_SUPPORT
    boot.cp_recovery_step = 0;
    #endif
    //adupsfota end
    set_bootloader_message(&boot);
}

static void
set_sdcard_update_bootloader_message() {
    struct bootloader_message boot;
    memset(&boot, 0, sizeof(boot));
    strlcpy(boot.command, "boot-recovery", sizeof(boot.command));
    strlcpy(boot.recovery, "recovery\n", sizeof(boot.recovery));
    set_bootloader_message(&boot);
}

// How much of the temp log we have copied to the copy in cache.
static long tmplog_offset = 0;

static void
copy_log_file(const char* source, const char* destination, int append) {
    FILE *log = fopen_path(destination, append ? "a" : "w");
    if (log == NULL) {
        LOGE("Can't open %s\n", destination);
    } else {
        FILE *tmplog = fopen(source, "r");
        if (tmplog != NULL) {
            if (append) {
                fseek(tmplog, tmplog_offset, SEEK_SET);  // Since last write
            }
            char buf[4096];
            while (fgets(buf, sizeof(buf), tmplog)) fputs(buf, log);
            if (append) {
                tmplog_offset = ftell(tmplog);
            }
            check_and_fclose(tmplog, source);
        }
        check_and_fclose(log, destination);
    }
}


// clear the recovery command and prepare to boot a (hopefully working) system,
// copy our log file to cache as well (for the system to read), and
// record any intent we were asked to communicate back to the system.
// this function is idempotent: call it as many times as you like.
static void
finish_recovery(const char *send_intent) {
    // By this point, we're ready to return to the main system...
    printf("finish_recovery intent=%s\n",send_intent);
    if (send_intent != NULL) {
        FILE *fp = fopen_path(INTENT_FILE, "w");
        if (fp == NULL) {
            LOGE("Can't open %s\n", INTENT_FILE);
        } else {
            fputs(send_intent, fp);
            check_and_fclose(fp, INTENT_FILE);
        }
    }

    // Copy logs to cache so the system can find out what happened.
    copy_log_file(TEMPORARY_LOG_FILE, LOG_FILE, true);
    copy_log_file(TEMPORARY_LOG_FILE, LAST_LOG_FILE, false);
    copy_log_file(TEMPORARY_INSTALL_FILE, LAST_INSTALL_FILE, false);
    chmod(LOG_FILE, 0600);
    chown(LOG_FILE, 1000, 1000);   // system user
    chmod(LAST_LOG_FILE, 0640);
    chmod(LAST_INSTALL_FILE, 0644);

	// Copy logs to sdcard
	char path[PATH_MAX];
	time_t m_time;
	struct tm* ts;
	time(&m_time);
	ts = localtime(&m_time);

	int ret;
	// return 0 if success
	ret = ensure_path_mounted(SDCARD_LOG_DIR);
	if (ret != 0){
		ret = ensure_path_mounted(SDCARD1_LOG_DIR);
		if (ret == 0){
			snprintf(path, sizeof(path), "%s/%d%02d%02d_%02d%02d%02d.log", SDCARD1_LOG_DIR,
				ts->tm_year + 1900,	ts->tm_mon +1, ts->tm_mday, ts->tm_hour, ts->tm_min, ts->tm_sec);
			// copy_log_file is responsible to create the directory hierarchy
			copy_log_file(TEMPORARY_LOG_FILE, path, false);
		}
	} else {
		snprintf(path, sizeof(path), "%s/%d%02d%02d_%02d%02d%02d.log", SDCARD_LOG_DIR,
				ts->tm_year + 1900,	ts->tm_mon +1, ts->tm_mday, ts->tm_hour, ts->tm_min, ts->tm_sec);
		copy_log_file(TEMPORARY_LOG_FILE, path, false);
	}

    // Reset to normal system boot so recovery won't cycle indefinitely.
    struct bootloader_message boot;
    memset(&boot, 0, sizeof(boot));
    //adupsfota start
    #ifdef ADUPS_FOTA_SUPPORT
    boot.cp_recovery_step = 0;
    finish_adupsfota();
    #endif
    //adupsfota end
    set_bootloader_message(&boot);

    // Remove the command file, so recovery won't repeat indefinitely.
    if (ensure_path_mounted(COMMAND_FILE) != 0 ||
        (unlink(COMMAND_FILE) && errno != ENOENT)) {
        LOGW("Can't unlink %s\n", COMMAND_FILE);
    }

#ifdef ADUPS_FOTA_SUPPORT
#else
    ensure_path_unmounted(CACHE_ROOT);
#endif
    sync();  // For good measure.
}

static int
erase_volume(const char *volume) {
    ui->SetBackground(RecoveryUI::INSTALLING);
    ui->SetProgressType(RecoveryUI::INDETERMINATE);
    ui->Print("Formatting %s...\n", volume);

    ensure_path_unmounted(volume);

    if (strcmp(volume, "/cache") == 0) {
        // Any part of the log we'd copied to cache is now gone.
        // Reset the pointer so we copy from the beginning of the temp
        // log.
        tmplog_offset = 0;
    }

    return format_volume(volume);
}

static char*
copy_sideloaded_package(const char* original_path) {
  if (ensure_path_mounted(original_path) != 0) {
    LOGE("Can't mount %s\n", original_path);
    return NULL;
  }

  if (ensure_path_mounted(SIDELOAD_TEMP_DIR) != 0) {
    LOGE("Can't mount %s\n", SIDELOAD_TEMP_DIR);
    return NULL;
  }

  if (mkdir(SIDELOAD_TEMP_DIR, 0700) != 0) {
    if (errno != EEXIST) {
      LOGE("Can't mkdir %s (%s)\n", SIDELOAD_TEMP_DIR, strerror(errno));
      return NULL;
    }
  }

  // verify that SIDELOAD_TEMP_DIR is exactly what we expect: a
  // directory, owned by root, readable and writable only by root.
  struct stat st;
  if (stat(SIDELOAD_TEMP_DIR, &st) != 0) {
    LOGE("failed to stat %s (%s)\n", SIDELOAD_TEMP_DIR, strerror(errno));
    return NULL;
  }
  if (!S_ISDIR(st.st_mode)) {
    LOGE("%s isn't a directory\n", SIDELOAD_TEMP_DIR);
    return NULL;
  }
  if ((st.st_mode & 0777) != 0700) {
    LOGE("%s has perms %o\n", SIDELOAD_TEMP_DIR, st.st_mode);
    return NULL;
  }
  if (st.st_uid != 0) {
    LOGE("%s owned by %lu; not root\n", SIDELOAD_TEMP_DIR, st.st_uid);
    return NULL;
  }

  char copy_path[PATH_MAX];
  strcpy(copy_path, SIDELOAD_TEMP_DIR);
  strcat(copy_path, "/package.zip");

  char* buffer = (char*)malloc(BUFSIZ);
  if (buffer == NULL) {
    LOGE("Failed to allocate buffer\n");
    return NULL;
  }

  size_t read;
  FILE* fin = fopen(original_path, "rb");
  if (fin == NULL) {
    LOGE("Failed to open %s (%s)\n", original_path, strerror(errno));
    return NULL;
  }
  FILE* fout = fopen(copy_path, "wb");
  if (fout == NULL) {
    LOGE("Failed to open %s (%s)\n", copy_path, strerror(errno));
    return NULL;
  }

  while ((read = fread(buffer, 1, BUFSIZ, fin)) > 0) {
    if (fwrite(buffer, 1, read, fout) != read) {
      LOGE("Short write of %s (%s)\n", copy_path, strerror(errno));
      return NULL;
    }
  }

  free(buffer);

  if (fclose(fout) != 0) {
    LOGE("Failed to close %s (%s)\n", copy_path, strerror(errno));
    return NULL;
  }

  if (fclose(fin) != 0) {
    LOGE("Failed to close %s (%s)\n", original_path, strerror(errno));
    return NULL;
  }

  // "adb push" is happy to overwrite read-only files when it's
  // running as root, but we'll try anyway.
  if (chmod(copy_path, 0400) != 0) {
    LOGE("Failed to chmod %s (%s)\n", copy_path, strerror(errno));
    return NULL;
  }

  return strdup(copy_path);
}

static const char**
prepend_title(const char* const* headers) {
    const char* title[] = { "Android system recovery <"
                            EXPAND(RECOVERY_API_VERSION) "e>",
                            "",
                            NULL };

    // count the number of lines in our title, plus the
    // caller-provided headers.
    int count = 0;
    const char* const* p;
    for (p = title; *p; ++p, ++count);
    for (p = headers; *p; ++p, ++count);

    const char** new_headers = (const char**)malloc((count+1) * sizeof(char*));
    const char** h = new_headers;
    for (p = title; *p; ++p, ++h) *h = *p;
    for (p = headers; *p; ++p, ++h) *h = *p;
    *h = NULL;

    return new_headers;
}

static int
get_menu_selection(const char* const * headers, const char* const * items,
                   int menu_only, int initial_selection, Device* device) {
    // throw away keys pressed previously, so user doesn't
    // accidentally trigger menu items.
    ui->FlushKeys();

    ui->StartMenu(headers, items, initial_selection);
    int selected = initial_selection;
    int chosen_item = -1;

    while (chosen_item < 0) {
        int key = ui->WaitKey();
        int visible = ui->IsTextVisible();

        if (key == -1) {   // ui_wait_key() timed out
            if (ui->WasTextEverVisible()) {
                continue;
            } else {
                LOGI("timed out waiting for key input; rebooting.\n");
                ui->EndMenu();
                return 0; // XXX fixme
            }
        }

        int action = device->HandleMenuKey(key, visible);

        if (action < 0) {
            switch (action) {
                case Device::kHighlightUp:
                    --selected;
                    selected = ui->SelectMenu(selected);
                    break;
                case Device::kHighlightDown:
                    ++selected;
                    selected = ui->SelectMenu(selected);
                    break;
                case Device::kInvokeItem:
                    chosen_item = selected;
                    break;
                case Device::kNoAction:
                    break;
            }
        } else if (!menu_only) {
            chosen_item = action;
        }
    }

    ui->EndMenu();
    return chosen_item;
}

int force_install(const char **title)
{
    int need_free = 0;
    if (gDevice == NULL) {
        LOGE("Device hasn't been initialized yet\n");
        return 0;
    }
    if (title == NULL) {
        const char* headers[] = {
            "Modem version inconsistency found",
            "Confirm force install?",
            "THIS MAY BRICK YOUR PHONE.",
            "",
            NULL };
        title = prepend_title((const char**)headers);
        need_free = 1;
    }

    const char* items[] = { " No",
        " No",
        " No",
        " No",
        " No",
        " No",
        " No",
        " Yes -- force install",   // [7]
        " No",
        " No",
        " No",
        NULL };

    int chosen_item = get_menu_selection(title, items, 1, 0, gDevice);

    if (need_free)
        free(title);

    if (chosen_item != 7) {
        return 0;
    }

    return 1;
}

static int compare_string(const void* a, const void* b) {
    return strcmp(*(const char**)a, *(const char**)b);
}

static int
update_directory(const char* path, const char* unmount_when_done,
                 int* wipe_cache, Device* device) {
    ensure_path_mounted(path);

    const char* MENU_HEADERS[] = { "Choose a package to install:",
                                   path,
                                   "",
                                   NULL };
    DIR* d;
    struct dirent* de;
    d = opendir(path);
    if (d == NULL) {
        LOGE("error opening %s: %s\n", path, strerror(errno));
        if (unmount_when_done != NULL) {
            ensure_path_unmounted(unmount_when_done);
        }
        return 0;
    }

    const char** headers = prepend_title(MENU_HEADERS);

    int d_size = 0;
    int d_alloc = 10;
    char** dirs = (char**)malloc(d_alloc * sizeof(char*));
    int z_size = 1;
    int z_alloc = 10;
    char** zips = (char**)malloc(z_alloc * sizeof(char*));
    zips[0] = strdup("../");

    while ((de = readdir(d)) != NULL) {
        int name_len = strlen(de->d_name);

        if (de->d_type == DT_DIR) {
            // skip "." and ".." entries
            if (name_len == 1 && de->d_name[0] == '.') continue;
            if (name_len == 2 && de->d_name[0] == '.' &&
                de->d_name[1] == '.') continue;

            if (d_size >= d_alloc) {
                d_alloc *= 2;
                dirs = (char**)realloc(dirs, d_alloc * sizeof(char*));
            }
            dirs[d_size] = (char*)malloc(name_len + 2);
            strcpy(dirs[d_size], de->d_name);
            dirs[d_size][name_len] = '/';
            dirs[d_size][name_len+1] = '\0';
            ++d_size;
        } else if (de->d_type == DT_REG &&
                   name_len >= 4 &&
                   strncasecmp(de->d_name + (name_len-4), ".zip", 4) == 0) {
            if (z_size >= z_alloc) {
                z_alloc *= 2;
                zips = (char**)realloc(zips, z_alloc * sizeof(char*));
            }
            zips[z_size++] = strdup(de->d_name);
        }
    }
    closedir(d);

    qsort(dirs, d_size, sizeof(char*), compare_string);
    qsort(zips, z_size, sizeof(char*), compare_string);

    // append dirs to the zips list
    if (d_size + z_size + 1 > z_alloc) {
        z_alloc = d_size + z_size + 1;
        zips = (char**)realloc(zips, z_alloc * sizeof(char*));
    }
    memcpy(zips + z_size, dirs, d_size * sizeof(char*));
    free(dirs);
    z_size += d_size;
    zips[z_size] = NULL;

    int result;
    int chosen_item = 0;
    do {
        chosen_item = get_menu_selection(headers, zips, 1, chosen_item, device);

        char* item = zips[chosen_item];
        int item_len = strlen(item);
        if (chosen_item == 0) {          // item 0 is always "../"
            // go up but continue browsing (if the caller is update_directory)
            result = -1;
            break;
        } else if (item[item_len-1] == '/') {
            // recurse down into a subdirectory
            char new_path[PATH_MAX];
            strlcpy(new_path, path, PATH_MAX);
            strlcat(new_path, "/", PATH_MAX);
            strlcat(new_path, item, PATH_MAX);
            new_path[strlen(new_path)-1] = '\0';  // truncate the trailing '/'
            result = update_directory(new_path, unmount_when_done, wipe_cache, device);
            if (result >= 0) break;
        } else {
            // selected a zip file:  attempt to install it, and return
            // the status to the caller.
            char new_path[PATH_MAX];
            strlcpy(new_path, path, PATH_MAX);
            strlcat(new_path, "/", PATH_MAX);
            strlcat(new_path, item, PATH_MAX);

            ui->Print("\n-- Install %s ...\n", path);
            set_sdcard_update_bootloader_message();
            char* copy = copy_sideloaded_package(new_path);
            // create radio folder for QCOM radio image update
            mkdir(RADIO_DIR,777);
            // do not unmount SD card here itself. It's unmounted at the end.
            //
            //if (unmount_when_done != NULL) {
            //    ensure_path_unmounted(unmount_when_done);
            //}
            if (copy) {
                result = install_package(copy, wipe_cache, TEMPORARY_INSTALL_FILE, verify_signature);
                free(copy);
            } else {
                result = INSTALL_ERROR;
            }
            break;
        }
    } while (true);

    int i;
    for (i = 0; i < z_size; ++i) free(zips[i]);
    free(zips);
    free(headers);

    if (unmount_when_done != NULL) {
        ensure_path_unmounted(unmount_when_done);
    }
    return result;
}

static void
wipe_data(int confirm, Device* device) {
    if (confirm) {
        static const char** title_headers = NULL;

        if (title_headers == NULL) {
            const char* headers[] = { "Confirm wipe of all user data?",
                                      "  THIS CAN NOT BE UNDONE.",
                                      "",
                                      NULL };
            title_headers = prepend_title((const char**)headers);
        }

        const char* items[] = { " No",
                                " No",
                                " No",
                                " No",
                                " No",
                                " No",
                                " No",
                                " Yes -- delete all user data",   // [7]
                                " No",
                                " No",
                                " No",
                                NULL };

        int chosen_item = get_menu_selection(title_headers, items, 1, 0, device);
        if (chosen_item != 7) {
            return;
        }
    }

    ui->Print("\n-- Wiping data...\n");
    device->WipeData();
    erase_volume("/data");
    erase_volume("/cache");
    ui->Print("Data wipe complete.\n");
    apply_pre_installed_apps();
}

static void copy_file(char* src, char* dst)
{
    int src_fd,dst_fd;
    int bytes_read,bytes_write;
    char buffer[BUFFER_SIZE];
    char *ptr;
    if((src_fd = open(src,O_RDONLY)) == -1){
        ui->Print("\n -- Open %s error !\n",src);
    }
    if((dst_fd = open(dst,O_WRONLY|O_CREAT,0666)) == -1){
        ui->Print("\n --open  %s error !\n",dst);
    }
    while(bytes_read = read(src_fd,buffer,BUFFER_SIZE))
    {
        if((bytes_read == -1)&&(errno != EINTR))
        {
            ui->Print("\n -- read error ! \n");
            break;
        }
        else if(bytes_read > 0)
        {
            ptr=buffer;
            while(bytes_write = write(dst_fd,ptr,bytes_read))
            {
                if((bytes_write == -1)&&(errno != EINTR))
                {
                    ui->Print("\n -- write error ! \n");
                    break;
                }
                else if(bytes_write == bytes_read)
                    break;
                else if(bytes_write > 0)
                {
                   ptr+=bytes_write;
                   bytes_read -= bytes_write;
                }
            }
            if(bytes_write == -1)
            {
                ui->Print("\n -- write error! \n");
                break;
            }
        }
    }
    close(src_fd);
    close(dst_fd);
    return;
}
static void apply_pre_installed_apps()
{
   //mount system and data
   const char* str_system = "/system";
   const char* str_data = "/data";
   if(ensure_path_mounted(str_system) != 0 ){
       ui->Print("\n-- system mounted fail.\n");
   }
   if(ensure_path_mounted(str_data) != 0){
       ui->Print("\n-- data mounted fail.\n");
   }
   //copy /system/pre-install/ to /data/app/
   //mkdir /data/app/
   //open system/pre-install
    struct dirent **namelist;
    int file_num = scandir("/system/pre-install", &namelist, 0, 0);
    if (file_num < 0){
        ;
    }
    else{
        int ret = mkdir("/data/app",S_IRWXU|S_IRWXG|S_IRWXO);
        if(ret != 0){
               ui->Print("\n-- mkdir /data/app/ fail. err is %d\n",ret);
        }
        int i =0;
        for(;i < file_num;i++){
            char src_name[100]="/system/pre-install/";
            char dst_name[100]="/data/app/";
            if(strstr(namelist[i]->d_name,".apk")){
                strcat(src_name,namelist[i]->d_name);
                strcat(dst_name,namelist[i]->d_name);
                //copy /system/pre-install/xx.apk to /data/app/xx.apk
                copy_file(src_name,dst_name);
                chmod(dst_name,0666);
            }
            free(namelist[i]);
        }
        free(namelist);
    }
    if(ensure_path_unmounted(str_system) != 0 ){
       ui->Print("\n-- system unmounted fail.\n");
    }
    if(ensure_path_unmounted(str_data) != 0){
       ui->Print("\n-- data unmounted fail.\n");
    }
}
static void
prompt_and_wait(Device* device) {
    const char* const* headers = prepend_title(device->GetMenuHeaders());

    for (;;) {
        finish_recovery(NULL);
        ui->SetProgressType(RecoveryUI::EMPTY);

        int chosen_item = get_menu_selection(headers, device->GetMenuItems(), 0, 0, device);

        // device-specific code may take some action here.  It may
        // return one of the core actions handled in the switch
        // statement below.
        chosen_item = device->InvokeMenuItem(chosen_item);

        int status;
        int wipe_cache;
        const char *sdroot;
        switch (chosen_item) {
            case Device::REBOOT:
                return;

            case Device::WIPE_DATA:
                wipe_data(ui->IsTextVisible(), device);
                if (!ui->IsTextVisible()) return;
                break;

            case Device::WIPE_CACHE:
                ui->Print("\n-- Wiping cache...\n");
                erase_volume("/cache");
                ui->Print("Cache wipe complete.\n");
                if (!ui->IsTextVisible()) return;
                break;

            case Device::APPLY_EXT:
            case Device::APPLY_INTERNAL:
                // Some packages expect /cache to be mounted (eg,
                // standard incremental packages expect to use /cache
                // as scratch space).
                sdroot = chosen_item == Device::APPLY_EXT ? SDCARD_ROOT : INTERNAL_SDCARD_ROOT;
                ensure_path_mounted(CACHE_ROOT);
                status = update_directory(sdroot, sdroot, &wipe_cache, device);
                if (status == INSTALL_SUCCESS && wipe_cache) {
                    ui->Print("\n-- Wiping cache (at package request)...\n");
                    if (erase_volume("/cache")) {
                        ui->Print("Cache wipe failed.\n");
                    } else {
                        ui->Print("Cache wipe complete.\n");
                    }
                }
                if (status >= 0) {
                    if (status != INSTALL_SUCCESS) {
                        ui->SetBackground(RecoveryUI::ERROR);
                        ui->Print("Installation aborted.\n");
                    } else if (!ui->IsTextVisible()) {
                        return;  // reboot if logs aren't visible
                    } else {
                        ui->Print("\nInstall from sdcard complete.\n");
                    }
                }
                break;

            case Device::APPLY_CACHE:
                // Don't unmount cache at the end of this.
                status = update_directory(CACHE_ROOT, NULL, &wipe_cache, device);
                if (status == INSTALL_SUCCESS && wipe_cache) {
                    ui->Print("\n-- Wiping cache (at package request)...\n");
                    if (erase_volume("/cache")) {
                        ui->Print("Cache wipe failed.\n");
                    } else {
                        ui->Print("Cache wipe complete.\n");
                    }
                }
                if (status >= 0) {
                    if (status != INSTALL_SUCCESS) {
                        ui->SetBackground(RecoveryUI::ERROR);
                        ui->Print("Installation aborted.\n");
                    } else if (!ui->IsTextVisible()) {
                        return;  // reboot if logs aren't visible
                    } else {
                        ui->Print("\nInstall from cache complete.\n");
                    }
                }
                break;

            case Device::APPLY_ADB_SIDELOAD:
                ensure_path_mounted(CACHE_ROOT);
                status = apply_from_adb(ui, &wipe_cache, TEMPORARY_INSTALL_FILE);
                if (status >= 0) {
                    if (status != INSTALL_SUCCESS) {
                        ui->SetBackground(RecoveryUI::ERROR);
                        ui->Print("Installation aborted.\n");
                    } else if (!ui->IsTextVisible()) {
                        return;  // reboot if logs aren't visible
                    } else {
                        ui->Print("\nInstall from ADB complete.\n");
                    }
                }
                break;
            case Device::MOUNT_STORAGE:
                status = ensure_path_unmounted(SDCARD_ROOT);
                if (status != 0)
                    break;
                for(;;) {
                    int ms_item = get_menu_selection(headers, MOUNT_STORAGE_ITEMS, 0, 0, device);
                    int fd;

                    fd = open(SYSFS_F_MASS_STORAGE, O_RDWR);
                    if (fd < 0) {
                        ui->Print("Open %s failed. (%s)\n", SYSFS_F_MASS_STORAGE, strerror(errno));
                        break;
                    }

                    // TODO: more functions should be added here
                    if (ms_item == TOGGLE_MASS_STORAGE) {
                        char buf[PATH_MAX];
                        int len;
                        char state[50];

                        property_get("sys.usb.state", state, "");
                        if (strcmp(state, "mass_storage,adb"))
                            property_set("sys.usb.config", "mass_storage,adb");
                        len = read(fd, buf, sizeof(buf));
                        /* There is other storage */
                        if (len > 0) {
                            write(fd, "", 1);
                        } else {
                            Volume* v = volume_for_path(SDCARD_ROOT);
                            if(v->device != NULL) {
                                write(fd, v->device, strlen(v->device) + 1);
                            }
                        }

                    } else {
                        // Disable mass storage to prevent accessing by both PC & device
                        property_set("sys.usb.config", "diag,adb");
                        write(fd, "", 1);
                        close(fd);
                        break;
                    }
                }
                break;
            case Device::ADVANCE:
                int adv_item = 0;
                char menu[40];
                for(;;) {
                    LOGI("verify_signature=%d\n", verify_signature);
                    snprintf(menu, sizeof(menu), "%s signature verification",
                            verify_signature ? "enable":"disable");
                    ADVANCE_ITEMS[TOGGLE_SIGNATURE_VERIFICATION] = menu;
                    adv_item = get_menu_selection(headers, ADVANCE_ITEMS, 0, adv_item, device);
                    if (adv_item == TOGGLE_SIGNATURE_VERIFICATION) {
                        verify_signature = verify_signature ? 0:1;
                    } else if (adv_item == ADVANCE_LAST_MENU){
                        break;
                    }
                }
                break;

        }
    }
}

static void
print_property(const char *key, const char *name, void *cookie) {
    printf("%s=%s\n", key, name);
}

static int write_file(const char *path, const char *value)
{
	int fd, ret, len;

	fd = open(path, O_WRONLY|O_CREAT, 0622);

	if (fd < 0)
		return -errno;

	len = strlen(value);

	do {
		ret = write(fd, value, len);
	} while (ret < 0 && errno == EINTR);

	close(fd);
	if (ret < 0) {
		return -errno;
	} else {
		return 0;
	}
}

static const char* skip_whitespaces(const char* ptr)
{
    while(*ptr != '\0')
    {
        if(*ptr == 0x09 /* Tab */
            || *ptr == 0x0A /* LF */
            || *ptr == 0x0D /* CR */
            || *ptr == 0x20 /* SP */
            )
            ptr++;
        else
            return ptr;
    }
    return NULL;
}

static char* trim_whitespace(char *str)
{
    char *end;

    // Trim leading space
    while(isspace(*str)) str++;

    if(*str == 0)
        return str;

    // Trim trailing space
    end = str + strlen(str) - 1;
    while(end > str && isspace(*end)) end--;

    // Write new null terminator
    *(end+1) = 0;

    return str;
}

static int deltaupdate_pkg_location(char* diff_pkg_path_name)
{
    static char line[MAX_STRING_LEN];
    struct stat status;
    FILE* fp;
    char *tmp_str, *saveptr;
    bool found = false;
    int i = 0;

    while (i++ < 3)
    {
        LOGI("fopen_path %d %s\n", i, FOTA_PROP_FILE);
        sleep(1);
        fp = fopen_path(FOTA_PROP_FILE, "r");
        if (fp)
            break;
    }

    if (fp == NULL)
    {
        LOGI("Failed to open %s, use default pkg location:%s\n",
             FOTA_PROP_FILE, DEFAULT_PKG_LOCATION);
        strlcpy(diff_pkg_path_name, DEFAULT_PKG_LOCATION, PATH_MAX);
    }
    else
    {
        while(fgets(line, MAX_STRING_LEN, fp)!=NULL)
        {
            tmp_str = strtok_r(line, "=", &saveptr);
            if(strcmp(tmp_str, PKG_LOCATION_STRING_NAME) == 0)
            {
               tmp_str = strtok_r(NULL, "=\n", &saveptr);
               strlcpy(diff_pkg_path_name, tmp_str, PATH_MAX);
               diff_pkg_path_name = trim_whitespace(diff_pkg_path_name);
               LOGI("Package location: %s\r\n", diff_pkg_path_name);
               found = true;
               break;
            }
        }
        if (!found)
        {
            LOGI("Package location is not defined in %s. Use default location: %s\n",
            FOTA_PROP_FILE, DEFAULT_PKG_LOCATION );
            strlcpy(diff_pkg_path_name, DEFAULT_PKG_LOCATION, PATH_MAX);
        }
        fclose(fp);
    }

    if (ensure_path_mounted(diff_pkg_path_name) != 0) {
        LOGI("Cannot mount %s\n", diff_pkg_path_name);
        return -1;
    }

    strlcat(diff_pkg_path_name, "/", PATH_MAX);
    strlcat(diff_pkg_path_name, DIFF_PACKAGE_NAME, PATH_MAX);

    if (access(diff_pkg_path_name, F_OK) != 0) {
        LOGI("Delta package does not exist %s\n", diff_pkg_path_name);
        return -1;
    }

    LOGI("Delta package path name: %s\n", diff_pkg_path_name);

    return 0;
}

static int get_deltaupdate_recoverycount(void)
{
    FILE* f;
    int len, num;
    char* buf;

    f = fopen_path(NUM_OF_RECOVERY, "r");
    if(f == NULL)
    {
       LOGI("Error opening recovery count file. Ignore.\n");
       return 0;
    }

    fseek(f, 0, SEEK_END);
    len = ftell(f);
    buf = (char *) malloc(len+1);
    if (buf == NULL) {
        LOGI("Failed to allocate buffer\n");
        return 0;
    }
    memset(buf,0x0,len+1);
    fseek(f, 0, SEEK_SET);
    fread(buf, sizeof(char), len, f);
    check_and_fclose(f,NUM_OF_RECOVERY);

    if ((buf = strstr((const char*)buf, "numRecovery")) == NULL)
    {
        LOGI("Recovery count string doesn't match.Ignore.\n");
    }
    else
    {
        buf += 11;
        if ((buf = strstr((const char*)buf, "=")) == NULL)
        {
            LOGI("Invalid recovery count value. Ignore.\n");
        }
        else
        {
            buf += 1;
            buf = (char*)skip_whitespaces((const char*)buf);
            num = atoi((const char*)buf);
            return num;
        }
    }
    return 0;
}

static int get_deltaupdate_status(void)
{
    FILE* f;
    int len;
    char* buf;
    int i, num_index;

    LOGI("Checking delta update status...\n");

    f = fopen_path(DELTA_UPDATE_STATUS_FILE, "r");
    if(f == NULL)
    {
       LOGI("fopen error(%s)\n",DELTA_UPDATE_STATUS_FILE);
       return -1;
    }

    fseek(f, 0, SEEK_END);
    len = ftell(f);
    buf = (char *) malloc(len+1);
    if (buf == NULL) {
        LOGI("Failed to allocate buffer\n");
        return -1;
    }
    memset(buf,0x0,len+1);
    fseek(f, 0, SEEK_SET);
    fread(buf, sizeof(char), len, f);
    check_and_fclose(f,DELTA_UPDATE_STATUS_FILE);

    num_index = sizeof(DELTA_UPDATE_STATUS_DB)/sizeof(deltaupdate_config_st);

    for (i = 0; i < num_index; i++)
    {
        if (strstr((const char*)buf, DELTA_UPDATE_STATUS_DB[i].str)!=NULL)
        {
            return DELTA_UPDATE_STATUS_DB[i].idx;
        }
    }

    LOGI("NO UPDATE SET\n");
    return NO_DELTA_UPDATE;
}

static int set_deltaupdate_status(int status, int error_code)
{
    FILE* f;
    char strbuf[64];

    LOGI("Setting delta update status...\n");

    f = fopen_path(DELTA_UPDATE_STATUS_FILE, "w");
    if(f == NULL)
    {
       LOGI("fopen error(%s)\n",DELTA_UPDATE_STATUS_FILE);
       return -1;
    }

    switch(status)
    {
        case START_DELTA_UPDATE:
        case DELTA_UPDATE_IN_PROGRESS:
        case DELTA_UPDATE_SUCCESSFUL:
        case DELTA_UPDATE_FAILED:
            if ((snprintf(strbuf, sizeof(strbuf), "%s %d", DELTA_UPDATE_STATUS_DB[status].str, error_code)) >= sizeof(strbuf)) {
               LOGI("Output Truncated while setting error code\n");
            }
            fwrite(strbuf, sizeof(char), strlen(strbuf), f);
            break;
        default:
            if ((snprintf(strbuf, sizeof(strbuf), "DELTA_NO_UPDATE %d", error_code)) >= sizeof(strbuf)) {
               LOGI("Output Truncated while setting error code\n");
            }
            fwrite(strbuf, sizeof(char), strlen(strbuf), f);
            break;
    }

    LOGI("Delta update status is set to (%s)\n",strbuf);
    check_and_fclose(f,DELTA_UPDATE_STATUS_FILE);
    return 0;
}

static void set_deltaupdate_recovery_bootmessage(void)
{
    struct bootloader_message boot;
    memset(&boot, 0, sizeof(boot));

    LOGI("Setting recovery boot...\n");

    if(MAX_NUM_UPDATE_RECOVERY > get_deltaupdate_recoverycount())
    {
       strlcpy(boot.command, "boot-recovery", sizeof(boot.command));
       strlcpy(boot.recovery, "recovery\n", sizeof(boot.recovery));
    }
    else
    {
       LOGI("Recovery mode reached maximum retry. Clear boot message.\n");
    }
    set_bootloader_message(&boot);

    LOGI("boot.command=%s\n",boot.command);
    LOGI("boot.recovery=%s\n",boot.recovery);
}

static void reset_deltaupdate_recovery_bootmessage(void)
{
    struct bootloader_message boot;

    memset(&boot, 0, sizeof(boot));

    LOGI("Resetting recovery boot...\n");

    set_bootloader_message(&boot);

    LOGI("boot.command=%s\n",boot.command);
    LOGI("boot.recovery=%s\n",boot.recovery);
}

static void increment_deltaupdate_recoverycount(void)
{
    FILE* f;
    int num;
    char numbuf[8];
    char strbuf[32];

    num = get_deltaupdate_recoverycount();
    num += 1;
    snprintf(numbuf, sizeof(numbuf), "%d", num);

    memset(strbuf,0x0,sizeof(strbuf));
    strlcpy(strbuf,"numRecovery=",sizeof(strbuf));
    strlcat(strbuf,numbuf,sizeof(strbuf));

    f = fopen_path(NUM_OF_RECOVERY, "w");
    if(f == NULL)
    {
       LOGI("Error Creating file %s\n",NUM_OF_RECOVERY);
       return;
    }
    fwrite(strbuf, sizeof(char), strlen(strbuf), f);
    check_and_fclose(f, NUM_OF_RECOVERY);
}

static int remove_tempfiles(char* diff_pkg_path_name)
{
   if (unlink(diff_pkg_path_name) && errno != ENOENT) {
       LOGI("Cannot unlink %s\n", diff_pkg_path_name);
       return -1;
   }
   if (unlink(NUM_OF_RECOVERY) && errno != ENOENT) {
       LOGI("Cannot unlink %s\n", NUM_OF_RECOVERY);
       return -1;
   }
   if (unlink(RADIO_DIFF_OUTPUT) && errno != ENOENT) {
       LOGI("Cannot unlink %s\n", RADIO_DIFF_OUTPUT);
       return -1;
   }
   return 0;
}

static int read_buildprop(char **ver)
{
    FILE* b_fp;
    char line[MAX_STRING_LEN];
    char *tmpStr, *saveptr;

    LOGI("read_buildprop.\n");

    b_fp = fopen_path(BUILD_PROP_FILE, "r");
    if(!b_fp)
        return -1;

    while(fgets(line, sizeof(line), b_fp))
    {
        tmpStr = strtok_r(line, "=", &saveptr);
        if(strcmp(tmpStr, BUILD_PROP_NAME) == 0)
        {
           tmpStr = strtok_r(NULL, "=", &saveptr);
           strlcpy(*ver, tmpStr, MAX_STRING_LEN);
           fclose(b_fp);
           return 0;
        }
    }
    fclose(b_fp);
    return -1;

}

static char *delta_update_replace_str(char *str, char *org, char *rep)
{
    static char buffer[MAX_STRING_LEN];
    char *p;

    if(!(p = strstr(str, org)))
       return str;

    if ((strlcpy(buffer, str, MAX_STRING_LEN)) >= MAX_STRING_LEN) {
        LOGI("Version Update string truncated\n");
        return NULL;
    }
    buffer[p-str] = '\0';

    strlcat(buffer, rep, MAX_STRING_LEN);
    strlcat(buffer, p + strlen(org), MAX_STRING_LEN);

    return buffer;
}

static int update_fotapropver(char *ver)
{
    int size;
    FILE *b_fp;
    char *buff;
    char *newbuff;
    char *orgstr=NULL;
    char newstr[MAX_STRING_LEN];
    char line[MAX_STRING_LEN];

    LOGI("update_ver:%s\r\n",ver);

    b_fp = fopen_path(FOTA_PROP_FILE, "r");
    if(!b_fp)
       return -1;

    //Read Old Version
    while(fgets(line, sizeof(line), b_fp))
    {
        orgstr = strstr(line, VERSION_STRING_NAME);
        if(orgstr)
        {
           break;
        }
    }

    if(orgstr == NULL)
    {
       LOGI("No firmware property.\r\n");
       return -1;
    }

    //Build New Version
    snprintf(newstr, MAX_STRING_LEN, "%s=%s",VERSION_STRING_NAME, ver);

    //Read Org File
    fseek(b_fp, 0, SEEK_END);
    size = ftell(b_fp);
    buff = (char*)malloc(size+1);
    if (buff == NULL) {
        LOGI("Failed to allocate buffer\n");
        return -1;
    }
    memset(buff, 0x0, size);

    //Update Version
    fseek(b_fp, 0, SEEK_SET);
    fread(buff, sizeof(char), size, b_fp);
    fclose(b_fp);
    buff[size] = '\0';
    newbuff = delta_update_replace_str(buff, orgstr, newstr);

    if (newbuff) {
        b_fp = fopen_path(FOTA_PROP_FILE, "w+");
        fwrite(newbuff, sizeof(char), strlen(newbuff), b_fp);
        fclose(b_fp);
        return 0;
    }

    return -1;
}

static int update_fotaprop(void)
{
    int ret;
    char *ver;

    ui->Print("update_fotaprop.\n");

    ver = (char *)malloc(MAX_STRING_LEN);
    if (ver == NULL) {
        LOGI("Failed to allocate buffer\n");
        return 0;
    }
    memset(ver, 0x0, MAX_STRING_LEN);
    ret = read_buildprop(&ver);
    if(ret != 0)
    {
       LOGI("Failed reading build version.\n");
       return -1;
    }
    LOGI("Found build version:%s\n",ver);

    ret = update_fotapropver(ver);
    if(ret != 0)
    {
       LOGI("Failed update version.\n");
       return -1;
    }

    return 0;
}

int start_deltaupdate(char* diff_pkg_path_name)
{
    int status;
    int wipe_cache = 0;
    int ret = 0;

    LOGI("Start delta update...\n");

    set_deltaupdate_recovery_bootmessage();

    status = install_package(diff_pkg_path_name, &wipe_cache, TEMPORARY_INSTALL_FILE, verify_signature);

    if (status != INSTALL_SUCCESS)
    {
        ui->SetBackground(RecoveryUI::ERROR);
        ui->Print("Delta update failed.\n");
        finish_recovery("--send_intent=DELTA_UPDATE_FAILED");
        set_deltaupdate_status(DELTA_UPDATE_FAILED, DELTA_UPDATE_FAILED_410);
        if (reset_fota_cookie())
            LOGE("Failed to reset FOTA cookie\n");
        return -1;
    }

    // modem update starts only if android update is successful
    status = start_delta_modemupdate(diff_pkg_path_name);
    if (reset_fota_cookie()) {
        LOGE("Failed to reset FOTA cookie\n");
        ret = -1;
    }

    // modem update is complete. Handle update result.
    if (status != DELTA_UPDATE_SUCCESS_200)
    {
        ui->SetBackground(RecoveryUI::ERROR);
        ui->Print("Delta update failed(%d)\n",status);
        finish_recovery("--send_intent=DELTA_UPDATE_FAILED");
        set_deltaupdate_status(DELTA_UPDATE_FAILED, DELTA_UPDATE_FAILED_410);
        return -1;
    }

    finish_recovery("--send_intent=DELTA_UPDATE_SUCCESSFUL");
    set_deltaupdate_status(DELTA_UPDATE_SUCCESSFUL, DELTA_UPDATE_SUCCESS_200);

    ui->Print("\nAndroid Delta Update Completed \n");
    // Remove all temp files
    remove_tempfiles(diff_pkg_path_name);
    update_fotaprop();
    return ret;
}

/* FOTA(Delta Update) INSTALL
 * 1. main system downloads delta update package to location specified in
 *    FOTA_PROP_FILE if it exists.
 *    -- Otherwise, downloads into default package location -
 *    cache/fota/DIFF_PACKAGE_NAME
 * 2. main system reboots into recovery
 * 3. get_args() writes BCB with "boot-recovery"
 *    -- after this, fota cookie is set to enable modem image update --
 *    -- rebooting into recovery to start android update --
 * 4. main system reboots into recovery
 * 5. get_args() writes BCB with "boot-recovery"
 * 6. install_package() attempts to install android delta update
 *    NOTE: the package install must itself be restartable from any point
 * 7. If update succeeds, calls start_delta_modemupdate() to begin
 *    modem update.
 *    NOTE: the package install must itself be restartable from any point
 * 8. If update succeeds, reset fota cookie.
 * 9. finish_recovery() erases BCB
 *    -- after this, rebooting will (try to) restart the main system --
 * 10. ** if install failed **
 *    10a. Show error icon, reset fota cookie.
 *    10b. finish_recovery() erases BCB
 *    -- after this, rebooting will (try to) restart the main system --
 * 11. handle_deltaupdate_status() calls reboot() to boot main system
 */
static int handle_deltaupdate_status(void)
{
    int update_status;
    struct stat status;

    // Proceed with normal GOTA if return is -1
    if (deltaupdate_pkg_location(diff_pkg_path_name) == -1 )
        return -1;

    //Increment count that indicates number of times device enters into recovery
    //during delta update. This prevents the device recycling endlessly in recovery mode.
    increment_deltaupdate_recoverycount();

    update_status = get_deltaupdate_status();

    // emmc device no need set fota cookie and reboot
    if (target_is_emmc())
       if (update_status == START_DELTA_UPDATE)
           update_status = DELTA_UPDATE_IN_PROGRESS;
    LOGI("update_status = %d\n", update_status);

    switch(update_status)
    {
    case START_DELTA_UPDATE:
          set_deltaupdate_status(DELTA_UPDATE_IN_PROGRESS, 0);
          if (set_fota_cookie())
              LOGE("Failed to set FOTA cookie\n");
          break;

    case DELTA_UPDATE_IN_PROGRESS:
          start_deltaupdate(diff_pkg_path_name);
          break;

    default:
          LOGI("No update set\n");
          if (MAX_NUM_UPDATE_RECOVERY < get_deltaupdate_recoverycount()){
             reset_deltaupdate_recovery_bootmessage();
             if (reset_fota_cookie())
                 LOGE("Failed to reset FOTA cookie\n");
          }
          return EXIT_SUCCESS;
    }
    sync();
    LOGI("android_reboot(ANDROID_RB_RESTART)\n");
    android_reboot(ANDROID_RB_RESTART, 0, 0);
    return EXIT_SUCCESS;
}

int
main(int argc, char **argv) {
    time_t start = time(NULL);

    load_volume_table();
    Device* device = gDevice = make_device();
    ui = device->GetUI();

    ui->Init();
    ui->SetBackground(RecoveryUI::NONE);
    
	/*enable the backlight*/
    //write_file("/sys/class/leds/lcd-backlight/brightness", "128");

    // If these fail, there's not really anywhere to complain...
    freopen(TEMPORARY_LOG_FILE, "a", stdout); setbuf(stdout, NULL);
    freopen(TEMPORARY_LOG_FILE, "a", stderr); setbuf(stderr, NULL);
    
    // If this binary is started with the single argument "--adbd",
    // instead of being the normal recovery binary, it turns into kind
    // of a stripped-down version of adbd that only supports the
    // 'sideload' command.  Note this must be a real argument, not
    // anything in the command file or bootloader control block; the
    // only way recovery should be run with this argument is when it
    // starts a copy of itself from the apply_from_adb() function.
    if (argc == 2 && strcmp(argv[1], "--adbd") == 0) {
        adb_main();
        return 0;
    }

    printf("Starting recovery on %s", ctime(&start));

    /*Device* device = make_device();
    ui = device->GetUI();

    ui->Init();
    ui->SetBackground(RecoveryUI::NONE);
    load_volume_table();*/
    get_args(&argc, &argv);

    int previous_runs = 0;
    const char *send_intent = NULL;
    const char *update_package = NULL;
    int wipe_data = 0, wipe_cache = 0;
    bool just_exit = false;
    int factory_reset = 0;

    //check delta update first
    handle_deltaupdate_status();

    int arg;
    while ((arg = getopt_long(argc, argv, "", OPTIONS, NULL)) != -1) {
        printf("\narg=%c\n",arg);
        switch (arg) {
        case 'p': previous_runs = atoi(optarg); break;
        case 's': send_intent = optarg; break;
        case 'u': update_package = optarg; break;
        case 'w': wipe_data = wipe_cache = 1; factory_reset = 1; break;
        case 'c': wipe_cache = 1; break;
        case 't': ui->ShowText(true); break;
        case 'x': just_exit = true; break;
        case '?':
            LOGE("Invalid command argument\n");
            continue;
        }
    }

#ifdef HAVE_SELINUX
    struct selinux_opt seopts[] = {
      { SELABEL_OPT_PATH, "/file_contexts" }
    };

    sehandle = selabel_open(SELABEL_CTX_FILE, seopts, 1);

    if (!sehandle) {
        fprintf(stderr, "Warning: No file_contexts\n");
        ui->Print("Warning:  No file_contexts\n");
    }
#endif

    device->StartRecovery();
    write_file("/sys/class/leds/lcd-backlight/brightness", "128");

    printf("Command:");
    for (arg = 0; arg < argc; arg++) {
        printf(" \"%s\"", argv[arg]);
    }
    printf("\n");

    if (update_package) {
        // For backwards compatibility on the cache partition only, if
        // we're given an old 'root' path "CACHE:foo", change it to
        // "/cache/foo".
        if (strncmp(update_package, "CACHE:", 6) == 0) {
            printf("Change CACHE: to /cache");
            int len = strlen(update_package) + 10;
            char* modified_path = (char*)malloc(len);
            strlcpy(modified_path, "/cache/", len);
            strlcat(modified_path, update_package+6, len);
            printf("(replacing path \"%s\" with \"%s\")\n",
                   update_package, modified_path);
            update_package = modified_path;
        }
    }
    printf("\n");
    printf("==property list==\n");
    property_list(print_property, NULL);
    printf("================\n");

    int status = INSTALL_SUCCESS;
    // do wipe_data first for the "cache/recovery/cmd" input
    // otherwise, it will never been executed.
    if (wipe_data) {
        int res = 0;
        ui->Print("to wipe data\n");
        if (erase_volume("/data")) status = -1;
        if (wipe_cache && erase_volume("/cache")) status = -1;
        if (res != 0) ui->Print("data wipe failed.\n");
    }

    if (update_package != NULL) {
	#ifdef ADUPS_FOTA_SUPPORT
		if(ensure_path_mounted(SDCARD_ROOT) != 0)
		{
			ui->Print("ensure_path_mounted SDCARD_ROOT failed.\n");
		}
		printf("to update_package=%s\n",update_package);
		mkdir(RADIO_DIR,777);
		status = prepare_install_firmwave();
		if (status == INSTALL_SUCCESS)
		{
			status = install_package(update_package, &wipe_cache, TEMPORARY_INSTALL_FILE, verify_signature);
			if (status == INSTALL_SUCCESS && wipe_cache) 
			{
				if (erase_volume("/cache")) 
				{
					LOGE("Cache wipe (requested by package) failed.");
				}
			}
		}

		adupsfota_install_finish(update_package, status);
		if (status != INSTALL_SUCCESS)
		{
			ui->Print("Installation aborted.\n");
			finish_recovery(send_intent);
			ensure_path_unmounted(SDCARD_ROOT);
			ui->Print("Rebooting...\n");
			android_reboot(ANDROID_RB_RESTART, 0, 0);
			return INSTALL_ERROR;
		}
		ensure_path_unmounted(SDCARD_ROOT);
	#else
        printf("to update_package=%s\n",update_package);
        ensure_path_mounted("/cache");
        mkdir(RADIO_DIR,777);
        status = install_package(update_package, &wipe_cache, TEMPORARY_INSTALL_FILE, verify_signature);
        if (status == INSTALL_SUCCESS && wipe_cache) {
            if (erase_volume("/cache")) {
                LOGE("Cache wipe (requested by package) failed.");
            }
        }
        if (status != INSTALL_SUCCESS) ui->Print("Installation aborted.\n");
	#endif
    }
    else if (wipe_data) {
        printf("to wipe_data\n");
        if (device->WipeData()) status = INSTALL_ERROR;
        if (erase_volume("/data")) status = INSTALL_ERROR;
        if (wipe_cache && erase_volume("/cache")) status = INSTALL_ERROR;
        if (status != INSTALL_SUCCESS) ui->Print("Data wipe failed.\n");
        if (factory_reset) {
            apply_pre_installed_apps();
            ui->Print("reboot system.\n");
            goto recovery_end;
        }
    } else if (wipe_cache) {
        printf("to wipe_cache\n");
        if (wipe_cache && erase_volume("/cache")) status = INSTALL_ERROR;
        if (status != INSTALL_SUCCESS) ui->Print("Cache wipe failed.\n");
    } else if (!just_exit) {
        status = INSTALL_ERROR;  // No command specified
    }

    if (status != INSTALL_SUCCESS) ui->SetBackground(RecoveryUI::ERROR);
    if (status != INSTALL_SUCCESS) {
        prompt_and_wait(device);
    } else if (ui->IsTextVisible()) {
        int key;

        ui->Print("Going to reboot. Press any key to stop\n");
        //Flush keys to clear keys accidently pressed by user.
        ui->FlushKeys();
        key = ui->WaitKey(UI_REBOOT_TIMEOUT_SEC, 0);

        if(key != -1)
            prompt_and_wait(device);
    }

recovery_end:
    // Otherwise, get ready to boot the main system...
    finish_recovery(send_intent);
    ui->Print("Rebooting...\n");
    android_reboot(ANDROID_RB_RESTART, 0, 0);
    return EXIT_SUCCESS;
}
