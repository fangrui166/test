/* Copyright (c) 2010-2012, Code Aurora Forum. All rights reserved.

 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Code Aurora Forum, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _BOOTLOADER_RECOVERY_H
#define _BOOTLOADER_RECOVERY_H

#define UPDATE_MAGIC       "MSM-RADIO-UPDATE"
#define UPDATE_MAGIC_SIZE  16
#define UPDATE_VERSION     0x00010000

#define FASTMMI_MSG_OFFSET (0)

typedef enum{
	BOOT_MODE_NORMAL = 0x0,
	
	BOOT_MODE_FASTMMI_PCBA = 0x10,
	BOOT_MODE_FASTMMI_FULL,

	BOOT_MODE_FTM = 0x20,
	BOOT_MODE_USB_CHG,

	BOOT_MODE_RECOVERY = 0x30,

	BOOT_MODE_BOOTLOADER = 0x40
}boot_mode_type;

#define BOOT_MODE_MAGIN_NUM 0x12344321

struct boot_mode_message {
	unsigned int magic;
	boot_mode_type boot_mode;
};


/* Recovery Message */
struct recovery_message {
	char command[32];
	char status[32];
	char recovery[1024];
	#if ADUPS_FOTA_SUPPORT
	unsigned int cp_recovery_step;
	#endif
};

struct save_log_message {
	unsigned flags[2];
	unsigned length;
};

struct update_header {
	unsigned char MAGIC[UPDATE_MAGIC_SIZE];

	unsigned version;
	unsigned size;

	unsigned image_offset;
	unsigned image_length;

	unsigned bitmap_width;
	unsigned bitmap_height;
	unsigned bitmap_bpp;

	unsigned busy_bitmap_offset;
	unsigned busy_bitmap_length;

	unsigned fail_bitmap_offset;
	unsigned fail_bitmap_length;
};



int get_recovery_message(struct recovery_message *out);
int set_recovery_message(const struct recovery_message *in);

int read_update_header_for_bootloader(struct update_header *header);
int update_firmware_image (struct update_header *header, char *name);

int recovery_init (void);

int save_debug_message(void);

extern unsigned boot_into_recovery;

#endif
