/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
 * Example of DataFlash library.
 * originally based on code by Jordi MuÒoz and Jose Julio
 */

// Libraries
#include <AP_HAL.h>
#include <AP_HAL_YUNEEC.h>
#include <AP_Common.h>
#include <AP_Param.h>
#include <AP_Progmem.h>
#include <AP_Math.h>
#include <AP_Compass.h>
#include <Filter.h>
#include <AP_Declination.h>
#include <AP_Airspeed.h>
#include <AP_Baro.h>
#include <AP_AHRS.h>
#include <AP_ADC.h>
#include <AP_ADC_AnalogSource.h>
#include <AP_InertialSensor.h>
#include <AP_GPS.h>
#include <DataFlash.h>
#include <GCS_MAVLink.h>
#include <AP_Mission.h>
#include <StorageManager.h>
#include <AP_Terrain.h>
#include <AP_Notify.h>
#include <AP_Vehicle.h>
#include <AP_NavEKF.h>
#include <AP_Rally.h>
#include <AP_Scheduler.h>
#include <utility/pinmap_typedef.h>
#include <string.h>
#include <stdio.h>

#define FIRST_STRING	"First string in my file\n"
#define FIRST_STRING_LEN	sizeof(FIRST_STRING)
#define SECOND_STRING	"Second string in my file\n"
#define SECOND_STRING_LEN	sizeof(SECOND_STRING)
#define THIRD_STRING	"File moved here\n"
#define THIRD_STRING_LEN	sizeof(THIRD_STRING)
const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

DataFlash_YUNEEC_File DataFlash(HAL_BOARD_LOG_DIRECTORY);

FRESULT scan_files (
    char* path        /* Start node to be scanned (also used as work area) */
)
{
    FRESULT res;
    FILINFO fno;
    DIR dir;
    int i;
    char *fn;   /* This function assumes non-Unicode configuration */
    static int times = 0;
    res = f_opendir(&dir, path);                       /* Open the directory */
    hal.console->printf("times = %d\n step1: res = %d\n", times++, res);
    if (res == FR_OK) {
        i = strlen(path);
        for (;;) {
            res = f_readdir(&dir, &fno);                   /* Read a directory item */
            hal.console->printf(" step2: res = %d fno.fname[0] = %c\n", res, fno.fname[0]);
            if (res != FR_OK || fno.fname[0] == 0) break;  /* Break on error or end of dir */
            if (fno.fname[0] == '.') continue;             /* Ignore dot entry */
            fn = fno.fname;
            if (fno.fattrib & AM_DIR) {                    /* It is a directory */
                sprintf(&path[i], "/%s", fn);
                hal.console->printf("  enter scan_file: path = %s\n", path);
                res = scan_files(path);
                hal.console->printf("  end scan_file: res = %d\n", res);
                path[i] = 0;
                if (res != FR_OK) break;
            } else {                                       /* It is a file. */
               hal.console->printf("%s/%s\n", path, fn);
            }
        }
        f_closedir(&dir);
        hal.console->printf("scan files end here\n");
    }

    return res;
}

FRESULT empty_directory (
    char* path      /* Working buffer filled with start directory */
)
{
    UINT i, j;
    FRESULT fr;
    DIR dir;
    FILINFO fno;

    fr = f_opendir(&dir, path);
    if (fr == FR_OK) {
        for (i = 0; path[i]; i++) ;
        path[i++] = '/';
        for (;;) {
            fr = f_readdir(&dir, &fno);
            if (fr != FR_OK || !fno.fname[0]) break;
            if (_FS_RPATH && fno.fname[0] == '.') continue;
            j = 0;
            do
                path[i+j] = fno.fname[j];
            while (fno.fname[j++]);
            if (fno.fattrib & AM_DIR) {
                fr = empty_directory(path);
                if (fr != FR_OK) break;
            }
            fr = f_unlink(path);
            if (fr != FR_OK) break;
        }
        path[--i] = '\0';
        f_closedir(&dir);
    }

    return fr;
}

void setup()
{
	/* Fatfs object */
	FATFS FatFs;
	/* File object */
	FIL fil;
	char buff[64];

	hal.gpio->pinMode(PC4, HAL_GPIO_OUTPUT);
	hal.gpio->pinMode(PC5, HAL_GPIO_OUTPUT);

	hal.gpio->write(PC4, 1);
	hal.gpio->write(PC5, 1);

	hal.console->printf("SD card test start\n");

	for (int i = 0; i < 4; i++) {
		hal.gpio->toggle(PC4);
		hal.gpio->toggle(PC5);
		hal.scheduler->delay(250);
	}

	FRESULT res = f_mount(&FatFs, "0:", 1);

	/* Mount drive */
	if (res == FR_OK) {
		/* Mounted OK, turn on LED */
		hal.gpio->write(PC4, 0);

		/* Try to open file */
		if (f_open(&fil, "1stfile.txt", FA_CREATE_ALWAYS | FA_READ | FA_WRITE) == FR_OK) {
			hal.console->printf("created 1stfile.txt\n");

			/* If we put more than 0 characters (everything OK) */
			if (f_puts(FIRST_STRING, &fil) > 0) {
				for (int i = 0; i < 6; i++) {
					hal.gpio->toggle(PC4);
					hal.gpio->toggle(PC5);
					hal.scheduler->delay(250);
				}
			}

			/* Close file, don't forget this! */
			f_close(&fil);
		}

		/* Try to open file */
		if (f_open(&fil, "2ndfile.txt", FA_CREATE_ALWAYS | FA_READ | FA_WRITE) == FR_OK) {
			hal.console->printf("created 2ndfile.txt\n");

			/* If we put more than 0 characters (everything OK) */
			if (f_puts(SECOND_STRING, &fil) > 0) {
				for (int i = 0; i < 6; i++) {
					hal.gpio->toggle(PC4);
					hal.gpio->toggle(PC5);
					hal.scheduler->delay(250);
				}
			}

			/* Close file, don't forget this! */
			f_close(&fil);
		}

		if (f_rename("2ndfile.txt", "3rdfile.txt") == FR_OK) {
			hal.console->printf("rename 2ndfile.txt -> 3rdfile.txt\n");
		}

		unsigned int bw;
		if (f_mkdir("Yuneec") == FR_OK) {
			if (f_mkdir("Yuneec/logs") == FR_OK) {
				hal.console->printf("created /Yuneec/logs dir\n");
				if (f_rename("3rdfile.txt", "Yuneec/logs/3rdfile.txt") == FR_OK) {
					hal.console->printf("changed 3rdfile.txt dir\n");
					if (f_open(&fil, "Yuneec/logs/3rdfile.txt", FA_WRITE) == FR_OK) {
						hal.console->printf("moved 3rdfile.txt\n");
						f_lseek(&fil, SECOND_STRING_LEN);
						f_write(&fil, THIRD_STRING, THIRD_STRING_LEN, &bw);
						hal.console->printf("string len: %d write: %d\n", THIRD_STRING_LEN, bw);
						f_close(&fil);
					}
				}
			}
		}

	    FILINFO fno;
		if (f_stat("Yuneec/logs/3rdfile.txt", &fno) == FR_OK) {
			hal.console->printf("write bytes: %d fsize: %u fname: %s\n", THIRD_STRING_LEN + SECOND_STRING_LEN, fno.fsize, fno.fname);
		}

		hal.console->set_blocking_writes(true);
		strcpy(buff, "Yuneec");
		res = scan_files(buff);
		hal.console->printf("result: %d\n", res);

		strcpy(buff, "Yuneec/logs");
	    res = empty_directory(buff);

	    if (res) {
	        hal.console->printf("Function failed. (%u)\n", res);
	    } else {
	        hal.console->printf("All contents in the %s are successfully removed.\n", buff);
	    }

		/* Unmount drive, don't forget this! */
		f_mount(0, "0:", 1);
	}
    hal.scheduler->delay(100);
}

void loop()
{
    hal.scheduler->delay(20000);
}

AP_HAL_MAIN();
