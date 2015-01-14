/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/* 
   DataFlash logging - file oriented variant

   This uses posix file IO to create log files called logs/NN.bin in the
   given directory
 */

#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_YUNEEC
#include "DataFlash.h"
#include <tm_stm32f4_fatfs.h>
#include <ff.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>

extern const AP_HAL::HAL& hal;

#define MAX_LOG_FILES 65536U
#define DATAFLASH_PAGE_SIZE 1024UL
#define DATAFLASH_MIN_FREE_SPACE	1024 * 1024UL // 1Mbytes
#define LAST_LOG_DIRECTORY	"YUNEEC/LASTLOG"
/*
  constructor
 */
DataFlash_YUNEEC_File::DataFlash_YUNEEC_File(const char *log_directory) :
	_FatFs(),
	_write_fil(),
	_read_fil(),
    _read_offset(0),
    _write_offset(0),
    _initialised(false),
    _log_directory(log_directory),
	_last_log_directory(LAST_LOG_DIRECTORY),
    _writebuf(NULL),
    _writebuf_size(16*1024),
    _writebuf_chunk(512),
    _writebuf_head(0),
    _writebuf_tail(0),
    _last_write_time(0)
{}


// initialisation
void DataFlash_YUNEEC_File::Init(const struct LogStructure *structure, uint8_t num_types)
{
    DataFlash_Class::Init(structure, num_types);
    // create the log directory if need be
    FRESULT res;
    FILINFO fno;

    // try to cope with an existing lower case log directory name.
    res = f_mount(&_FatFs, "0:", 1);
    if (res != FR_OK) {
    	hal.console->printf("Failed to mount SD card\n");
    }

    res = f_stat(_log_directory, &fno);
    if (res != FR_OK) {
    	res = f_mkdir("YUNEEC");
    	if (res == FR_OK) {
    		res = f_mkdir("YUNEEC/LOGS");

			if (res != FR_OK) {
				hal.console->printf("Failed to create log directory %s\n", _log_directory);
				return;
			}

        }
    }

    if (_writebuf != NULL) {
        free(_writebuf);
    }

    /*
      if we can't allocate the full writebuf then try reducing it
      until we can allocate it
     */
    while (_writebuf == NULL && _writebuf_size >= _writebuf_chunk) {
        _writebuf = (uint8_t *)malloc(_writebuf_size);
        if (_writebuf == NULL) {
            _writebuf_size /= 2;
        }
    }
    if (_writebuf == NULL) {
        hal.console->printf("Out of memory for logging\n");
        return;
    }
    _writebuf_head = _writebuf_tail = 0;
    hal.scheduler->register_timer_process(AP_HAL_MEMBERPROC(&DataFlash_YUNEEC_File::_timer_proc));
    _initialised = true;
}

// return true for CardInserted() if we successfully initialised
bool DataFlash_YUNEEC_File::CardInserted(void)
{
    return _initialised;
}


// erase handling
bool DataFlash_YUNEEC_File::NeedErase(void)
{
	uint32_t total, free;

	// get free space of SD card
	if (TM_FATFS_DriveSize(&total, &free) == FR_OK) {
		hal.console->printf("SD card free space: %u total: %u\n", free, total);
		// free space is less than 1Mbytes, we need erase SD card
		if (free < DATAFLASH_MIN_FREE_SPACE)
			return true;
	}

    return false;
}

/*
  construct a log file name given a log number. 
  Note: Caller must free.
 */
char *DataFlash_YUNEEC_File::_log_file_name(uint16_t log_num)
{
    char *buf = NULL;
    asprintf(&buf, "%s/%u.BIN", _log_directory, (unsigned)log_num);
    return buf;
}

/*
  return path name of the lastlog.txt marker file
  Note: Caller must free.
 */
char *DataFlash_YUNEEC_File::_lastlog_file_name(void)
{
    char *buf = NULL;
    asprintf(&buf, "%s/LASTLOG.TXT", _log_directory);
    return buf;
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

// remove all log files except the latest one
void DataFlash_YUNEEC_File::EraseAll()
{
	if (!_initialised)
		return;

	DIR d;
	FRESULT res;
	FILINFO fno;
	FIL File[2];
	char *fname;
    char path[64];

    stop_logging();

	//
	// check if YUNEEC/LASTLOG directory is exist
	//
    res = f_stat(_last_log_directory, &fno);
    // create one if not exist
    if (res != FR_OK) {
		res = f_mkdir(_last_log_directory);

		if (res != FR_OK) {
			hal.console->printf("Failed to create %s\n", _last_log_directory);
			return;
		}
    }
    // it's there, empty this directory
    else {
		strcpy(path, _last_log_directory);

	    res = empty_directory(path);

	    if (res) {
	        hal.console->printf("Failed to empty directory %s\n", path);
	    } else {
	        hal.console->printf("All contents in the %s are successfully removed.\n", path);
	    }
    }

    uint16_t last_log_num = find_last_log();

    // log_num start from 1 to indicate files exist
    if (last_log_num == 0)
    	return;

	fname = _log_file_name(last_log_num);
	if (fname != NULL) {
		char *buf = NULL;
		asprintf(&buf, "%s/LASTLOG.BIN", _last_log_directory);
		res = f_rename(fname, buf);
		if (res != FR_OK) {
			hal.console->printf("Failed to rename and move %s to %s\n", fname, buf);
		}

		free(buf);
		free(fname);
	}


    // clean all log files
	strcpy(path, _log_directory);
    empty_directory(path);
    res = empty_directory(path);

    if (res) {
        hal.console->printf("Failed to empty directory %s\n", path);
    } else {
        hal.console->printf("All contents in the %s are successfully removed.\n", path);
    }



}

/*
  buffer handling macros
 */
#define BUF_AVAILABLE(buf) ((buf##_head > (_tail=buf##_tail))? (buf##_size - buf##_head) + _tail: _tail - buf##_head)
#define BUF_SPACE(buf) (((_head=buf##_head) > buf##_tail)?(_head - buf##_tail) - 1:((buf##_size - buf##_tail) + _head) - 1)
#define BUF_EMPTY(buf) (buf##_head == buf##_tail)
#define BUF_ADVANCETAIL(buf, n) buf##_tail = (buf##_tail + n) % buf##_size
#define BUF_ADVANCEHEAD(buf, n) buf##_head = (buf##_head + n) % buf##_size


/* Write a block of data at current offset */
void DataFlash_YUNEEC_File::WriteBlock(const void *pBuffer, uint16_t size)
{
    if (_write_fil.fs == NULL | !_initialised || !_writes_enabled) {
        return;
    }
    uint16_t _head;
    uint16_t space = BUF_SPACE(_writebuf);
    if (space < size) {
        // discard the whole write, to keep the log consistent
        return;
    }

    if (_writebuf_tail < _head) {
        // perform as single memcpy
        assert(_writebuf_tail+size <= _writebuf_size);
        memcpy(&_writebuf[_writebuf_tail], pBuffer, size);
        BUF_ADVANCETAIL(_writebuf, size);
    } else {
        // perform as two memcpy calls
        uint16_t n = _writebuf_size - _writebuf_tail;
        if (n > size) n = size;
        assert(_writebuf_tail+n <= _writebuf_size);
        memcpy(&_writebuf[_writebuf_tail], pBuffer, n);
        BUF_ADVANCETAIL(_writebuf, n);
        pBuffer = (const void *)(((const uint8_t *)pBuffer) + n);
        n = size - n;
        if (n > 0) {
            assert(_writebuf_tail+n <= _writebuf_size);
            memcpy(&_writebuf[_writebuf_tail], pBuffer, n);
            BUF_ADVANCETAIL(_writebuf, n);
        }
    }
}

/*
  read a packet. The header bytes have already been read.
*/
void DataFlash_YUNEEC_File::ReadBlock(void *pkt, uint16_t size)
{
    if (_read_fil.fs == NULL || !_initialised) {
        return;
    }

    memset(pkt, 0, size);
    unsigned int br;
    f_read(&_read_fil, pkt, size, &br);
    _read_offset += size;
}


/*
  find the highest log number
 */
uint16_t DataFlash_YUNEEC_File::find_last_log(void)
{
    unsigned ret = 0;
    char *fname = _lastlog_file_name();
    if (fname == NULL) {
        return ret;
    }

    FRESULT fr = f_open(&_read_fil, fname, FA_OPEN_EXISTING | FA_READ);
    free(fname);
    if (fr == FR_OK) {
        char buf[10];
        unsigned int br;
        memset(buf, 0, sizeof(buf));
        if (f_read(&_read_fil, buf, sizeof(buf), &br) == FR_OK) {
            sscanf(buf, "%u", &ret);            
        }
        f_close(&_read_fil);
    }
    return ret;
}


uint32_t DataFlash_YUNEEC_File::_get_log_size(uint16_t log_num)
{
    char *fname = _log_file_name(log_num);
    if (fname == NULL) {
        return 0;
    }
    FILINFO fno;
    if (f_stat(fname, &fno) != FR_OK) {
        free(fname);
        return 0;
    }
    free(fname);
    return fno.fsize;
}

uint32_t DataFlash_YUNEEC_File::_get_log_time(uint16_t log_num)
{
    char *fname = _log_file_name(log_num);
    if (fname == NULL) {
        return 0;
    }
    FILINFO fno;
    if (f_stat(fname, &fno) != FR_OK) {
        free(fname);
        return 0;
    }
    free(fname);
    return fno.ftime;
}

/*
  find the number of pages in a log
 */
void DataFlash_YUNEEC_File::get_log_boundaries(uint16_t log_num, uint16_t & start_page, uint16_t & end_page)
{
    start_page = 0;
    end_page = _get_log_size(log_num) / DATAFLASH_PAGE_SIZE;
}

/*
  find the number of pages in a log
 */
int16_t DataFlash_YUNEEC_File::get_log_data(uint16_t log_num, uint16_t page, uint32_t offset, uint16_t len, uint8_t *data)
{
    if (!_initialised) {
        return -1;
    }
    if (_read_fil.fs != NULL && log_num != _read_fd_log_num) {
        f_close(&_read_fil);
    }
    if (_read_fil.fs == NULL) {
        char *fname = _log_file_name(log_num);
        if (fname == NULL) {
            return -1;
        }
        stop_logging();
        if (f_open(&_read_fil, fname, FA_OPEN_EXISTING | FA_READ) != FR_OK)
        	return -1;
        free(fname);

        _read_offset = 0;
        _read_fd_log_num = log_num;
    }
    uint32_t ofs = page * (uint32_t)DATAFLASH_PAGE_SIZE + offset;

    if (ofs != _read_offset) {
    	f_lseek(&_read_fil, ofs);
    	_read_offset = ofs;
    }

    unsigned int br;
    if (f_read(&_read_fil, data, len, &br) == FR_OK) {
        _read_offset += br;
        return br;
    } else
    	return -1;
}

/*
  find size and date of a log
 */
void DataFlash_YUNEEC_File::get_log_info(uint16_t log_num, uint32_t &size, uint32_t &time_utc)
{
    size = _get_log_size(log_num);
    time_utc = _get_log_time(log_num);
}

/*
  get the number of logs - note that the log numbers must be consecutive
 */
uint16_t DataFlash_YUNEEC_File::get_num_logs(void)
{
    uint16_t ret;
    uint16_t high = find_last_log();
    for (ret=0; ret<high; ret++) {
        if (_get_log_size(high - ret) <= 0) {
            break;
        }
    }
    return ret;
}

/*
  stop logging
 */
void DataFlash_YUNEEC_File::stop_logging(void)
{
    if (_write_fil.fs != NULL) {
        log_write_started = false;
    	f_close(&_write_fil);
    }
}


/*
  start writing to a new log file
 */
uint16_t DataFlash_YUNEEC_File::start_new_log(void)
{
    stop_logging();

    if (_read_fil.fs != NULL) {
    	f_close(&_read_fil);
    }

    uint16_t log_num = find_last_log();
    // re-use empty logs if possible
    if (_get_log_size(log_num) > 0 || log_num == 0) {
        log_num++;
    }
    if (log_num > MAX_LOG_FILES) {
        log_num = 1;
    }
    char *fname = _log_file_name(log_num);
    FRESULT res = f_open(&_write_fil, fname, FA_WRITE | FA_CREATE_ALWAYS);
    free(fname);
    if (res != FR_OK) {
        _initialised = false;
        return 0xFFFF;
    }

    _write_offset = 0;
    _writebuf_head = 0;
    _writebuf_tail = 0;
    log_write_started = true;

    // now update lastlog.txt with the new log number
    fname = _lastlog_file_name();
    FIL f;
    if (f_open(&f, fname, FA_WRITE | FA_CREATE_ALWAYS) == FR_OK) {
        f_printf(&f, "%u\r\n", (unsigned)log_num);
        f_close(&f);
    }
    free(fname);

    return log_num;
}

/*
  Read the log and print it on port
*/
void DataFlash_YUNEEC_File::LogReadProcess(uint16_t log_num,
                                    uint16_t start_page, uint16_t end_page, 
                                    void (*print_mode)(AP_HAL::BetterStream *port, uint8_t mode),
                                    AP_HAL::BetterStream *port)
{
    if (!_initialised) {
        return;
    }
    if (_read_fil.fs != NULL) {
        f_close(&_read_fil);
    }
    char *fname = _log_file_name(log_num);
    if (fname == NULL) {
        return;
    }
    FRESULT res = f_open(&_read_fil, fname, FA_OPEN_EXISTING | FA_READ);
    free(fname);
    if (res != FR_OK) {
        return;
    }
    _read_fd_log_num = log_num;
    _read_offset = 0;
    if (start_page != 0) {
        f_lseek(&_read_fil, start_page * DATAFLASH_PAGE_SIZE);
        _read_offset = start_page * DATAFLASH_PAGE_SIZE;
    }

    uint8_t log_step = 0;

    while (true) {
        uint8_t data;
        unsigned int br;
        FRESULT res = f_read(&_read_fil, &data, 1, &br);

        if (res != FR_OK) {
        	hal.console->printf("\nres = %d\n", res);
            // reached end of file
            break;
        }
        _read_offset++;

        // This is a state machine to read the packets
        switch(log_step) {
            case 0:
                if (data == HEAD_BYTE1) {
                    log_step++;
                }
                break;

            case 1:
                if (data == HEAD_BYTE2) {
                    log_step++;
                } else {
                    log_step = 0;
                }
                break;

            case 2:
                log_step = 0;
                _print_log_entry(data, print_mode, port);
                break;
        }
        if (_read_offset >= (end_page+1) * DATAFLASH_PAGE_SIZE) {
            break;
        }
    }

    f_close(&_read_fil);
}

/*
  this is a lot less verbose than the block interface. Dumping 2Gbyte
  of logs a page at a time isn't so useful. Just pull the SD card out
  and look at it on your PC
 */
void DataFlash_YUNEEC_File::DumpPageInfo(AP_HAL::BetterStream *port)
{
    port->printf_P(PSTR("DataFlash: num_logs=%u\n"), 
                   (unsigned)get_num_logs());    
}

void DataFlash_YUNEEC_File::ShowDeviceInfo(AP_HAL::BetterStream *port)
{
    port->printf_P(PSTR("DataFlash logs stored in %s\n"), 
                   _log_directory);
}


/*
  list available log numbers
 */
void DataFlash_YUNEEC_File::ListAvailableLogs(AP_HAL::BetterStream *port)
{
    uint16_t num_logs = get_num_logs();
    int16_t last_log_num = find_last_log();

    if (num_logs == 0) {
        port->printf_P(PSTR("\nNo logs\n\n"));
        return;
    }
    port->printf_P(PSTR("\n%u logs\n"), (unsigned)num_logs);

    for (uint16_t i=num_logs; i>=1; i--) {
        uint16_t log_num = last_log_num - i + 1;
        off_t size;

        char *filename = _log_file_name(log_num);
        if (filename != NULL) {
            size = _get_log_size(log_num);
            if (size != 0) {
                FILINFO fno;
                if (f_stat(filename, &fno) == FR_OK) {
                    port->printf_P(PSTR("Log %u in %s of size %u %u/%u/%u %u:%u\n"), 
                                   (unsigned)log_num, 
                                   filename,
                                   (unsigned)size,
                                   (unsigned)(fno.fdate >> 9 + 1980),
                                   (unsigned)((fno.fdate >> 5) & 15),
                                   (unsigned)(fno.fdate & 31),
                                   (unsigned)((fno.ftime >> 11) & 31),
                                   (unsigned)((fno.ftime >> 5) & 63));
                }
            }
            free(filename);
        }
    }
    port->println();    
}


void DataFlash_YUNEEC_File::_timer_proc(void)
{
    uint16_t _tail;
    if (_write_fil.fs == NULL || !_initialised) {
        return;
    }

    uint16_t nbytes = BUF_AVAILABLE(_writebuf);
    if (nbytes == 0) {
        return;
    }
    uint32_t tnow = hal.scheduler->millis();
    if (nbytes < _writebuf_chunk && 
        tnow - _last_write_time < 2000UL) {
        // write in 512 byte chunks, but always write at least once
        // per 2 seconds if data is available
        return;
    }

    _last_write_time = tnow;
    if (nbytes > _writebuf_chunk) {
        nbytes = _writebuf_chunk;
    }
    if (_writebuf_head > _tail) {
        // only write to the end of the buffer
        nbytes = min(nbytes, _writebuf_size - _writebuf_head);
    }

    // try to align writes on a 512 byte boundary to avoid filesystem
    // reads
    uint32_t ofs = (nbytes + _write_offset) % 512;
    if (ofs != 0) {
        if (ofs < nbytes) {
            nbytes -= ofs;
        }
    }

    assert(_writebuf_head+nbytes <= _writebuf_size);
    unsigned int bw = 0;
    if (f_write(&_write_fil, &_writebuf[_writebuf_head], nbytes, &bw) != FR_OK) {
        f_close(&_write_fil);
        _initialised = false;
    } else {
        _write_offset += bw;
        /*
          the best strategy for minimising corruption on microSD cards
          seems to be to write in 4k chunks and fsync the file on each
          chunk, ensuring the directory entry is updated after each
          write.
         */
        f_sync(&_write_fil);

        BUF_ADVANCEHEAD(_writebuf, bw);
    }
}

#endif

