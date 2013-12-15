// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
  Common GCS MAVLink functions for all vehicle types

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


#include <GCS.h>
#include <DataFlash.h>
#include <stdio.h>

/**
   handle all types of log download requests from the GCS
 */
void GCS_MAVLINK::handle_log_message(mavlink_message_t *msg, DataFlash_Class &dataflash)
{
    switch (msg->msgid) {
    case MAVLINK_MSG_ID_LOG_REQUEST_LIST:
        handle_log_request_list(msg, dataflash);
        break;
    case MAVLINK_MSG_ID_LOG_REQUEST_DATA:
        handle_log_request_data(msg, dataflash);
        break;
    }
        
}


/**
   handle all types of log download requests from the GCS
 */
void GCS_MAVLINK::handle_log_request_list(mavlink_message_t *msg, DataFlash_Class &dataflash)
{
    mavlink_log_request_list_t packet;
    mavlink_msg_log_request_list_decode(msg, &packet);
    if (mavlink_check_target(packet.target_system, packet.target_component))
        return;
    _log_listing = true;
    _log_sending = false;

    _log_num_logs = dataflash.get_num_logs();
    int16_t last_log_num = dataflash.find_last_log();

    _log_next_list_entry = packet.start;
    _log_last_list_entry = packet.end;

    if (_log_last_list_entry > last_log_num) {
        _log_last_list_entry = last_log_num;
    }
    if (_log_next_list_entry < last_log_num + 1 - _log_num_logs) {
        _log_next_list_entry = last_log_num + 1 - _log_num_logs;
    }
}


/**
   handle request for log data
 */
void GCS_MAVLINK::handle_log_request_data(mavlink_message_t *msg, DataFlash_Class &dataflash)
{
    mavlink_log_request_data_t packet;
    mavlink_msg_log_request_data_decode(msg, &packet);
    if (mavlink_check_target(packet.target_system, packet.target_component))
        return;

    _log_listing = false;
    _log_sending = false;

    uint16_t num_logs = dataflash.get_num_logs();
    int16_t last_log_num = dataflash.find_last_log();
    if (packet.id > last_log_num || packet.id < last_log_num + 1 - num_logs) {
        return;
    }

    uint32_t time_utc, size;
    dataflash.get_log_info(packet.id, size, time_utc);
    uint16_t end;
    dataflash.get_log_boundaries(packet.id, _log_data_page, end);
    _log_num_data = packet.id;
    _log_data_offset = packet.ofs;
    if (_log_data_offset >= size) {
        _log_data_remaining = 0;
    } else {
        _log_data_remaining = size - _log_data_offset;
    }
    _log_sending = true;
}

/**
   trigger sending of log messages if there are some pending
 */
void GCS_MAVLINK::handle_log_send(DataFlash_Class &dataflash)
{
    if (_log_listing) {
        handle_log_send_listing(dataflash);
    }
    uint8_t num_sends = 5;
    if (stream_slowdown != 0) {
        // we're using a radio and starting to clag up, slowdown log send
        num_sends = 1;
    }
    for (uint8_t i=0; i<num_sends; i++) {
        if (_log_sending) {
            handle_log_send_data(dataflash);
        }
    }
}

/**
   trigger sending of log messages if there are some pending
 */
void GCS_MAVLINK::handle_log_send_listing(DataFlash_Class &dataflash)
{
    int16_t payload_space = comm_get_txspace(chan) - MAVLINK_NUM_NON_PAYLOAD_BYTES;
    if (payload_space < MAVLINK_MSG_ID_LOG_ENTRY_LEN) {
        // no space
        return;
    }

    uint32_t size, time_utc;
    dataflash.get_log_info(_log_next_list_entry, size, time_utc);
    mavlink_msg_log_entry_send(chan, _log_next_list_entry, _log_num_logs, _log_last_list_entry, time_utc, size);
    if (_log_next_list_entry == _log_last_list_entry) {
        _log_listing = false;
    } else {
        _log_next_list_entry++;
    }
}

/**
   trigger sending of log data if there are some pending
 */
void GCS_MAVLINK::handle_log_send_data(DataFlash_Class &dataflash)
{
    int16_t payload_space = comm_get_txspace(chan) - MAVLINK_NUM_NON_PAYLOAD_BYTES;
    if (payload_space < MAVLINK_MSG_ID_LOG_DATA_LEN) {
        // no space
        return;
    }

    int16_t ret = 0;
    uint32_t len = _log_data_remaining;
    uint8_t data[90];

    if (len > 90) {
        len = 90;
    }
    ret = dataflash.get_log_data(_log_num_data, _log_data_page, _log_data_offset, len, data);
    if (ret < 0) {
        // report as EOF on error
        ret = 0;
    }
    mavlink_msg_log_data_send(chan, _log_num_data, _log_data_offset, ret, data);
    _log_data_offset += len;
    _log_data_remaining -= len;
    if (len < 90) {
        _log_sending = false;
    }
}