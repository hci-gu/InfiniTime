# Minute Data BLE Service

This document describes the on-device contract for the minute-resolution motion and heart-rate history export over BLE. The intent is to share the details with the Flutter companion and other tools.

## Service and characteristics

* **Service UUID**: `00040000-78fc-48fe-8e23-433b3a1942d0`. The UUID is only advertised once at least one minute of history exists on the watch.
* **Control characteristic** (`00040100-78fc-48fe-8e23-433b3a1942d0`): Write + Indicate. Carries handshake, range requests, acks and aborts. All multi-byte fields are little-endian.
* **Status characteristic** (`00040200-78fc-48fe-8e23-433b3a1942d0`): Read-only snapshot of the log window {status, oldest epoch, newest epoch, available minutes}.
* **Data characteristic** (`00040300-78fc-48fe-8e23-433b3a1942d0`): Notify. Streams chunked payloads that contain one or more minute samples.

## Control opcodes

| Opcode | Direction | Payload | Notes |
| ------ | --------- | ------- | ----- |
| `0x01` | Client→watch | `{client_version:8, mtu:16}` | Handshake. The watch responds via Control Indicate with `{0x81, server_version, sample_size, max_window, flags}`. `sample_size` is always 11 bytes. `flags` currently only contains `0x01` (`missing HR supported`).
| `0x02` | Client→watch | `{start_epoch:32, minute_count:16}` | Request a streaming window starting at the provided epoch. The watch copies up to `minute_count` (bounded by storage) into the transfer buffer.
| `0x03` | Client→watch | `{last_sequence:16}` | Acknowledge that all samples up to `last_sequence` have been safely stored by the companion. The watch releases those minutes from RAM/FS.
| `0x7F` | Client→watch | none | Abort the current transfer. The watch stops streaming and sends a terminal status frame.
| `0x81` | Watch→client | `{server_version:8, sample_size:16, max_window:16, flags:8}` | Handshake response.
| `0x82` | Watch→client | `{status:8, oldest_epoch:32, newest_epoch:32, available:16}` | Status/terminal frame. `status` values are listed below.

### Status/error codes

* `0x00` – OK / transfer complete
* `0x01` – Busy (a transfer is already running)
* `0x02` – Invalid range (malformed payload)
* `0x03` – Nothing to send (no samples >= requested epoch)
* `0x04` – Transfer aborted by the companion
* `0x7F` – Internal error

## Data notifications

Each notification contains a packed array of samples. Every sample uses the following layout (11 bytes):

```
struct DataSamplePayload {
  uint16_t sequence;      // Monotonic per transfer, starting at 1
  uint32_t minuteEpoch;   // Start of the minute, in UTC seconds
  int16_t accelAvg;       // Average acceleration magnitude (mg)
  int16_t heartRateAvg;   // Average bpm over the minute
  uint8_t flags;          // Bit 0 => 1 when heart-rate was missing for that minute
};
```

The watch batches as many samples as will fit inside the current ATT MTU (payload ≤ `mtu-3`). Notifications are throttled by a short scheduler delay (`~20 ms`) to avoid starving other services.

## Connection flow

1. Companion requests an MTU ≥247 and subscribes to the Data notifications and Control indications.
2. Write `0x01` handshake. The watch responds with `0x81` describing the maximum window (`1440` minutes) and supported flags.
3. Issue a range request `0x02 {start_ts, count}`. When history exists the watch enters streaming mode and begins notifying `DataSamplePayload` packets sequentially.
4. As soon as the companion has safely written a batch to disk it writes `0x03 {last_seq}` so the watch can reclaim space.
5. When the requested count is satisfied (or storage ends) the watch sends a terminal `0x82` status indication. The companion may start another window immediately.
6. Send `0x7F` at any time to abort. The watch halts the transfer, clears the queued buffer, and reports status `0x04`.

## Flags and gaps

`flags & 0x01` signals that no heart-rate data existed for that minute. Acceleration is still valid, so the Flutter decoder should treat the HR value as unknown when this bit is set. Minutes without timestamps are never emitted on the BLE interface.

## Manual verification checklist

1. **nRF Connect sanity** – pair with the watch, confirm that the Minute Data service appears in the attribute table once at least one minute is logged. Check that the Control/Status/Data characteristics match the UUIDs above.
2. **Flood test** – request `count=200+` minutes at MTU 247 and watch the logs (`NRF_LOG_INFO` messages) to confirm the streaming loop completes without memory growth.
3. **Resume test** – start a transfer, disconnect mid-stream, reconnect, handshake, and issue another range request. The watch should restart cleanly and drop data only when it receives the next `0x03` ACK.

## Advertising behavior

The watch only appends the Minute Data service UUID to advertisement packets when `motionController.LoggedMinuteCount() > 0`. That keeps idle devices from advertising an unsupported feature.
