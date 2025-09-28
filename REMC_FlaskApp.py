import socket
import struct
import time
import sys
import threading
import copy
import io  # for CSV
import csv  # for CSV
from datetime import datetime
from flask import Flask, redirect, url_for, jsonify, Response, request
from collections import deque

# Attempt to use Waitress for a production WSGI server
try:
    from waitress import serve

    USE_WAITRESS = True
except ImportError:
    USE_WAITRESS = False

# --- Configuration ---
# Network and multicast settings
MULTICAST_GROUP_TELEM = '239.9.9.33'  # Multicast IP for telemetry.
MULTICAST_GROUP_CMND = '239.9.9.32'
MULTICAST_INTERFACE_IP = '192.168.1.10'  # Local IP to send/receive multicast (PC's IP, on subnet 255.255.255.0)
LISTEN_IP_FOR_BIND = '0.0.0.0'  # IP to listen on (0.0.0.0 = all).
PORT_TELEM = 13013  # UDP port for telemetry.
PORT_CMND = 13012

# Flask web server settings
WEB_SERVER_HOST = '192.168.1.10'  # IP for the web dashboard.
WEB_SERVER_PORT = 5002  # Port for the web dashboard.
DATA_LOGGING_INTERVAL_SECONDS = 0.0001  # Data logging interval (seconds).
MAX_CSV_BYTES = 1 * 1024 * 1024 * 1024  # 1 GiB hard cap

BYTES_PER_RECORD_EST = 320  # ≈ dict with 6 floats + overhead
MAX_RECORDS_IN_RAM = (1 * 1024 * 1024 * 1024) // BYTES_PER_RECORD_EST


# --- Packet Structure Definitions ---
# Neutrino framing
HEADER_SIZE = 64
# Legacy: 5 floats (20 bytes) + 6 bytes = 26 bytes payload  
# Old: 5 floats (20 bytes) + uint32 (4 bytes) + 6 bytes = 30 bytes payload
# Current: 5 floats (20 bytes) + uint64 (8 bytes) + 6 bytes = 34 bytes payload
DATA_PAYLOAD_SIZE = (5 * 4) + (6 * 1)  # Legacy 26-byte format
EXPECTED_PAYLOAD_SIZE = HEADER_SIZE + DATA_PAYLOAD_SIZE  # For legacy compatibility

NEUTRINO_HEADER_FORMAT = '>IIII16s16sIIQ'
NEUTRINO_HEADER_FIELDS = [
    'msg_id', 'flags', 'num_frags', 'num_atomic_frags',
    'schema_hash', 'schema_frag',
    'frag_idx', 'atomic_idx', 'timestamp_ns'
]

DATA_PAYLOAD_FORMAT = '<fffffBBBBBB'
DATA_PAYLOAD_FIELDS = [
    'switch_voltage_kv', 'switch_current_a',
    'output_voltage_a_kv', 'output_voltage_b_kv',
    'temperature_1_degc',
    'armed_status', 'em_status',
    'msw_a_status', 'msw_b_status',
    'manual_mode_status', 'hold_mode_status'
]



# --- Batched Neutrino Parser (supports 1..N samples per datagram) ---
def parse_neutrino_packet(datagram: bytes):
    """
    Returns: (header_dict, [sample_dict, ...])

    - Header fields are big-endian/network order (as sent by the Due).
    - Payload floats are little-endian (memcpy from Due floats).
    - Accepts multiple formats:
        * legacy 26-byte samples: 5 floats + 6 flags
        * old    30-byte samples: 5 floats + uint32 micros + 6 flags  
        * current 34-byte samples: 5 floats + uint64 ntp_timestamp_us + 6 flags
    Each sample dict will include timestamp fields if available.
    """
    if len(datagram) < HEADER_SIZE:
        raise ValueError('packet too short')

    hdr = datagram[:HEADER_SIZE]
    payload = datagram[HEADER_SIZE:]

    # Header (big-endian) per your NEUTRINO_HEADER_FORMAT
    hv = struct.unpack(NEUTRINO_HEADER_FORMAT, hdr)
    header = dict(zip(NEUTRINO_HEADER_FIELDS, hv))

    # Pretty up schema_hash for downstream consumers
    if isinstance(header.get('schema_hash'), bytes):
        header['schema_hash'] = header['schema_hash'].hex()

    # Decide sample size: try 42 (with us_end), 34 (64-bit NTP), then 30 (32-bit micros), then 26 (legacy)
    SAMPLE_SIZE_WITH_END = (5 * 4) + 8 + 6 + 8  # 42 bytes (5 floats + uint64 us + 6 flags + uint64 us_end)
    SAMPLE_SIZE_CURRENT = (5 * 4) + 8 + 6       # 34 bytes (5 floats + uint64 + 6 flags)
    SAMPLE_SIZE_OLD = (5 * 4) + 4 + 6           # 30 bytes (5 floats + uint32 + 6 flags)
    SAMPLE_SIZE_LEGACY = DATA_PAYLOAD_SIZE       # 26 bytes (5 floats + 6 flags)
    
    if len(payload) % SAMPLE_SIZE_WITH_END == 0:
        sample_size = SAMPLE_SIZE_WITH_END
        timestamp_format = 'ntp64_with_end'  # 64-bit NTP timestamp with end timestamp
    elif len(payload) % SAMPLE_SIZE_CURRENT == 0:
        sample_size = SAMPLE_SIZE_CURRENT
        timestamp_format = 'ntp64'  # 64-bit NTP timestamp
    elif len(payload) % SAMPLE_SIZE_OLD == 0:
        sample_size = SAMPLE_SIZE_OLD
        timestamp_format = 'micros32'  # 32-bit micros
    elif len(payload) % SAMPLE_SIZE_LEGACY == 0:
        sample_size = SAMPLE_SIZE_LEGACY
        timestamp_format = 'none'  # No timestamp
    else:
        raise ValueError(f'payload length {len(payload)} not multiple of 42, 34, 30, or 26')

    n = len(payload) // sample_size
    samples = []
    offset = 0

    # Header time base (ns → sec + fractional us)
    t_ns = header.get('timestamp_ns', 0)
    base_sec = int(t_ns // 1_000_000_000)
    hdr_us = int((t_ns % 1_000_000_000) // 1_000)

    for _ in range(n):
        # 5 floats, little-endian
        sv, sc, ova, ovb, t1 = struct.unpack_from('<fffff', payload, offset)
        offset += 20

        # Parse timestamp based on format
        sample_micros = None
        sample_timestamp_us = None
        sample_timestamp_us_end = None
        
        if timestamp_format == 'ntp64_with_end':
            # 64-bit NTP timestamp in microseconds (little-endian)
            (ntp_timestamp_us,) = struct.unpack_from('<Q', payload, offset)
            offset += 8
            sample_timestamp_us = ntp_timestamp_us
            # For backward compatibility, extract just the microsecond portion
            sample_micros = int(ntp_timestamp_us % 1_000_000)
            
        elif timestamp_format == 'ntp64':
            # 64-bit NTP timestamp in microseconds (little-endian)
            (ntp_timestamp_us,) = struct.unpack_from('<Q', payload, offset)
            offset += 8
            sample_timestamp_us = ntp_timestamp_us
            # For backward compatibility, extract just the microsecond portion
            sample_micros = int(ntp_timestamp_us % 1_000_000)
            
        elif timestamp_format == 'micros32':
            # 32-bit microseconds (legacy format)
            (sample_micros,) = struct.unpack_from('<I', payload, offset)
            offset += 4
            # Compute absolute timestamp using header time base
            sec = base_sec
            # Guard: if header is at very early µs in the next second but sample micros are very high,
            # assume sample was just before the header second (roll back one second).
            if hdr_us < 10_000 and sample_micros > 990_000:
                sec = max(0, sec - 1)
            sample_timestamp_us = sec * 1_000_000 + int(sample_micros)

        # 6 single-byte flags
        ready, em, a, b, manual, hold = struct.unpack_from('<6B', payload, offset)
        offset += 6
        
        # Parse end timestamp if present (new format)
        if timestamp_format == 'ntp64_with_end':
            (ntp_timestamp_us_end,) = struct.unpack_from('<Q', payload, offset)
            offset += 8
            sample_timestamp_us_end = ntp_timestamp_us_end

        samples.append({
            'switch_voltage_kv': sv,
            'switch_current_a':  sc,
            'output_voltage_a_kv': ova,
            'output_voltage_b_kv': ovb,
            'temperature_1_degc': t1,
            'armed_status': ready,
            'em_status': em,
            'msw_a_status': a,
            'msw_b_status': b,
            'manual_mode_status': manual,
            'hold_mode_status': hold,
            'sample_micros': sample_micros,
            'sample_timestamp_us': sample_timestamp_us,
            'sample_timestamp_us_end': sample_timestamp_us_end,
        })

    return header, samples

# --- Shared Data Structures ---
latest_data = {
    'header': {f: None for f in NEUTRINO_HEADER_FIELDS},
    'payload': {f: None for f in DATA_PAYLOAD_FIELDS},
    'last_update_time': 0,
    'packets_received': 0,
    'samples_received': 0,  # Track total samples received
    'last_bundle_size': 0,  # Track last bundle size
    'status': 'Initializing...',
    'sender_ip': None,
    'fsm_state_name': 'UNKNOWN'
}
data_lock = threading.Lock()

historical_data_log = deque(maxlen=MAX_RECORDS_IN_RAM)
historical_data_lock = threading.Lock()

# --- Live Feed Data Storage (only flags=0 samples) ---
# Keep 1 minute of live feed data at 10kHz sample rate / 10000 ratio = ~6 samples per minute
# But we'll keep more to handle varying rates and ensure smooth graphs
live_feed_data = deque(maxlen=1000)  # Keep last 1000 live feed samples
live_feed_lock = threading.Lock()

# --- Batch Collection Storage ---
collection_batches = {}  # Dictionary: batch_id -> batch_data
batches_lock = threading.Lock()
next_batch_id = 1

# --- Batch Capture State ---
batch_capture_active = False
current_batch_id = None
current_batch_samples = []
batch_capture_start_time = None

LOGGING_DATA_FIELDS = [
    'timestamp',            # float seconds since epoch, derived per sample
    'sample_timestamp_us',  # precise microseconds per sample (if present)
    'sample_timestamp_us_end', # precise end microseconds per sample (if present)
    'sample_timestamp_iso', # human readable string
    'switch_voltage_kv',
    'switch_current_a',
    'output_voltage_a_kv',
    'output_voltage_b_kv',
    'temperature_1_degc'
]


# --- UDP Listener Thread ---
def udp_listener_thread():
    global batch_capture_active, current_batch_id, current_batch_samples, batch_capture_start_time
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((LISTEN_IP_FOR_BIND, PORT_TELEM))
        print(f"[UDP] Bound to {LISTEN_IP_FOR_BIND}:{PORT_TELEM}")
    except OSError as e:
        print(f"[UDP] Bind error: {e}")
        sys.exit(1)

    # Join multicast group
    try:
        mreq = socket.inet_aton(MULTICAST_GROUP_TELEM) + socket.inet_aton(MULTICAST_INTERFACE_IP)
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        print(f"[UDP] Joined {MULTICAST_GROUP_TELEM} on {MULTICAST_INTERFACE_IP}")
    except OSError as e:
        print(f"[UDP] Multicast join error: {e}")
        sock.close()
        sys.exit(1)

    packet_count = 0
    while True:
        
        try:
            data, addr = sock.recvfrom(4096)  # Increased for larger bundled packets

            # Accept 1..N samples per datagram
            try:
                header, samples = parse_neutrino_packet(data)
            except Exception:
                continue

            # Debug: Print bundle statistics occasionally
            if packet_count % 100 == 0:  # Every 100 packets
                sample_size_str = ""
                if samples:
                    if 'sample_timestamp_us_end' in samples[0] and samples[0]['sample_timestamp_us_end'] is not None:
                        sample_size_str = " (64-bit NTP timestamps with end times)"
                    elif 'sample_timestamp_us' in samples[0] and samples[0]['sample_timestamp_us'] is not None:
                        if samples[0]['sample_timestamp_us'] > 1e12:  # Likely NTP timestamp
                            sample_size_str = " (64-bit NTP timestamps)"
                        else:
                            sample_size_str = " (32-bit timestamps)"
                    else:
                        sample_size_str = " (no timestamps)"
                print(f"[UDP] Packet {packet_count}: Bundle of {len(samples)} samples ({len(data)} bytes){sample_size_str}")

            # Process samples for both historical logging and live feed
            flags = header.get('flags', 0)
            
            with historical_data_lock:
                for s in samples:
                    if s.get('sample_timestamp_us') is not None:
                        ts_float = s['sample_timestamp_us'] / 1e6   # precise per-sample time
                        ts_iso = time.strftime('%Y-%m-%d %H:%M:%S', time.gmtime(ts_float)) \
                                 + f'.{int(s["sample_timestamp_us"] % 1_000_000):06d}Z'
                    else:
                        ts_float = time.time()                       # fallback if running legacy 26B samples
                        ts_iso = time.strftime('%Y-%m-%d %H:%M:%S', time.gmtime(ts_float)) + 'Z'

                    sample_record = {
                        'sample_timestamp_iso': ts_iso,
                        'timestamp': ts_float,  # float seconds for UI/CSV
                        'sample_timestamp_us': s.get('sample_timestamp_us'),
                        'sample_timestamp_us_end': s.get('sample_timestamp_us_end'),
                        'switch_voltage_kv': s.get('switch_voltage_kv'),
                        'switch_current_a': s.get('switch_current_a'),
                        'output_voltage_a_kv': s.get('output_voltage_a_kv'),
                        'output_voltage_b_kv': s.get('output_voltage_b_kv'),
                        'temperature_1_degc': s.get('temperature_1_degc'),
                    }

                    # Add to continuous historical log
                    historical_data_log.append(sample_record)
                    
                    # Add to live feed if this is a normal sample (flags=0)
                    if flags == 0:  # FLAGS_NORMAL - live feed samples
                        with live_feed_lock:
                            live_feed_data.append(sample_record)

            # Check if we're in batch capture mode
            with batches_lock:
                if batch_capture_active and current_batch_id is not None:
                    flags = header.get('flags', 0)
                    
                    if flags == 2:  # Batch end marker
                        print(f"[BATCH] Received batch end marker for batch {current_batch_id}")
                        finalize_current_batch()
                    elif flags == 1:  # Collected samples
                        # Only capture samples with flags=1 (collected samples)
                        for s in samples:
                            if s.get('sample_timestamp_us') is not None:
                                ts_float = s['sample_timestamp_us'] / 1e6
                                ts_iso = time.strftime('%Y-%m-%d %H:%M:%S', time.gmtime(ts_float)) \
                                         + f'.{int(s["sample_timestamp_us"] % 1_000_000):06d}Z'
                            else:
                                ts_float = time.time()
                                ts_iso = time.strftime('%Y-%m-%d %H:%M:%S', time.gmtime(ts_float)) + 'Z'

                            current_batch_samples.append({
                                'sample_timestamp_iso': ts_iso,
                                'timestamp': ts_float,
                                'sample_timestamp_us': s.get('sample_timestamp_us'),
                                'sample_timestamp_us_end': s.get('sample_timestamp_us_end'),
                                'switch_voltage_kv': s.get('switch_voltage_kv'),
                                'switch_current_a': s.get('switch_current_a'),
                                'output_voltage_a_kv': s.get('output_voltage_a_kv'),
                                'output_voltage_b_kv': s.get('output_voltage_b_kv'),
                                'temperature_1_degc': s.get('temperature_1_degc'),
                            })


            # Keep last sample for UI compatibility (if samples exist)
            if samples:
                payload = samples[-1]
            else:
                # Empty samples (e.g., batch end marker), use a dummy payload
                payload = {}

            # Convert header fields removed here because already done in parser
            frag = header.get('schema_frag')
            if isinstance(frag, bytes):
                fsm_state_name = frag.split(b'\x00', 1)[0].decode(errors='replace')
                header['schema_frag'] = frag.hex()
            else:
                fsm_state_name = 'N/A'

            packet_count += 1
            with data_lock:
                latest_data.update({
                    'header': header,
                    'payload': payload,
                    'last_update_time': time.time(),
                    'packets_received': packet_count,
                    'samples_received': latest_data.get('samples_received', 0) + len(samples),
                    'last_bundle_size': len(samples),
                    'status': 'Receiving OK',
                    'sender_ip': addr[0],
                    'fsm_state_name': fsm_state_name
                })
        except Exception as e:
            print(f"[UDP] Listener error: {e}")
            with data_lock:
                latest_data['status'] = f"Listener Error: {e}"
            time.sleep(1)


# --- Batch Completion Check Thread ---
def finalize_current_batch():
    """Finalize the current batch"""
    global batch_capture_active, current_batch_id, current_batch_samples, batch_capture_start_time
    
    if current_batch_samples:
        collection_batches[current_batch_id] = {
            'id': current_batch_id,
            'timestamp': batch_capture_start_time,
            'sample_count': len(current_batch_samples),
            'samples': current_batch_samples.copy()
        }
        print(f"[BATCH] Batch {current_batch_id} completed with {len(current_batch_samples)} samples")
    
    # Reset batch capture state
    batch_capture_active = False
    current_batch_id = None
    current_batch_samples.clear()
    batch_capture_start_time = None


# --- Data Logging Thread ---
def data_logging_thread_function():
    """
    Log telemetry data at a fixed rate.

    The previous implementation simply slept for the logging interval at the
    *start* of the loop.  This meant that any processing time (collecting data,
    formatting timestamps, acquiring locks, etc.) was added on top of the sleep
    time which drastically slowed the effective logging rate.  When the interval
    was set to 100 µs (10 kHz) we only achieved about 2 kHz.

    To provide a more accurate logging rate we now schedule the next logging
    time using ``time.perf_counter()`` and account for the processing time of
    each iteration.  The timestamp is stored as a floating point seconds value
    (via ``time.time()``) which is much faster than formatting a datetime
    string on every iteration.  The timestamp is formatted only when the CSV is
    generated, keeping the high‑rate logging lightweight.
    """

    next_log_time = time.perf_counter()
    last_packet_count = -1
    while True:
        # Copy the latest packet count and payload under lock
        with data_lock:
            packet_count = latest_data.get('packets_received', -1)
            p = latest_data.get('payload')

        # Only log a new record when a fresh packet has been received
        if packet_count != last_packet_count and p and p.get(LOGGING_DATA_FIELDS[1]) is not None:
            record = {'timestamp': time.time()}
            for f in LOGGING_DATA_FIELDS[1:]:
                record[f] = p.get(f)
            with historical_data_lock:
                historical_data_log.append(record)
            last_packet_count = packet_count

        # Schedule the next iteration accounting for processing overhead
        next_log_time += DATA_LOGGING_INTERVAL_SECONDS
        sleep_time = next_log_time - time.perf_counter()
        if sleep_time > 0:
            time.sleep(sleep_time)


# --- Flask App and Routes ---
app = Flask(__name__)


# --- Flask Routes ---
@app.route('/data')
def get_telemetry_data():
    with data_lock:
        data_to_send = copy.deepcopy(latest_data)

    # Format last‐update timestamp
    if data_to_send.get("last_update_time"):
        data_to_send["last_update_str"] = time.strftime(
            '%Y-%m-%d %H:%M:%S',
            time.localtime(data_to_send["last_update_time"])
        )
    else:
        data_to_send["last_update_str"] = "N/A"

    # If no packet in 3s, mark stale and override status
    now = time.time()
    if data_to_send.get("last_update_time") and (now - data_to_send["last_update_time"] > 3):
        data_to_send["status"] = "No Packets"
        data_to_send["stale"] = True
    else:
        data_to_send["status"] = "Receiving OK"
        data_to_send["stale"] = False

    return jsonify(data_to_send)


@app.route('/live_feed')
def get_live_feed_data():
    """Get live feed samples from the last minute for graphing"""
    with live_feed_lock:
        # Get samples from the last 60 seconds
        current_time = time.time()
        cutoff_time = current_time - 60  # 60 seconds ago
        
        # Filter to only recent samples
        recent_samples = [
            sample for sample in live_feed_data 
            if sample.get('timestamp', 0) >= cutoff_time
        ]
        
        # Return the data
        return jsonify({
            'samples': recent_samples,
            'count': len(recent_samples),
            'time_range_seconds': 60
        })


@app.route('/download_csv')
def download_csv_route():
    csv_fieldnames = LOGGING_DATA_FIELDS
    float_fields_to_format = [
        'switch_voltage_kv', 'switch_current_a', 'output_voltage_a_kv',
        'output_voltage_b_kv', 'temperature_1_degc'
    ]

    def fmt_row(src):
        out = {}
        for f in csv_fieldnames:
            v = src.get(f)
            if f == 'timestamp':
                out[f] = (datetime.fromtimestamp(v).strftime('%Y-%m-%d %H:%M:%S.%f')
                          if isinstance(v, (int, float)) else "")
            elif f in float_fields_to_format:
                out[f] = (f"{v:.4f}" if isinstance(v, (int, float)) else "")
            else:
                out[f] = v if v is not None else ""
        return out

    def generate():
        # csv writer + reusable buffer
        buf = io.StringIO(newline='')
        writer = csv.DictWriter(buf, fieldnames=csv_fieldnames)

        # 1) Write header now and account for its size
        writer.writeheader()
        header_chunk = buf.getvalue()
        header_bytes = len(header_chunk.encode('utf-8'))
        yield header_chunk
        buf.seek(0); buf.truncate(0)

        # Snapshot rows (don’t hold the lock while formatting)
        with historical_data_lock:
            rows = tuple(historical_data_log)

        # 2) Find the start index of the newest slice that fits under MAX_CSV_BYTES
        budget = max(0, MAX_CSV_BYTES - header_bytes)
        start_idx = 0
        if rows and budget > 0:
            # Walk backward measuring exact CSV byte size per row until we'd exceed budget
            idx = len(rows) - 1
            while idx >= 0:
                writer.writerow(fmt_row(rows[idx]))
                row_bytes = len(buf.getvalue().encode('utf-8'))
                buf.seek(0); buf.truncate(0)
                if row_bytes > budget:
                    # this row would exceed the budget → stop; start after this row
                    break
                budget -= row_bytes
                idx -= 1
            start_idx = max(0, idx + 1)

        # 3) Stream rows forward from start_idx, never crossing MAX_CSV_BYTES
        total = header_bytes
        for i in range(start_idx, len(rows)):
            writer.writerow(fmt_row(rows[i]))
            chunk = buf.getvalue()
            b = len(chunk.encode('utf-8'))
            if total + b > MAX_CSV_BYTES:
                break
            yield chunk
            total += b
            buf.seek(0); buf.truncate(0)

    ts = datetime.now().strftime('%Y%m%d_%H%M%S')
    fname = f"remc_telemetry_log_{ts}.csv"
    return Response(generate(),
                    mimetype="text/csv",
                    headers={"Content-Disposition": f"attachment; filename={fname}"})



# --- Helper Functions for HTML Generation ---
def get_measurements_table_html():
    return """
    <div class='section'>
        <h2>Measurements & Data</h2>
        <table id="data-table">
            <thead><tr><th>Signal</th><th>Value</th></tr></thead>
            <tbody>
                <tr>
                  <td>Switch Voltage</td>
                  <td id="payload-switch_voltage_kv">0.00 kV</td>
                </tr>
                <tr>
                  <td>Switch Current</td>
                  <td id="payload-switch_current_a">0.00 A</td>
                </tr>
                <tr>
                  <td>Output Voltage A</td>
                  <td id="payload-output_voltage_a_kv">0.00 kV</td>
                </tr>
                <tr>
                  <td>Output Voltage B</td>
                  <td id="payload-output_voltage_b_kv">0.00 kV</td>
                </tr>
                <tr>
                  <td>Temperature 1</td>
                  <td id="payload-temperature_1_degc">0.00 °C</td>
                </tr>
            </tbody>
        </table>
    </div>
    """


def get_header_table_html():
    return """
    <div class='section'>
        <h2>Neutrino Header Info</h2>
        <table>
            <thead><tr><th>Field</th><th>Value</th></tr></thead>
            <tbody id="header-table-body">
                <tr><td>Msg ID</td><td id="header-msg_id">N/A</td></tr>
                <tr><td>Flags</td><td id="header-flags">N/A</td></tr>
                <tr><td>Num Frags</td><td id="header-num_frags">N/A</td></tr>
                <tr><td>Num Atomic Frags</td><td id="header-num_atomic_frags">N/A</td></tr>
                <tr><td>Schema Hash</td><td id="header-schema_hash">N/A</td></tr>
                <tr><td>Frag Idx</td><td id="header-frag_idx">N/A</td></tr>
                <tr><td>Atomic Idx</td><td id="header-atomic_idx">N/A</td></tr>
                <tr><td>Timestamp (ns)</td><td id="header-timestamp_ns">N/A</td></tr>
            </tbody>
        </table>
    </div>
    """


def get_javascript_html():
    return """
    <script>
      const POLLING_INTERVAL_MS = 500;
      let manualMode = false;
      
      // Live Feed Chart Variables
      let liveFeedChart = null;
      const chartDataSeries = {
        'switch_voltage_kv': { label: 'Switch Voltage (kV)', color: '#ff6384', visible: true },
        'switch_current_a': { label: 'Switch Current (kA)', color: '#36a2eb', visible: true },
        'output_voltage_a_kv': { label: 'Output A (kV)', color: '#ffce56', visible: true },
        'output_voltage_b_kv': { label: 'Output B (kV)', color: '#4bc0c0', visible: true },
        'temperature_1_degc': { label: 'Temperature (°C)', color: '#9966ff', visible: true }
      };

      async function sendCommand(endpoint) {
        try {
          const res = await fetch(endpoint, { method: 'POST' });
          if (!res.ok) console.error(`POST ${endpoint} →`, res.status);
        } catch (err) {
          console.error('Error sending', endpoint, err);
        }
      }

      function updateElementText(id, value) {
        const el = document.getElementById(id);
        if (!el) return;
        const txt = (value !== null && value !== undefined) ? String(value) : 'N/A';
        if (el.innerText !== txt) el.innerText = txt;
      }

      function updateElementClass(id, base, status) {
        const el = document.getElementById(id);
        if (!el) return;
        const cls = base ? `${base} ${status}` : status;
        if (el.className !== cls) el.className = cls;
      }

      function formatFloat(v) {
        return (typeof v === 'number' && !isNaN(v)) ? v.toFixed(2) : 'N/A';
      }

      function initializeLiveFeedChart() {
        const ctx = document.getElementById('liveFeedChart').getContext('2d');
        
        liveFeedChart = new Chart(ctx, {
          type: 'line',
          data: {
            datasets: Object.keys(chartDataSeries).map(key => ({
              label: chartDataSeries[key].label,
              data: [],
              borderColor: chartDataSeries[key].color,
              backgroundColor: chartDataSeries[key].color + '20',
              fill: false,
              tension: 0.1,
              pointRadius: 2,
              pointHoverRadius: 4,
              hidden: !chartDataSeries[key].visible
            }))
          },
          options: {
            responsive: true,
            maintainAspectRatio: false,
            interaction: {
              intersect: false,
              mode: 'index'
            },
            scales: {
              x: {
                type: 'time',
                time: {
                  unit: 'second',
                  displayFormats: {
                    second: 'HH:mm:ss'
                  }
                },
                title: {
                  display: true,
                  text: 'Time'
                }
              },
              y: {
                title: {
                  display: true,
                  text: 'Value'
                }
              }
            },
            plugins: {
              legend: {
                display: true,
                position: 'top'
              },
              tooltip: {
                callbacks: {
                  title: function(context) {
                    return new Date(context[0].parsed.x).toLocaleTimeString();
                  },
                  label: function(context) {
                    return context.dataset.label + ': ' + context.parsed.y.toFixed(3);
                  }
                }
              }
            }
          }
        });
      }

      async function updateLiveFeedChart() {
        try {
          const res = await fetch('/live_feed');
          if (!res.ok) throw new Error(res.status);
          const data = await res.json();

          if (liveFeedChart && data.samples) {
            // Prepare data for each series
            const seriesData = {};
            Object.keys(chartDataSeries).forEach(key => {
              seriesData[key] = data.samples
                .filter(sample => sample[key] !== null && sample[key] !== undefined)
                .map(sample => ({
                  x: sample.timestamp * 1000, // Convert to milliseconds for Chart.js
                  y: sample[key]
                }))
                .sort((a, b) => a.x - b.x); // Sort by time
            });

            // Update chart datasets
            liveFeedChart.data.datasets.forEach((dataset, index) => {
              const seriesKey = Object.keys(chartDataSeries)[index];
              dataset.data = seriesData[seriesKey] || [];
            });

            liveFeedChart.update('none'); // Update without animation for smoother performance
          }
        } catch (err) {
          console.error('Error updating live feed chart:', err);
        }
      }

      function toggleDataSeries(seriesKey) {
        const seriesIndex = Object.keys(chartDataSeries).indexOf(seriesKey);
        if (seriesIndex >= 0 && liveFeedChart) {
          const dataset = liveFeedChart.data.datasets[seriesIndex];
          dataset.hidden = !dataset.hidden;
          chartDataSeries[seriesKey].visible = !dataset.hidden;
          
          // Force a complete redraw to clean up any visual artifacts
          liveFeedChart.update('none');
          
          // Update button appearance
          const button = document.querySelector(`[data-series="${seriesKey}"]`);
          if (button) {
            button.classList.toggle('active', !dataset.hidden);
          }
        }
      }

      document.addEventListener('DOMContentLoaded', () => {
        // wire main buttons
        const armBtn  = document.getElementById('arm-button');
        const fireBtn = document.getElementById('fire-button');

        if (armBtn) {
          armBtn.addEventListener('click', () => {
            sendCommand('/trigger_arm');
            armBtn.disabled = true;
          });
        }
        if (fireBtn) {
          fireBtn.addEventListener('click', () => {
            // Simple fire command - no timing window
            sendCommand('/trigger_fire');
            if (armBtn) armBtn.disabled = false;
          });
        }

        document.getElementById('set-switch-button')?.addEventListener('click', () => sendCommand('/set_switch'));
        const manualBtn = document.getElementById('manual-mode-button');
        if (manualBtn) {
          manualBtn.addEventListener('click', (e) => {
            e.preventDefault();
            const endpoint = manualMode ? '/mode_auto' : '/mode_manual';
            sendCommand(endpoint);
          });
        }

        // wire manual engage/disengage
        const eng = document.getElementById('manual-engage-button'),
              dis = document.getElementById('manual-disengage-button');
        if (eng) {
          eng.addEventListener('mousedown',  e => { e.preventDefault(); sendCommand('/manual_engage_start'); });
          eng.addEventListener('mouseup',    () => sendCommand('/manual_actuator_stop'));
          eng.addEventListener('mouseleave', () => sendCommand('/manual_actuator_stop'));
          eng.addEventListener('touchstart', e => { e.preventDefault(); sendCommand('/manual_engage_start'); }, { passive: false });
          eng.addEventListener('touchend',   e => { e.preventDefault(); sendCommand('/manual_actuator_stop'); });
        }
        if (dis) {
          dis.addEventListener('mousedown',  e => { e.preventDefault(); sendCommand('/manual_disengage_start'); });
          dis.addEventListener('mouseup',    () => sendCommand('/manual_actuator_stop'));
          dis.addEventListener('mouseleave', () => sendCommand('/manual_actuator_stop'));
          dis.addEventListener('touchstart', e => { e.preventDefault(); sendCommand('/manual_disengage_start'); }, { passive: false });
          dis.addEventListener('touchend',   e => { e.preventDefault(); sendCommand('/manual_actuator_stop'); });
        }

        // wire collect button
        const collectBtn = document.getElementById('collect-button');
        if (collectBtn) {
          collectBtn.addEventListener('click', () => {
            sendCollectCommand();
          });
        }

        // wire live feed toggle buttons
        document.querySelectorAll('.toggle-btn').forEach(btn => {
          btn.addEventListener('click', () => {
            const seriesKey = btn.getAttribute('data-series');
            toggleDataSeries(seriesKey);
          });
        });

        // initialize live feed chart
        initializeLiveFeedChart();

        // start polling
        fetchDataAndUpdate();
        fetchBatchesAndUpdate();
        updateLiveFeedChart(); // Initial chart update
        setInterval(fetchDataAndUpdate, POLLING_INTERVAL_MS);
        setInterval(fetchBatchesAndUpdate, 2000); // Update batches every 2 seconds
        setInterval(updateLiveFeedChart, 2000); // Update live feed chart every 2 seconds
      });

      async function sendCollectCommand() {
        try {
          // Read current field values
          const startInput = document.getElementById('sample-start');
          const stopInput = document.getElementById('sample-stop');
          
          if (!startInput || !stopInput) {
            console.error('Sample range input fields not found');
            return;
          }
          
          const start = parseInt(startInput.value);
          const stop = parseInt(stopInput.value);
          
          // Validate range
          if (isNaN(start) || isNaN(stop)) {
            alert('Please enter valid numbers for both start and stop samples.');
            return;
          }
          
          if (stop <= start) {
            alert('Stop sample must be greater than start sample.');
            return;
          }
          
          console.log(`Collecting samples from ${start} to ${stop}`);
          
          const res = await fetch('/trigger_collect', {
            method: 'POST',
            headers: {
              'Content-Type': 'application/json',
            },
            body: JSON.stringify({
              sample_start: start,
              sample_stop: stop
            })
          });
          
          if (!res.ok) {
            const errorData = await res.json().catch(() => ({ error: 'Unknown error' }));
            console.error('POST /trigger_collect →', res.status, errorData);
            alert(`Error: ${errorData.error || 'Failed to send collect command'}`);
          } else {
            const responseData = await res.json();
            console.log('Collect command sent successfully:', responseData);
          }
        } catch (err) {
          console.error('Error sending collect command', err);
          alert('Error sending collect command: ' + err.message);
        }
      }

      async function fetchBatchesAndUpdate() {
        try {
          const res = await fetch('/batches');
          if (!res.ok) throw new Error(res.status);
          const data = await res.json();

          const batchesList = document.getElementById('batches-list');
          if (!batchesList) return;

          if (data.batches.length === 0) {
            batchesList.innerHTML = '<p style="color: #666;">No batches collected yet</p>';
          } else {
            let html = '<table style="width: 100%; border-collapse: collapse; font-size: 12px;"><thead><tr style="background-color: #f5f5f5;"><th style="padding: 8px; text-align: left;">Batch #</th><th style="padding: 8px; text-align: left;">Time</th><th style="padding: 8px; text-align: left;">Samples</th><th style="padding: 8px; text-align: left;">Download</th></tr></thead><tbody>';
            
            data.batches.forEach(batch => {
              html += `<tr>
                <td style="padding: 8px;">#${batch.id}</td>
                <td style="padding: 8px;">${batch.timestamp_iso}</td>
                <td style="padding: 8px;">${batch.sample_count.toLocaleString()}</td>
                <td style="padding: 8px;"><a href="/download_batch/${batch.id}" class="button download" style="padding: 4px 8px; font-size: 11px; text-decoration: none;">Download CSV</a></td>
              </tr>`;
            });
            
            html += '</tbody></table>';
            
            if (data.capture_active) {
              html += `<p style="color: #ff6600; font-weight: bold; margin-top: 10px;">Collecting batch #${data.current_batch_id}... (${data.current_batch_samples} samples captured)</p>`;
            }
            
            batchesList.innerHTML = html;
          }
        } catch (err) {
          console.error('fetchBatches error', err);
          const batchesList = document.getElementById('batches-list');
          if (batchesList) {
            batchesList.innerHTML = '<p style="color: #ff0000;">Error loading batches</p>';
          }
        }
      }

      async function fetchDataAndUpdate() {
        try {
          const res = await fetch('/data');
          if (!res.ok) throw new Error(res.status);
          const d = await res.json();

          // top‐line status
          const stale = d.stale;
          updateElementText('receiver-status', d.status);
          ['arm-button','set-switch-button','fire-button','manual-mode-button',
           'manual-engage-button','manual-disengage-button','em-toggle-button']
            .forEach(id => {
              const b = document.getElementById(id);
              if (b) b.disabled = stale;
            });
          updateElementText('last-update',    d.last_update_str);
          updateElementText('sender-ip',      d.sender_ip);
          updateElementText('packet-count',   d.packets_received);
          updateElementText('sample-count',   d.samples_received);
          updateElementText('bundle-size',    d.last_bundle_size);

          const p = d.payload || {}, h = d.header || {};
          const manual   = p.manual_mode_status === 1;
          manualMode = manual;
          const armed    = p.armed_status === 1;
          const emOn     = p.em_status === 1;
          const mswB_low = p.msw_b_status === 0; 

          const holdMode = p.hold_mode_status === 1;
          updateElementText('hold-mode-status-text', holdMode ? 'ON' : 'OFF');
          updateElementClass('hold-mode-status-text', '', holdMode ? 'on' : 'off');


          // show/hide manual block
          document.querySelectorAll('.manual-only')
                  .forEach(el => el.style.display = manual ? 'block' : 'none');

          // mode & state
          const modeBtn = document.getElementById('manual-mode-button');
          if (modeBtn) modeBtn.innerText = manual ? 'Switch to Auto Mode' : 'Switch to Manual Mode';
          updateElementText('system-mode-text', manual ? 'MANUAL MODE' : 'AUTO MODE');
          updateElementClass('system-mode-text','', manual ? 'manual-active' : 'auto-active');
          updateElementText('fsm-state-display', manual ? 'MANUAL' : 'AUTO');

          // main controls
          const armBtnElem = document.getElementById('arm-button');
          const setBtn     = document.getElementById('set-switch-button');
          if (armBtnElem) {
            armBtnElem.disabled = manual;
            armBtnElem.innerText = armed ? 'DISARM System' : 'ARM System';
            armBtnElem.className = armed ? 'disarm' : 'arm';
          }
          if (setBtn) setBtn.disabled = manual;

          // EM status
          updateElementText('em-status-text', emOn ? 'ON' : 'OFF');
          updateElementClass('em-status-text', '', emOn ? 'on' : 'off');

          // manual buttons enable/disable
          const engElem = document.getElementById('manual-engage-button'),
                disElem = document.getElementById('manual-disengage-button');
          if (engElem) engElem.disabled = !manual;
          if (disElem) disElem.disabled = !manual || mswB_low;

          // armed & MSW
          let armedText, armedClass;
          if (armed) {
            armedText  = 'ARMED';
            armedClass = 'on';
          } else if ((!armed && p.msw_a_status === 1 && p.msw_b_status === 1 && p.em_status === 1)) {
            armedText  = 'ARMING';
            armedClass = 'arming';
          } else {
            armedText  = 'NOT ARMED';
            armedClass = 'off';
          }
          updateElementText('armed-status-text', armedText);
          updateElementClass('armed-status-text','', armedClass);
          updateElementText('msw-a-status-text', p.msw_a_status === 0 ? 'LOW' : 'HIGH');
          updateElementText('msw-b-status-text', p.msw_b_status === 0 ? 'LOW' : 'HIGH');

          // ─── Telemetry table (2‐dec + unit) ──────────────────────────────────
          updateElementText('payload-switch_voltage_kv',
                            formatFloat(p.switch_voltage_kv) + ' kV');
          updateElementText('payload-switch_current_a',
                            formatFloat(p.switch_current_a) + ' kA');
          updateElementText('payload-output_voltage_a_kv',
                            formatFloat(p.output_voltage_a_kv) + ' kV');
          updateElementText('payload-output_voltage_b_kv',
                            formatFloat(p.output_voltage_b_kv) + ' kV');
          updateElementText('payload-temperature_1_degc',
                            formatFloat(p.temperature_1_degc) + ' °C');
          // ─────────────────────────────────────────────────────────────────────

          // ─── Header table ────────────────────────────────────────────────────
          updateElementText('header-msg_id',    h.msg_id);
          updateElementText('header-flags',     h.flags);
          updateElementText('header-num_frags', h.num_frags);
          updateElementText('header-num_atomic_frags', h.num_atomic_frags);
          updateElementText('header-schema_hash',      h.schema_hash);
          updateElementText('header-frag_idx',        h.frag_idx);
          updateElementText('header-atomic_idx',      h.atomic_idx);
          const ts = h.timestamp_ns;
          updateElementText('header-timestamp_ns',
                            (typeof ts === 'number') ? `${ts} ns (~${(ts/1e9).toFixed(3)} s)` : 'N/A');
          // ─────────────────────────────────────────────────────────────────────

          // ─── NEW: Current State & State‐related Errors ──────────────────────
          // 1) Update the state name:

          // 2) Check for any state‐specific error flags:
          const flags = p.error_flags ?? 0; // from payload
          let stateErrors = [];
          // If ARM_TIMEOUT bit (bit0) is set but FSM isn't in ARM_START_ENGAGE, show message
          if ((flags & 0x01) && currentStateText !== 'ARM_START_ENGAGE') {
            stateErrors.push('⚠ ARM_TIMEOUT: didn’t reach MSW_A in time');
          }
          // If PULLBACK_TIMEOUT (bit1) but FSM isn't in ARM_PULL_BACK
          if ((flags & 0x02) && currentStateText !== 'ARM_PULL_BACK') {
            stateErrors.push('⚠ PULLBACK_TIMEOUT: didn’t reach MSW_B in time');
          }
          // If RETAIN_FAIL (bit2) but FSM isn't in ARMED_READY
          if ((flags & 0x04) && currentStateText !== 'ARMED_READY') {
            stateErrors.push('⚠ RETAIN_FAIL: EM lost while “armed”');
          }
          // Display either “No issues” or the list of errors
          const stateErrorBox = document.getElementById('state-error-box');
          if (stateErrorBox) {
            if (stateErrors.length === 0) {
              stateErrorBox.innerHTML = '<span style="color:green;">No state errors.</span>';
            } else {
              stateErrorBox.innerHTML = '<ul>' + stateErrors.map(m => `<li>${m}</li>`).join('') + '</ul>';
            }
          }
          // ─────────────────────────────────────────────────────────────────────

        } catch (err) {
          console.error('fetchData error', err);
          updateElementText('receiver-status','Error');
        }
      }
    </script>
    """


@app.route('/')
def index_page():
    html = f"""
    <!DOCTYPE html>
    <html lang="en">
      <head>
        <meta charset="UTF-8">
        <meta name="viewport" content="width=device-width,initial-scale=1.0">
        <title>REMC Switch Dashboard</title>
        <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
        <script src="https://cdn.jsdelivr.net/npm/chartjs-adapter-date-fns/dist/chartjs-adapter-date-fns.bundle.min.js"></script>
 <style>
          body {{
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            margin: 20px;
            background-color: #f0f2f5;
            color: #333;
          }}
          .container {{
            background-color: #fff;
            max-width: 900px;
            margin: 0 auto;
            padding: 20px 30px;
            border-radius: 10px;
            box-shadow: 0 6px 20px rgba(0,0,0,0.08);
          }}
          h1 {{
            text-align: center;
            color: #1a237e;
            margin-bottom: 25px;
          }}
          h2 {{
            color: #3949ab;
            border-bottom: 2px solid #5c6bc0;
            padding-bottom: 10px;
            margin-top: 30px;
          }}
          .button-group {{
            background-color: #e8eaf6;
            padding: 10px;
            border-radius: 8px;
            text-align: center;
            margin-bottom: 20px;
          }}
          .button,
          .button-group button,
          .button-group form button {{
            display: inline-block;
            margin: 5px;
            padding: 10px 18px;
            font-size: 15px;
            border: none;
            border-radius: 6px;
            color: white;
            cursor: pointer;
            transition: background-color 0.2s ease, box-shadow 0.2s ease;
            text-decoration: none;
          }}
          button.arm {{
            background-color: #43a047;
          }}
          button.arm:hover {{
            background-color: #388e3c;
          }}
          button.disarm {{
            background-color: #e53935;
          }}
          button.disarm:hover {{
            background-color: #d32f2f;
          }} 
           button.actuate {{
            background-color: #e53935;
          }}
          button.actuate:hover {{
            background-color: #d32f2f;
          }}

          button.action, a.button.action {{
            background-color: #00897b;
          }}
          button.action:hover, a.button.action:hover {{
            background-color: #00796b;
          }}
          button.toggle {{
            background-color: #1e88e5;
          }}
          button.toggle:hover {{
            background-color: #1565c0;
          }}
          button.manual-action {{
            background-color: #546e7a;
          }}
          button.manual-action:hover {{
            background-color: #455a64;
          }}
          .button.download {{
            background-color: #7e57c2;
          }}
          .button.download:hover {{
            background-color: #673ab7;
          }} 
            .button.download {{
            background-color: #7e57c2;
          }}
          .button.download:hover {{
            background-color: #673ab7;
          }}


          button:disabled, .button:disabled {{
            background-color: #9e9e9e;
            cursor: not-allowed;
          }}
          .manual-only {{ display: none; }}
          .on {{ color: #388e3c; font-weight: bold; }}
          .off {{ color: #c62828; font-weight: bold; }} 
          .arming {{ color: orange; font-weight: bold; }}
          table {{
            width: 100%;
            border-collapse: collapse;
            margin-top: 15px;
            box-shadow: 0 2px 5px rgba(0,0,0,0.05);
          }}
          th, td {{
            text-align: left;
            padding: 10px 12px;
            border-bottom: 1px solid #e0e0e0;
          }}
          th {{
            background-color: #3949ab;
            color: white;
            font-weight: 600;
          }}
          tr:nth-child(even) {{
            background-color: #f9f9f9;
          }}
          
          /* Live Feed Chart Styles */
          .live-feed-controls {{
            margin-bottom: 20px;
          }}
          .toggle-buttons {{
            display: flex;
            flex-wrap: wrap;
            gap: 8px;
            margin-top: 10px;
          }}
          .toggle-btn {{
            padding: 6px 12px;
            font-size: 12px;
            border: 2px solid;
            border-radius: 4px;
            cursor: pointer;
            transition: all 0.2s ease;
            margin: 2px;
            font-weight: 500;
            opacity: 0.6;
            min-width: 120px;
            text-align: center;
          }}
          .toggle-btn.active {{
            opacity: 1;
            font-weight: 500;
            box-shadow: 0 2px 4px rgba(0,0,0,0.2);
          }}
          .toggle-btn:hover {{
            opacity: 0.8;
            transform: translateY(-1px);
          }}
          .toggle-btn.active:hover {{
            opacity: 1;
            transform: translateY(-1px);
          }}
          
          /* Individual series button colors - INACTIVE state */
          #toggle-voltage {{
            border-color: #ff6384;
            background-color: #fff;
            color: #ff6384;
            font-weight: 500;
            min-width: 120px;
            text-align: center;
          }}
          #toggle-voltage.active {{
            background-color: #ff6384;
            color: white;
            box-shadow: 0 2px 4px rgba(0,0,0,0.2);
            font-weight: 500;
            min-width: 120px;
            text-align: center;
          }}
          
          #toggle-current {{
            border-color: #36a2eb;
            background-color: #fff;
            color: #36a2eb;
            font-weight: 500;
            min-width: 120px;
            text-align: center;
          }}
          #toggle-current.active {{
            background-color: #36a2eb;
            color: white;
            box-shadow: 0 2px 4px rgba(0,0,0,0.2);
            font-weight: 500;
            min-width: 120px;
            text-align: center;
          }}
          
          #toggle-output-a {{
            border-color: #ffce56;
            background-color: #fff;
            color: #ffce56;
            font-weight: 500;
            min-width: 120px;
            text-align: center;
          }}
          #toggle-output-a.active {{
            background-color: #ffce56;
            color: #333;
            box-shadow: 0 2px 4px rgba(0,0,0,0.2);
            font-weight: 500;
            min-width: 120px;
            text-align: center;
          }}
          
          #toggle-output-b {{
            border-color: #4bc0c0;
            background-color: #fff;
            color: #4bc0c0;
            font-weight: 500;
            min-width: 120px;
            text-align: center;
          }}
          #toggle-output-b.active {{
            background-color: #4bc0c0;
            color: white;
            box-shadow: 0 2px 4px rgba(0,0,0,0.2);
            font-weight: 500;
            min-width: 120px;
            text-align: center;
          }}
          
          #toggle-temp {{
            border-color: #9966ff;
            background-color: #fff;
            color: #9966ff;
            font-weight: 500;
            min-width: 120px;
            text-align: center;
          }}
          #toggle-temp.active {{
            background-color: #9966ff;
            color: white;
            box-shadow: 0 2px 4px rgba(0,0,0,0.2);
            font-weight: 500;
            min-width: 120px;
            text-align: center;
          }}
          .chart-container {{
            position: relative;
            height: 400px;
            margin: 20px 0;
            background-color: #f9f9f9;
            border: 1px solid #e0e0e0;
            border-radius: 8px;
            padding: 10px;
          }}
          .chart-container canvas {{
            max-width: 100%;
            height: auto;
          }}
        </style>
      </head>
      <body>
        <div class="container">
          <h1>REMC Switch Dashboard</h1>

        <h2>Main Controls (Auto Mode)</h2>
        <div class="button-group"> 
        <button id="arm-button" type="button" class="arm">ARM System</button>
        <button id="set-switch-button" type="button" class="action">Set Switch to Idle</button>
        <button id="fire-button" type="button" class="actuate">FIRE SWITCH</button>
        </div>

        <h2>Toggle System Mode</h2>
        <div class="button-group">
        <button id="manual-mode-button" type="button" class="toggle">Toggle Manual/Auto Mode</button>
          <p>Current Mode: <strong id="system-mode-text" class="auto-active">AUTO MODE (FSM)</strong></p>
        </div> 

        <h2>Special Modes</h2>
        <div class="button-group">
          <form action="/hold_mode_enable" method="post" style="display:inline;">
            <button type="submit" class="toggle">Enable Hold-After-Fire Mode</button>
          </form>
          <form action="/hold_mode_disable" method="post" style="display:inline;">
            <button type="submit" class="toggle">Disable Hold-After-Fire Mode</button>
          </form>
        </div>

        <h2>Manual Controls</h2>
        <div class="button-group manual-only">
          <button id="manual-engage-button" class="manual-action">ENGAGE (Hold)</button>
          <button id="manual-disengage-button" class="manual-action">DISENGAGE (Hold)</button>
          <form action="/manual_em_toggle" method="post" style="display:inline;">
            <button id="em-toggle-button" class="toggle">Toggle EM</button>
          </form>
        </div>



          <h2>Data Collection</h2>
          <div class="button-group">
            <h3>Live Feed</h3>
            <div class="live-feed-controls">
              <div class="toggle-buttons">
                <button type="button" id="toggle-voltage" class="toggle-btn active" data-series="switch_voltage_kv">Switch Voltage (kV)</button>
                <button type="button" id="toggle-current" class="toggle-btn active" data-series="switch_current_a">Switch Current (kA)</button>
                <button type="button" id="toggle-output-a" class="toggle-btn active" data-series="output_voltage_a_kv">Output A (kV)</button>
                <button type="button" id="toggle-output-b" class="toggle-btn active" data-series="output_voltage_b_kv">Output B (kV)</button>
                <button type="button" id="toggle-temp" class="toggle-btn active" data-series="temperature_1_degc">Temperature (°C)</button>
              </div>
            </div>
            <div class="chart-container">
              <canvas id="liveFeedChart" width="800" height="400"></canvas>
            </div>
            <h3>Dense Sampling</h3>
            <div style="margin: 10px 0;">
              <label for="sample-start" style="margin-right: 10px;">Start at sample:</label>
              <input type="number" id="sample-start" name="sample_start" value="-1000" 
                     style="width: 100px; padding: 5px; margin-right: 15px;">
              
              <label for="sample-stop" style="margin-right: 10px;">End at sample:</label>
              <input type="number" id="sample-stop" name="sample_stop" value="1000" 
                     style="width: 100px; padding: 5px; margin-right: 15px;">
            </div>
            <p style="font-size: 12px; color: #666; margin: 5px 0;">
              The system maintains a continuous data buffer. Sample 0 represents the moment you click "Collect".<br>
              Use negative numbers to capture data from before you clicked, positive numbers for after.<br>
              <strong>Example:</strong> To capture 0.5 seconds before and after, use -5000 to +5000 (10,000 samples/sec)
            </p>
            <div style="margin-top: 15px;">
              <button type="button" id="collect-button" class="button download">Collect Batch</button>
            </div>
          <hr style="width: 95%; margin: 20px auto; border: 0; border-top: 1px solid #ccc;">
            <div id="batches-list" style="margin: 10px 0;">
              <p style="color: #666;">Loading batches...</p>
            </div>
          </div>
    
          <div class="section">
            <h2>System Status</h2>
            <!-- status lines and the tables below -->
            <p>Receiver Status: <strong id="receiver-status">Initializing...</strong></p>
            <p>Last Packet:     <strong id="last-update">N/A</strong></p>
            <p>Packets:         <strong id="packet-count">0</strong></p>
            <p>Samples:         <strong id="sample-count">0</strong></p>
            <p>Last Bundle:     <strong id="bundle-size">0</strong> samples</p>
            <p>FSM State:       <strong id="fsm-state-display">N/A</strong></p>
            <p>Armed:           <strong id="armed-status-text" class="off">NOT ARMED</strong></p> 
            <p>EM Coil:         <strong id="em-status-text" class="off">OFF</strong></p> 
            <p>Hold Mode: <strong id="hold-mode-status-text" class="off">OFF</strong></p>
            <div class="section">
            <h2>Current State</h2>
            <div id="state-error-box" style="padding: 8px; border: 1px solid #e0e0e0; background-color: #fff3e0; color: #e65100; margin-top: 5px;">
              <!-- Any state‐related errors will appear here -->
            </div>
          </div>
            <p>MSW A:           <strong id="msw-a-status-text">N/A</strong></p>
            <p>MSW B:           <strong id="msw-b-status-text">N/A</strong></p> 
          </div>

          {get_measurements_table_html()}
          {get_header_table_html()}
        </div>

        {get_javascript_html()}
      </body>
    </html>
    """
    return (html
            .replace("{get_measurements_table_html()}", get_measurements_table_html())
            .replace("{get_header_table_html()}", get_header_table_html())
            .replace("{get_javascript_html()}", get_javascript_html())
            )


# --- Command Sending Function ---
# REMC_WEBPAGE.py


def send_udp_command(command_byte):
    try:
        with socket.socket(socket.AF_INET,
                           socket.SOCK_DGRAM,
                           socket.IPPROTO_UDP) as cmd_sock:
            cmd_sock.setsockopt(socket.SOL_SOCKET,
                                socket.SO_REUSEADDR, 1)
            cmd_sock.bind((MULTICAST_INTERFACE_IP, 0))
            cmd_sock.setsockopt(socket.IPPROTO_IP,
                                socket.IP_MULTICAST_IF,
                                socket.inet_aton(MULTICAST_INTERFACE_IP))
            cmd_sock.setsockopt(socket.IPPROTO_IP,
                                socket.IP_MULTICAST_TTL, 2)
            cmd_sock.setsockopt(socket.IPPROTO_IP,
                                socket.IP_MULTICAST_LOOP, 1)

            packet = b'\x00' * HEADER_SIZE + command_byte
            cmd_sock.sendto(packet,
                            (MULTICAST_GROUP_CMND, PORT_CMND))
            print(f"Sent command {command_byte.hex()} "
                  f"to {MULTICAST_GROUP_CMND}:{PORT_CMND}")

    except socket.error as e:
        print(f"Socket error sending command {command_byte.hex()}: {e}")
    except Exception as e:
        print(f"An unexpected error occurred in send_udp_command: {e}")


def send_collect_command_with_range(sample_start, sample_stop):
    """Send collect command with sample range parameters"""
    try:
        with socket.socket(socket.AF_INET,
                           socket.SOCK_DGRAM,
                           socket.IPPROTO_UDP) as cmd_sock:
            cmd_sock.setsockopt(socket.SOL_SOCKET,
                                socket.SO_REUSEADDR, 1)
            cmd_sock.bind((MULTICAST_INTERFACE_IP, 0))
            cmd_sock.setsockopt(socket.IPPROTO_IP,
                                socket.IP_MULTICAST_IF,
                                socket.inet_aton(MULTICAST_INTERFACE_IP))
            cmd_sock.setsockopt(socket.IPPROTO_IP,
                                socket.IP_MULTICAST_TTL, 2)
            cmd_sock.setsockopt(socket.IPPROTO_IP,
                                socket.IP_MULTICAST_LOOP, 1)

            # Create packet: 64-byte header + collect command + range parameters
            # Command format: 0x04 (collect) + 4 bytes start (int32) + 4 bytes stop (int32)
            command_payload = struct.pack('<Bii', 0x04, sample_start, sample_stop)
            packet = b'\x00' * HEADER_SIZE + command_payload
            
            cmd_sock.sendto(packet, (MULTICAST_GROUP_CMND, PORT_CMND))
            print(f"Sent collect command with range {sample_start} to {sample_stop} "
                  f"to {MULTICAST_GROUP_CMND}:{PORT_CMND}")

    except socket.error as e:
        print(f"Socket error sending collect command with range: {e}")
    except Exception as e:
        print(f"An unexpected error occurred in send_collect_command_with_range: {e}")


# --- Flask Command Routes ---
@app.route('/trigger_arm', methods=['POST'])
def handle_trigger_arm():
    send_udp_command(b'\x01')
    return redirect(url_for('index_page'))


@app.route('/trigger_disarm', methods=['POST'])
def handle_trigger_disarm():
    send_udp_command(b'\x03')
    return redirect(url_for('index_page'))


@app.route('/trigger_fire', methods=['POST'])
def handle_trigger_fire():
    """Simple fire command - no timing window"""
    try:
        send_udp_command(b'\x02')
        if request.is_json:
            return jsonify(status="fire_command_sent")
        else:
            return redirect(url_for('index_page'))
    except Exception as e:
        print(f"Error in handle_trigger_fire: {e}")
        if request.is_json:
            return jsonify(error=str(e)), 500
        else:
            return redirect(url_for('index_page'))


@app.route('/trigger_collect', methods=['POST'])
def handle_trigger_collect():
    """Collect samples with timing window - bypasses StateManager"""
    global batch_capture_active, current_batch_id, current_batch_samples, batch_capture_start_time, next_batch_id
    
    try:
        if request.is_json:
            data = request.get_json()
            sample_start = data.get('sample_start', -50000)
            sample_stop = data.get('sample_stop', 50000)
            
            # Validate range
            if sample_stop <= sample_start:
                return jsonify(error="Stop must be greater than Start"), 400
            
            # Start batch capture mode
            with batches_lock:
                if batch_capture_active:
                    return jsonify(error="Another batch collection is already in progress"), 400
                
                batch_capture_active = True
                current_batch_id = next_batch_id
                next_batch_id += 1
                current_batch_samples.clear()
                batch_capture_start_time = time.time()
                
            print(f"[BATCH] Starting batch {current_batch_id} collection: {sample_start} to {sample_stop}")
            
            # Send collect command with range parameters
            send_collect_command_with_range(sample_start, sample_stop)
            return jsonify(status="collect_command_sent", 
                         batch_id=current_batch_id,
                         sample_start=sample_start, 
                         sample_stop=sample_stop)
        else:
            # Fallback for form-based requests (use default range)
            with batches_lock:
                if batch_capture_active:
                    return redirect(url_for('index_page'))
                
                batch_capture_active = True
                current_batch_id = next_batch_id
                next_batch_id += 1
                current_batch_samples.clear()
                batch_capture_start_time = time.time()
                
            send_collect_command_with_range(-50000, 50000)
            return redirect(url_for('index_page'))
    except Exception as e:
        print(f"Error in handle_trigger_collect: {e}")
        # Reset batch capture state on error
        with batches_lock:
            batch_capture_active = False
            current_batch_id = None
            current_batch_samples.clear()
            batch_capture_start_time = None
            
        if request.is_json:
            return jsonify(error=str(e)), 500
        else:
            return redirect(url_for('index_page'))


@app.route('/batches')
def get_batches():
    """Return list of available collection batches"""
    with batches_lock:
        batch_list = []
        for batch_id, batch_data in collection_batches.items():
            batch_list.append({
                'id': batch_data['id'],
                'timestamp': batch_data['timestamp'],
                'timestamp_iso': datetime.fromtimestamp(batch_data['timestamp']).strftime('%Y-%m-%d %H:%M:%S'),
                'sample_count': batch_data['sample_count']
            })
        
        # Sort by timestamp (newest first)
        batch_list.sort(key=lambda x: x['timestamp'], reverse=True)
        
        return jsonify({
            'batches': batch_list,
            'capture_active': batch_capture_active,
            'current_batch_id': current_batch_id,
            'current_batch_samples': len(current_batch_samples) if current_batch_samples else 0
        })


@app.route('/download_batch/<int:batch_id>')
def download_batch(batch_id):
    """Download a specific collection batch as CSV"""
    with batches_lock:
        if batch_id not in collection_batches:
            return jsonify(error="Batch not found"), 404
        
        batch_data = collection_batches[batch_id]
    
    csv_fieldnames = LOGGING_DATA_FIELDS
    float_fields_to_format = [
        'switch_voltage_kv', 'switch_current_a', 'output_voltage_a_kv',
        'output_voltage_b_kv', 'temperature_1_degc'
    ]

    def fmt_row(src):
        out = {}
        for f in csv_fieldnames:
            v = src.get(f)
            if f == 'timestamp':
                out[f] = (datetime.fromtimestamp(v).strftime('%Y-%m-%d %H:%M:%S.%f')
                          if isinstance(v, (int, float)) else "")
            elif f in float_fields_to_format:
                out[f] = (f"{v:.4f}" if isinstance(v, (int, float)) else "")
            else:
                out[f] = v if v is not None else ""
        return out

    def generate():
        # csv writer + reusable buffer
        buf = io.StringIO(newline='')
        writer = csv.DictWriter(buf, fieldnames=csv_fieldnames)

        # Write header
        writer.writeheader()
        yield buf.getvalue()
        buf.seek(0); buf.truncate(0)

        # Write batch samples
        for sample in batch_data['samples']:
            writer.writerow(fmt_row(sample))
            yield buf.getvalue()
            buf.seek(0); buf.truncate(0)

    ts = datetime.fromtimestamp(batch_data['timestamp']).strftime('%Y%m%d_%H%M%S')
    fname = f"remc_batch_{batch_id}_{ts}.csv"
    return Response(generate(),
                    mimetype="text/csv",
                    headers={"Content-Disposition": f"attachment; filename={fname}"})


@app.route('/set_switch', methods=['POST'])
def handle_set_switch():
    send_udp_command(b'\x03')
    return redirect(url_for('index_page'))


@app.route('/manual_engage_start', methods=['POST'])
def handle_manual_engage_start():
    send_udp_command(b'\x11')
    return jsonify(status="engage_start_sent", message="Engage start command sent.")


@app.route('/manual_actuator_stop', methods=['POST'])
def handle_manual_actuator_stop():
    send_udp_command(b'\x12')
    return jsonify(status="actuator_stop_sent", message="Actuator stop command sent.")


@app.route('/manual_disengage_start', methods=['POST'])
def handle_manual_disengage_start():
    send_udp_command(b'\x13')
    return jsonify(status="disengage_start_sent", message="Disengage start command sent.")


@app.route('/mode_manual', methods=['POST'])
def handle_mode_manual():
    send_udp_command(b'\x1F')
    return redirect(url_for('index_page'))


@app.route('/mode_auto', methods=['POST'])
def handle_mode_auto():
    send_udp_command(b'\x1E')
    return redirect(url_for('index_page'))


@app.route('/hold_mode_enable', methods=['POST'])
def handle_hold_mode_enable():
    send_udp_command(b'\x20')
    return redirect(url_for('index_page'))


@app.route('/hold_mode_disable', methods=['POST'])
def handle_hold_mode_disable():
    send_udp_command(b'\x21')
    return redirect(url_for('index_page'))


@app.route('/manual_em_enable', methods=['POST'])
def handle_manual_em_enable():
    send_udp_command(b'\x15')
    return redirect(url_for('index_page'))


@app.route('/manual_em_disable', methods=['POST'])
def handle_manual_em_disable():
    send_udp_command(b'\x16')
    return redirect(url_for('index_page'))


# --- Main Execution ---
if __name__ == "__main__":
    print("Starting UDP listener thread for telemetry...")
    telemetry_listener = threading.Thread(target=udp_listener_thread, daemon=True)
    telemetry_listener.start()
    
    
# Data logging thread disabled (logging per-sample occurs in UDP listener)
# print(f"Starting data logging thread (interval: {DATA_LOGGING_INTERVAL_SECONDS}s)...")
# data_logger = threading.Thread(target=data_logging_thread_function, daemon=True)
# data_logger.start()
    print(f"Starting Flask web server. Access at: http://{WEB_SERVER_HOST}:{WEB_SERVER_PORT}")
    if USE_WAITRESS:
        print("Using Waitress as the production WSGI server.")
        serve(app, host=WEB_SERVER_HOST, port=WEB_SERVER_PORT, threads=6)
    else:
        print("Using Flask's built-in development server.")
        app.run(host=WEB_SERVER_HOST, port=WEB_SERVER_PORT, debug=False)
