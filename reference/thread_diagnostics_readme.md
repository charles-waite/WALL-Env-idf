# Thread Diagnostics (`0x0035`) Quick Test With `chip-tool`

This is a ground-truth check for Matter Thread diagnostics before relying on Home Assistant UI exposure.

## 1) Prerequisites

- Device is commissioned and on Thread.
- Your Mac/Linux host is on the same LAN as the Thread Border Router.
- `chip-tool` is installed and runnable.
- You know:
  - `NODE_ID` (your device node id on that fabric)
  - `EP` (endpoint hosting Thread Network Diagnostics; root endpoint is usually `0`)

## 2) Confirm command syntax on your `chip-tool` build

```bash
chip-tool any read-by-id --help
chip-tool any subscribe-by-id --help
```

Different chip-tool versions vary slightly in argument order/flags. Use the local help as canonical.

## 3) One-shot reads (selected attributes)

Use cluster `0x0035` (Thread Network Diagnostics):

```bash
# routingRole (0x0001)
chip-tool any read-by-id 0x0035 0x0001 ${EP} ${NODE_ID}

# neighborTable (0x0007) - includes averageRssi/lastRssi/lqi/isChild
chip-tool any read-by-id 0x0035 0x0007 ${EP} ${NODE_ID}

# routeTable (0x0008) - includes routerId/nextHop/pathCost/linkEstablished
chip-tool any read-by-id 0x0035 0x0008 ${EP} ${NODE_ID}

# parentChangeCount (0x0015)
chip-tool any read-by-id 0x0035 0x0015 ${EP} ${NODE_ID}

# txRetryCount (0x0021)
chip-tool any read-by-id 0x0035 0x0021 ${EP} ${NODE_ID}

# txErrCcaCount (0x0024)
chip-tool any read-by-id 0x0035 0x0024 ${EP} ${NODE_ID}

# rxErrNoFrameCount (0x0032)
chip-tool any read-by-id 0x0035 0x0032 ${EP} ${NODE_ID}

# rxErrUnknownNeighborCount (0x0033)
chip-tool any read-by-id 0x0035 0x0033 ${EP} ${NODE_ID}

# rxErrInvalidSrcAddrCount (0x0034)
chip-tool any read-by-id 0x0035 0x0034 ${EP} ${NODE_ID}

# rxErrSecCount (0x0035)
chip-tool any read-by-id 0x0035 0x0035 ${EP} ${NODE_ID}

# rxErrFcsCount (0x0036)
chip-tool any read-by-id 0x0035 0x0036 ${EP} ${NODE_ID}

# rxErrOtherCount (0x0037)
chip-tool any read-by-id 0x0035 0x0037 ${EP} ${NODE_ID}
```

## 4) Subscribe to trend over time

Use min/max report intervals that fit your test (example: 2s min, 60s max).

```bash
# parentChangeCount trend
chip-tool any subscribe-by-id 0x0035 0x0015 2 60 ${EP} ${NODE_ID}

# txRetryCount trend
chip-tool any subscribe-by-id 0x0035 0x0021 2 60 ${EP} ${NODE_ID}

# txErrCcaCount trend
chip-tool any subscribe-by-id 0x0035 0x0024 2 60 ${EP} ${NODE_ID}
```

Run one subscription per terminal tab for easier reading.

## 5) Interpreting quickly

- `routingRole`: `child`, `router`, `leader` transitions are expected in some topologies.
- `neighborTable`: more negative RSSI means weaker signal (e.g. `-85 dBm` weaker than `-65 dBm`).
- `parentChangeCount`: rising quickly implies unstable parent link/roaming.
- `txRetryCount` and `txErrCcaCount`: increasing rapidly suggests RF contention/weak links.
- `rxErr*` counters: persistent growth can indicate interference, collisions, or malformed/noisy traffic.

## 6) If HA does not show these values

- If `chip-tool` reads/subscriptions succeed: firmware + Matter cluster are working; HA UI/entity mapping is the gap.
- If `chip-tool` fails too: troubleshoot endpoint/cluster availability and commissioning/fabric/reachability first.
