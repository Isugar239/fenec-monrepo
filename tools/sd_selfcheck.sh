#!/usr/bin/env bash
# Health check for the boot disk (microSD / eMMC / NVMe) on Raspberry Pi.
# Usage: sd_selfcheck.sh [--bench] [--write]
#   --bench  sequential 32 MiB direct read (may stall a dying card ~45s)
#   --write  extra 8 MiB write+fsync on / (optional, extra wear)

set -u

BENCH=0
DO_WRITE=0
for arg in "$@"; do
  case "$arg" in
    --bench) BENCH=1 ;;
    --write) DO_WRITE=1 ;;
    -h|--help)
      echo "Usage: $0 [--bench] [--write]"
      exit 0
      ;;
  esac
done

FAIL=0
WARN=0
notes=()

ok()   { printf '  [OK]   %s\n' "$*"; }
warn() { printf '  [WARN] %s\n' "$*"; WARN=1; notes+=("WARN: $*"); }
fail() { printf '  [FAIL] %s\n' "$*"; FAIL=1; notes+=("FAIL: $*"); }
info() { printf '  [..]   %s\n' "$*"; }

sudo_cmd() {
  if [[ "$(id -u)" -eq 0 ]]; then
    "$@"
  else
    sudo -n "$@"
  fi
}

root_src=$(findmnt -no SOURCE /)
root_fstype=$(findmnt -no FSTYPE /)
disk=""
if [[ "$root_src" =~ ^/dev/mmcblk[0-9]+ ]]; then
  disk="${root_src%%p*}"
elif [[ "$root_src" =~ ^/dev/nvme[0-9]+n[0-9]+ ]]; then
  disk="${root_src%%p*}"
elif [[ "$root_src" =~ ^/dev/sd[a-z] ]]; then
  disk="${root_src%%[0-9]*}"
else
  disk=$(lsblk -no PKNAME "$root_src" 2>/dev/null | head -1)
  [[ -n "$disk" ]] && disk="/dev/$disk"
fi

echo "=== storage self-check $(date '+%Y-%m-%d %H:%M:%S %Z') ==="
echo
echo "-- identity --"
info "root: $root_src ($root_fstype)"
info "boot disk: ${disk:-unknown}"
lsblk -o NAME,SIZE,TYPE,FSTYPE,MOUNTPOINT,MODEL,TRAN "$disk" 2>/dev/null | sed 's/^/  /'
if [[ "$disk" == /dev/nvme* ]]; then
  info "this is NVMe SSD"
elif [[ "$disk" == /dev/mmcblk* ]]; then
  kind=$(cat /sys/block/${disk#/dev/}/device/type 2>/dev/null || echo unknown)
  name=$(cat /sys/block/${disk#/dev/}/device/name 2>/dev/null || echo unknown)
  manfid=$(cat /sys/block/${disk#/dev/}/device/manfid 2>/dev/null || echo unknown)
  date_m=$(cat /sys/block/${disk#/dev/}/device/date 2>/dev/null || echo unknown)
  info "MMC type=$kind name=$name manfid=$manfid date=$date_m"
  case "$manfid" in
    0x00001b|0x1b) info "vendor: Samsung" ;;
    0x000003|0x03) info "vendor: SanDisk" ;;
    0x00009f|0x9f) info "vendor: Kingston / Toshiba" ;;
  esac
  if [[ "$kind" == SD ]]; then
    warn "boot is microSD, not SSD. Pi 5 + dying/slow SD = long boot and random no-start"
  fi
  life=$(cat /sys/block/${disk#/dev/}/device/life_time 2>/dev/null || true)
  eol=$(cat /sys/block/${disk#/dev/}/device/pre_eol_info 2>/dev/null || true)
  if [[ -n "${life:-}" ]]; then
    info "eMMC life_time=$life pre_eol=$eol  (hex 0x01=0-10% used ... 0x0B=90-100%)"
  fi
else
  info "not an MMC/NVMe device"
fi
df -hT / /boot/firmware 2>/dev/null | sed 's/^/  /'

echo
echo "-- power / throttle --"
throttled=$(vcgencmd get_throttled 2>/dev/null | awk -F= '{print $2}')
ext5v=$(vcgencmd pmic_read_adc EXT5V_V 2>/dev/null | sed -n 's/.*=\([0-9.]\+\).*/\1/p')
info "throttled=$throttled  EXT5V=${ext5v:-?}V"
if [[ -n "$throttled" ]]; then
  python3 - "$throttled" <<'PY'
import sys
s=sys.argv[1].strip()
t=int(s, 16) if s.startswith("0x") else int(s, 0)
bits={
  0:"Under-voltage NOW",
  1:"ARM freq capped NOW",
  2:"Throttled NOW",
  3:"Soft temp limit NOW",
  16:"Under-voltage has occurred this boot",
  17:"ARM freq capped this boot",
  18:"Throttled this boot",
  19:"Soft temp limit this boot",
}
hit=[v for k,v in bits.items() if t & (1<<k)]
print("  flags: " + (", ".join(hit) if hit else "none (0)"))
PY
fi
if [[ -n "${ext5v:-}" ]]; then
  python3 - "$ext5v" <<'PY' || true
import sys
v=float(sys.argv[1])
if v < 4.70:
    print(f"  [FAIL] 5V rail {v:.3f}V — Pi 5 wants ~5.1V")
    sys.exit(2)
if v < 4.95:
    print(f"  [WARN] 5V rail {v:.3f}V — marginal (want ~5.1V)")
    sys.exit(1)
print(f"  [OK]   5V rail {v:.3f}V now")
PY
  rc=$?
  [[ $rc -eq 1 ]] && WARN=1
  [[ $rc -eq 2 ]] && FAIL=1
fi
uv_now=$(dmesg 2>/dev/null | grep -c 'Undervoltage detected' || true)
info "Undervoltage lines this boot (dmesg): $uv_now"
if [[ "$uv_now" -gt 20 ]]; then
  fail "frequent undervoltage ($uv_now). Even short dips stall the SD controller"
elif [[ "$uv_now" -gt 0 ]]; then
  warn "undervoltage seen $uv_now times this boot (may be brief dips under load)"
fi

echo
echo "-- kernel I/O this boot --"
stuck=$(dmesg 2>/dev/null | grep -c 'Card stuck being busy' || true)
ioerr=$(dmesg 2>/dev/null | grep -c 'I/O error, dev mmcblk' || true)
jbd=$(dmesg 2>/dev/null | grep -c 'jbd2/mmcblk' || true)
ext4hang=$(dmesg 2>/dev/null | grep -c 'blocked for more than' || true)
info "mmc Card stuck: $stuck"
info "mmc I/O errors: $ioerr"
info "jbd2/ext4 hung tasks: $jbd / $ext4hang"
if [[ "$stuck" -gt 0 ]]; then
  fail "SD controller reports card stuck busy ($stuck). Classic dying/fake/undervolted SD"
fi
if [[ "$ioerr" -gt 0 ]]; then
  fail "block I/O errors on mmcblk ($ioerr). Data at risk — clone this card"
fi
if [[ "$ext4hang" -gt 0 ]]; then
  fail "filesystem thread blocked >120s — storage not completing I/O"
fi
if [[ "$stuck" -eq 0 && "$ioerr" -eq 0 ]]; then
  ok "no mmc I/O errors in current dmesg"
fi

echo
echo "-- I/O latency (kernel counters since boot) --"
if [[ -n "$disk" && -r /sys/block/${disk#/dev/}/stat ]]; then
  python3 - "${disk#/dev/}" <<'PY'
import sys
name=sys.argv[1]
raw=open(f"/sys/block/{name}/stat").read().split()
# 0 reads, 1 rd_merged, 2 rd_sectors, 3 rd_ms, 4 writes, 5 wr_merged, 6 wr_sectors, 7 wr_ms
reads, rd_ms = int(raw[0]), int(raw[3])
writes, wr_ms = int(raw[4]), int(raw[7])
rd_avg = (rd_ms / reads) if reads else 0.0
wr_avg = (wr_ms / writes) if writes else 0.0
print(f"  reads={reads}  avg={rd_avg:.1f} ms/IO")
print(f"  writes={writes}  avg={wr_avg:.1f} ms/IO")
# healthy UHS-I SD: typically <15ms read, <40ms write
if reads and rd_avg > 80:
    print(f"  [FAIL] average read {rd_avg:.0f}ms (healthy SD is usually <15ms)")
    sys.exit(2)
if writes and wr_avg > 200:
    print(f"  [FAIL] average write {wr_avg:.0f}ms (healthy SD is usually <40ms)")
    sys.exit(2)
if reads and rd_avg > 20:
    print(f"  [WARN] average read {rd_avg:.0f}ms is slow")
    sys.exit(1)
if writes and wr_avg > 50:
    print(f"  [WARN] average write {wr_avg:.0f}ms is slow")
    sys.exit(1)
print("  [OK]   average I/O latency looks healthy")
PY
  rc=$?
  [[ $rc -eq 1 ]] && WARN=1
  [[ $rc -eq 2 ]] && FAIL=1
else
  warn "could not read block stats"
fi

echo
echo "-- filesystem --"
if sudo_cmd dumpe2fs -h "${root_src}" >/tmp/sd_selfcheck_ext4.txt 2>/dev/null; then
  state=$(sed -n 's/^Filesystem state:[[:space:]]*//p' /tmp/sd_selfcheck_ext4.txt)
  last=$(sed -n 's/^Last checked:[[:space:]]*//p' /tmp/sd_selfcheck_ext4.txt)
  writes=$(sed -n 's/^Lifetime writes:[[:space:]]*//p' /tmp/sd_selfcheck_ext4.txt)
  info "ext4 state=$state  last_fsck=$last  lifetime_writes=$writes"
  if [[ "$state" != "clean" ]]; then
    fail "ext4 state is '$state' (not clean)"
  else
    ok "ext4 is clean"
  fi
  rm -f /tmp/sd_selfcheck_ext4.txt
else
  warn "dumpe2fs failed (need sudo?)"
fi
used=$(df -P / | awk 'NR==2{print $5}' | tr -d '%')
if [[ "${used:-0}" -ge 95 ]]; then
  fail "root filesystem ${used}% full"
elif [[ "${used:-0}" -ge 85 ]]; then
  warn "root filesystem ${used}% full"
fi

echo
echo "-- boot time --"
if command -v systemd-analyze >/dev/null; then
  systemd-analyze 2>/dev/null | sed 's/^/  /'
  chain=$(systemd-analyze critical-chain --no-pager 2>/dev/null | sed -n '1,8p')
  printf '%s\n' "$chain" | sed 's/^/  /'
fi

if [[ "$BENCH" -eq 1 && -n "$disk" ]]; then
  echo
  echo "-- sequential read bench (32 MiB, iflag=direct, timeout 45s) --"
  info "healthy UHS-I SD on Pi 5: typically 40–90 MiB/s; <15 MiB/s is bad"
  out=$(sudo_cmd timeout -k 5 45 dd if="$disk" of=/dev/null bs=1M count=32 iflag=direct 2>&1)
  rc=$?
  printf '%s\n' "$out" | tail -3 | sed 's/^/  /'
  if [[ $rc -eq 124 || $rc -eq 137 ]]; then
    fail "read bench timed out — card hung (same as boot-time 'Card stuck')"
  elif [[ $rc -ne 0 ]]; then
    fail "read bench failed rc=$rc"
  else
    python3 - "$out" <<'PY' || true
import sys, re
text=sys.argv[1]
# 33554432 bytes (32 MB, 31 MiB) copied, 1.234 s, 27.2 MB/s
m=re.search(r',\s*([0-9.]+)\s*([MG])B/s', text)
if not m:
    print("  [WARN] could not parse dd speed")
    sys.exit(1)
v=float(m.group(1))
if m.group(2)=="G":
    v*=1000
if v < 15:
    print(f"  [FAIL] sequential read {v:.1f} MB/s")
    sys.exit(2)
if v < 35:
    print(f"  [WARN] sequential read {v:.1f} MB/s is below a healthy UHS-I card")
    sys.exit(1)
print(f"  [OK]   sequential read {v:.1f} MB/s")
PY
    rc=$?
    [[ $rc -eq 1 ]] && WARN=1
    [[ $rc -eq 2 ]] && FAIL=1
  fi
else
  echo
  info "skip sequential bench (pass --bench to run a 32 MiB direct read)"
fi

if [[ "$DO_WRITE" -eq 1 ]]; then
  echo
  echo "-- write+fsync bench (8 MiB on /) --"
  f=/var/tmp/sd_selfcheck_write.bin
  out=$(timeout -k 5 30 dd if=/dev/zero of="$f" bs=1M count=8 conv=fsync oflag=direct 2>&1)
  rc=$?
  rm -f "$f"
  printf '%s\n' "$out" | tail -3 | sed 's/^/  /'
  if [[ $rc -eq 124 || $rc -eq 137 ]]; then
    fail "write bench timed out — card hung on fsync"
  elif [[ $rc -ne 0 ]]; then
    fail "write bench failed rc=$rc"
  fi
fi

echo
echo "=== verdict ==="
if [[ "$FAIL" -eq 1 ]]; then
  echo "FAIL — storage is not healthy. Clone the card NOW, then replace it."
  echo "Pi 5 is happiest on NVMe (HAT) or a known-good A2/V30 SD (SanDisk High Endurance / Samsung PRO Endurance)."
elif [[ "$WARN" -eq 1 ]]; then
  echo "WARN — not dead, but not healthy. Expect slow/unreliable boots until the card (and/or 5V supply) is replaced."
else
  echo "OK — no storage red flags this boot."
fi
if [[ ${#notes[@]} -gt 0 ]]; then
  echo
  echo "highlights:"
  for n in "${notes[@]}"; do
    echo "  - $n"
  done
fi

exit $(( FAIL ? 2 : WARN ? 1 : 0 ))
