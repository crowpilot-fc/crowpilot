// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar
//
// Firmware flasher for the configurator. Parses a .uf2 image and writes
// it to an RP2350 in BOOTSEL mode over WebUSB, using the bootrom PICOBOOT
// protocol.
//
// VERIFICATION NOTE. parseUf2() and encodePicobootCmd() are pure and are
// checked in the browser. The WebUSB path (everything from requestDevice
// onward) cannot be exercised without a physical board and must be
// bench-tested before it is trusted. A failed flash is recoverable: an
// RP2350 with no valid image falls back to BOOTSEL, so the firmware can
// always be re-flashed by the manual UF2 drag-and-drop.
//
// Protocol references: the RP2350 datasheet bootrom chapter and the
// pico-sdk picoboot.h / picotool picoboot_connection.c definitions.

'use strict';

// --- UF2 container format ---------------------------------------------------

const UF2_BLOCK_SIZE = 512;
const UF2_MAGIC_START0 = 0x0a324655;
const UF2_MAGIC_START1 = 0x9e5d5157;
const UF2_MAGIC_END = 0x0ab16f30;
const UF2_FLAG_NOT_MAIN_FLASH = 0x00000001;
const UF2_FLAG_FAMILY_ID = 0x00002000;

// RP2350 UF2 family IDs (ARM secure, RISC-V, ARM non-secure) and RP2040.
const UF2_FAMILIES = {
  0xe48bff59: 'RP2350 Arm-S',
  0xe48bff5a: 'RP2350 RISC-V',
  0xe48bff5b: 'RP2350 Arm-NS',
  0xe48bff56: 'RP2040',
};
const UF2_RP2350_FAMILIES = [0xe48bff59, 0xe48bff5a, 0xe48bff5b];

// Parse a .uf2 ArrayBuffer into flash blocks. Throws on a malformed file.
function parseUf2(buffer) {
  if (buffer.byteLength === 0 || buffer.byteLength % UF2_BLOCK_SIZE !== 0) {
    throw new Error('not a UF2 file (size is not a multiple of 512 bytes)');
  }
  const view = new DataView(buffer);
  const blocks = [];
  const families = new Set();
  const nBlocks = buffer.byteLength / UF2_BLOCK_SIZE;
  for (let i = 0; i < nBlocks; i++) {
    const o = i * UF2_BLOCK_SIZE;
    if (view.getUint32(o, true) !== UF2_MAGIC_START0 ||
        view.getUint32(o + 4, true) !== UF2_MAGIC_START1 ||
        view.getUint32(o + 508, true) !== UF2_MAGIC_END) {
      throw new Error('bad UF2 block magic at block ' + i);
    }
    const flags = view.getUint32(o + 8, true);
    const addr = view.getUint32(o + 12, true);
    const payloadSize = view.getUint32(o + 16, true);
    if (flags & UF2_FLAG_FAMILY_ID) {
      families.add(view.getUint32(o + 28, true) >>> 0);
    }
    if (flags & UF2_FLAG_NOT_MAIN_FLASH) {
      continue;
    }
    if (payloadSize > 476) {
      throw new Error('UF2 block ' + i + ' has an invalid payload size');
    }
    blocks.push({
      addr: addr,
      data: new Uint8Array(buffer.slice(o + 32, o + 32 + payloadSize)),
    });
  }
  if (blocks.length === 0) {
    throw new Error('UF2 file has no flash data blocks');
  }
  return { blocks: blocks, families: families };
}

// Collapse parsed blocks into one contiguous, sector-aligned image. The
// gaps between blocks are filled with 0xff (the erased-flash value).
function buildFlashImage(uf2) {
  const SECTOR = 4096;
  let min = Infinity;
  let max = 0;
  for (const b of uf2.blocks) {
    if (b.addr < min) {
      min = b.addr;
    }
    if (b.addr + b.data.length > max) {
      max = b.addr + b.data.length;
    }
  }
  const base = Math.floor(min / SECTOR) * SECTOR;
  const end = Math.ceil(max / SECTOR) * SECTOR;
  const bytes = new Uint8Array(end - base).fill(0xff);
  for (const b of uf2.blocks) {
    bytes.set(b.data, b.addr - base);
  }
  return { base: base, bytes: bytes };
}

// --- PICOBOOT protocol ------------------------------------------------------

const PICOBOOT_VENDOR_ID = 0x2e8a;
const PICOBOOT_MAGIC = 0x431fd10b;

// Command IDs. The high bit marks a device-to-host data phase; none of
// the commands used here set it.
const PC_EXCLUSIVE_ACCESS = 0x01;
const PC_REBOOT = 0x02;
const PC_FLASH_ERASE = 0x03;
const PC_WRITE = 0x05;
const PC_EXIT_XIP = 0x06;

// Vendor control requests on the PICOBOOT interface.
const PICOBOOT_IF_RESET = 0x41;
const PICOBOOT_IF_CMD_STATUS = 0x42;

const FLASH_SECTOR = 4096;
const WRITE_CHUNK = 4096;

let s_token = 1;

// Encode a 32-byte PICOBOOT command. args is a Uint8Array of up to 16
// bytes placed in the command's argument area.
function encodePicobootCmd(cmdId, cmdSize, transferLength, args) {
  const buf = new ArrayBuffer(32);
  const v = new DataView(buf);
  v.setUint32(0, PICOBOOT_MAGIC, true);
  v.setUint32(4, s_token++, true);
  v.setUint8(8, cmdId);
  v.setUint8(9, cmdSize);
  v.setUint16(10, 0, true);
  v.setUint32(12, transferLength >>> 0, true);
  const out = new Uint8Array(buf);
  if (args && args.length > 0) {
    out.set(args.subarray(0, Math.min(16, args.length)), 16);
  }
  return out;
}

// Two little-endian uint32 args, the layout FLASH_ERASE and WRITE use.
function rangeArgs(addr, size) {
  const a = new Uint8Array(8);
  const v = new DataView(a.buffer);
  v.setUint32(0, addr >>> 0, true);
  v.setUint32(4, size >>> 0, true);
  return a;
}

// A thin PICOBOOT client over an opened WebUSB device.
class Picoboot {
  constructor(device, ifaceNumber, epOut, epIn) {
    this.device = device;
    this.iface = ifaceNumber;
    this.epOut = epOut;
    this.epIn = epIn;
  }

  async resetInterface() {
    await this.device.controlTransferOut({
      requestType: 'vendor',
      recipient: 'interface',
      request: PICOBOOT_IF_RESET,
      value: 0,
      index: this.iface,
    });
  }

  async readStatus() {
    const res = await this.device.controlTransferIn({
      requestType: 'vendor',
      recipient: 'interface',
      request: PICOBOOT_IF_CMD_STATUS,
      value: 0,
      index: this.iface,
    }, 16);
    if (!res.data || res.data.byteLength < 10) {
      throw new Error('short PICOBOOT status response');
    }
    return {
      token: res.data.getUint32(0, true),
      status: res.data.getUint32(4, true),
      cmdId: res.data.getUint8(8),
      inProgress: res.data.getUint8(9),
    };
  }

  // Send one command, an optional OUT data phase, the zero-length ack,
  // then verify the command status. Used for every non-reboot command.
  async runCommand(cmdId, cmdSize, args, dataOut) {
    const transferLength = dataOut ? dataOut.length : 0;
    const cmd = encodePicobootCmd(cmdId, cmdSize, transferLength, args);
    await this._out(cmd);
    if (dataOut) {
      await this._out(dataOut);
    }
    // Commands with no IN data phase are acknowledged by a zero-length
    // IN packet before the status can be read.
    await this.device.transferIn(this.epIn, 64);
    const st = await this.readStatus();
    if (st.status !== 0) {
      throw new Error('PICOBOOT command 0x' + cmdId.toString(16) +
                      ' returned status ' + st.status);
    }
  }

  async _out(data) {
    const res = await this.device.transferOut(this.epOut, data);
    if (res.status !== 'ok') {
      throw new Error('USB bulk OUT ' + res.status);
    }
  }

  exclusiveAccess(mode) {
    return this.runCommand(PC_EXCLUSIVE_ACCESS, 1,
                           new Uint8Array([mode]), null);
  }

  exitXip() {
    return this.runCommand(PC_EXIT_XIP, 0, null, null);
  }

  flashErase(addr, size) {
    return this.runCommand(PC_FLASH_ERASE, 8, rangeArgs(addr, size), null);
  }

  flashWrite(addr, data) {
    return this.runCommand(PC_WRITE, 8, rangeArgs(addr, data.length), data);
  }

  // Reboot into the freshly written image. The device resets as it
  // accepts the command, so the transfer error that follows is expected.
  async reboot() {
    const args = new Uint8Array(12);  // dPC = 0, dSP = 0, dDelayMS
    new DataView(args.buffer).setUint32(8, 500, true);
    const cmd = encodePicobootCmd(PC_REBOOT, 12, 0, args);
    try {
      await this._out(cmd);
    } catch (e) {
      /* expected: the device resets mid-transfer */
    }
  }
}

// Locate the PICOBOOT vendor interface (USB class 0xff) and its bulk
// endpoints on an opened device.
function findPicobootInterface(device) {
  if (!device.configuration) {
    return null;
  }
  for (const iface of device.configuration.interfaces) {
    const alt = iface.alternate;
    if (alt.interfaceClass !== 0xff) {
      continue;
    }
    let epOut = null;
    let epIn = null;
    for (const ep of alt.endpoints) {
      if (ep.type !== 'bulk') {
        continue;
      }
      if (ep.direction === 'out') {
        epOut = ep.endpointNumber;
      } else if (ep.direction === 'in') {
        epIn = ep.endpointNumber;
      }
    }
    if (epOut !== null && epIn !== null) {
      return { number: iface.interfaceNumber, epOut: epOut, epIn: epIn };
    }
  }
  return null;
}

// Flash a parsed UF2 to a BOOTSEL-mode board. onProgress(message, frac)
// reports each step; frac is 0..1 during the write phase. Throws on any
// failure; the caller closes the device on the way out.
async function flashUf2(uf2, onProgress) {
  const report = onProgress || (() => {});
  const device = await navigator.usb.requestDevice({
    filters: [{ vendorId: PICOBOOT_VENDOR_ID }],
  });
  await device.open();
  if (device.configuration === null) {
    await device.selectConfiguration(1);
  }
  const iface = findPicobootInterface(device);
  if (!iface) {
    await device.close();
    throw new Error(
        'no PICOBOOT interface found. Is the board in BOOTSEL mode?');
  }
  await device.claimInterface(iface.number);
  const pb = new Picoboot(device, iface.number, iface.epOut, iface.epIn);

  try {
    report('Resetting the bootloader interface', 0);
    await pb.resetInterface();
    report('Claiming exclusive access', 0);
    await pb.exclusiveAccess(1);
    await pb.exitXip();

    const image = buildFlashImage(uf2);
    report('Erasing ' + image.bytes.length + ' bytes of flash', 0);
    await pb.flashErase(image.base, image.bytes.length);

    for (let off = 0; off < image.bytes.length; off += WRITE_CHUNK) {
      const end = Math.min(off + WRITE_CHUNK, image.bytes.length);
      const chunk = image.bytes.subarray(off, end);
      await pb.flashWrite(image.base + off, chunk);
      report('Writing flash', end / image.bytes.length);
    }

    report('Rebooting into the new firmware', 1);
    await pb.reboot();
    report('Done', 1);
  } finally {
    try {
      await device.close();
    } catch (e) {
      /* ignore */
    }
  }
}
