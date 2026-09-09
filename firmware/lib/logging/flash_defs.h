#pragma once

// SPI NAND command opcodes and GET/SET FEATURES register addresses.
//
// TODO - the exact NAND part is not locked in yet (candidate: GD5F1GQ5RExxG, DS-00889 Rev1.4).
// The opcodes below are the conventional SPI NAND command set shared by most vendors
// (GigaDevice/Winbond/Micron/ISSI all agree on these), but must be re-checked against
// the final datasheet (see DS p.32 for program/erase, p.42 & p.46 for status registers)
// before this is trusted on real hardware.

#define CMD_NAND_RESET 0xFFu
#define CMD_NAND_READ_ID 0x9Fu
#define CMD_NAND_GET_FEATURES 0x0Fu
#define CMD_NAND_SET_FEATURES 0x1Fu
#define CMD_NAND_WRITE_ENABLE 0x06u
#define CMD_NAND_WRITE_DISABLE 0x04u

#define CMD_NAND_PAGE_READ 0x13u // page read: array -> cache
#define CMD_NAND_READ_FROM_CACHE 0x03u
#define CMD_NAND_READ_FROM_CACHE_QUAD 0x6Bu // quad output, cache -> host

#define CMD_NAND_PROGRAM_LOAD 0x02u        // load cache, resetting the rest of the cache to 0xFF first
#define CMD_NAND_PROGRAM_LOAD_QUAD 0x32u   // same, quad input
#define CMD_NAND_PROGRAM_LOAD_RANDOM 0x84u // load cache without resetting the rest of the cache
#define CMD_NAND_PROGRAM_EXECUTE 0x10u     // cache -> array

#define CMD_NAND_BLOCK_ERASE 0xD8u

// GET/SET FEATURES register addresses.
// status_a (C0h) and status_b (F0h) are confirmed - see Jacob's spec.
#define FEATURE_ADDR_BLOCK_LOCK 0xA0u
#define FEATURE_ADDR_CONFIG 0xB0u // TODO - confirm QE (quad enable) bit position against final datasheet
#define FEATURE_ADDR_STATUS_A 0xC0u
#define FEATURE_ADDR_STATUS_B 0xF0u

#define CONFIG_QE_BIT (1u << 0) // TODO - confirm bit position against final datasheet

// NAND array geometry.
// TODO - confirm against final datasheet. Values below assume a 1Gb x1 part with a
// 2048B (+64B spare) page, 64 pages/block, 1024 blocks - matches the GD5F1GQ5RExxG candidate.
#define NAND_PAGES_PER_BLOCK 64u
#define NAND_NUM_BLOCKS 1024u
#define NAND_NUM_PAGES (NAND_PAGES_PER_BLOCK * NAND_NUM_BLOCKS)
