# Flash Explanation

## Flash

Low Level API for reading and writing to NAND Flash.
We are using Indirect functinal mode HAL operations (see pg. 1261 of user manual) for our discrete actions.

### begin()

First, pins from STM32H7 schematic are mapped using Pin Map to their corresponding names. 

Then, initialize Quad SPI, then reset and check (RCC), then disable block protection, enable Quad Mode, and finally ensure that error correction is enabled.

### read()

Read from main memory, first by checking that the cache has enough space, then commanding a load from flash, waiting for page to arrive in cache, and then sending the data from cache to a provided buffer.

See pg. 10 of datasheet to understand memory layout. There is 2k bytes available of the main page and 64 bytes with ECC enabled for spare page reading/writing. 

Note, here a row address is equivalent to specifying a page and a column addresss is equivalent to specifying the specific byte.

There is 64 pages in a block 2176 valid bytes per page.

### write_to_cache()

First we ensure we have enough space to write into the cache. Then, if this is writing at the very first byte, we will using CMD_NAND_PROGRAM_LOAD command, which clears the cache. Otherwise, we use CMD_NAND_PROGRAM_LOAD_RANDOM command, which will not delete the cache.

We then tell the cache how much data will will be sending and then transmit the specified amount of data.

### program()

Ensure there is no going operation and if there is, fail. See pg. 44 of datasheet. An operation is in progress is a PROGRAM_EXECUTE, PAGE_READ, BLOCK_ERASE, or RESET operation is executing and OIP will be 1.

Unlock flash and start program operation.

Intentionally end early without waiting for program operation finish so CPU is freed.

### check_program_status()

Check previous program operation to ensure it successfully finished. Fail early if an operation is still in progress. If Program previously failed, mark the block as bad.

### erase_block()

Erasing should not be done in flight so its okay to wait for any previous operations to finish up. Then enable writing to flash, erase the current block, and then disable flash writing.

## Bad Page Table

### Begin

If the magic key is inplace, then the bad page table has been established
and we just return. Otherwise, we need to identify the initial bad pages 
from the factory.

> A block is bad is one or more pages are bad

We start by ensuring that the user wants to proceed, then deleting the entirity
of flash on the STM32H7. After, we visit every spare section on each block and 
if the first byte is not 0xFF, then the block is bad. We represent all 1024
NAND blocks with a single bit, where a 1 in the bbt means the block is good 
and a 0 represents a bad block. 

After, we set a magic byte at a set location to indicate that the bbt is set.

### Mark Page Bad

The bbt is never written to again after it is initialized. Instead, if a 
page goes bad at runtime, then we will track it via a linear array of 
uint32s, where each is a page address of a bad page.

## flash_defs.h

The number of pages and blocks for the GD5F chip is fixed, but the number of bytes per page varies depending on whether ECC is on or off. There is a fixed 2k bytes for the main page, but the number of available bytes that the user can program in the spare region is 64 when ECC is on and 128 when ECC is off.

The commands are the hex codes that are used to signify to the cache what it should do.


