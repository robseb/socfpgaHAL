/*
 * soc_regacc.h
 *
 *  Created on: 27.08.2020
 *      Author: robin
 */

#ifndef SOCFPGAHAL_INC_SOC_REGACC_H_
#define SOCFPGAHAL_INC_SOC_REGACC_H_


#ifndef __ASSEMBLY__
#ifdef __cplusplus
#include <cstddef>
#include <cstdbool>
#include <cstdint>
#else   /* __cplusplus */
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#endif  /* __cplusplus */
#endif  /* __ASSEMBLY__ */

#ifdef __cplusplus
extern "C"
{
#endif  /* __cplusplus */

/*!
 * \addtogroup SOC_SOCAL_UTIL SoCAL Utilities
 *
 * This file contains utility and support functions for the Altera SoCAL.
 * @{
 */

#ifdef __ASSEMBLY__
#define SOC_CAST(type, ptr)  ptr
#else   /* __ASSEMBLY__ */
/*! Cast the pointer to specified pointer type.
 *
 *  Note: This macro expands to \e ptr value only for assembler language
 *        targets.
 *
 *  \param type     The pointer type to cast to
 *  \param ptr      The pointer to apply the type cast to
 */
#define SOC_CAST(type, ptr)  ((type) (ptr))
#endif  /* __ASSEMBLY__ */

/*!
 * \addtogroup SOC_SOCAL_UTIL_RW_FUNC SoCAL Memory Read/Write Utilities
 *
 * This section implements read and write functionality for various
 * memory untis. The memory unit terms used for these functions are
 * consistent with those used in the ARM Architecture Reference Manual
 * ARMv7-A and ARMv7-R edition manual. The terms used for units of memory are:
 *
 *  Unit of Memory | Abbreviation | Size in Bits
 * :---------------|:-------------|:------------:
 *  Byte           | byte         |       8
 *  Half Word      | hword        |      16
 *  Word           | word         |      32
 *  Double Word    | dword        |      64
 *
 * @{
 */

/*! Write the 8 bit byte to the destination address in device memory.
 *  \param dest - Write destination pointer address
 *  \param src  - 8 bit data byte to write to memory
 */
#define soc_write_byte(dest, src)       (*SOC_CAST(volatile uint8_t *, (dest)) = (src))

/*! Read and return the 8 bit byte from the source address in device memory.
 *  \param src    Read source pointer address
 *  \returns      8 bit data byte value
 */
#define soc_read_byte(src)              (*SOC_CAST(volatile uint8_t *, (src)))

/*! Write the 16 bit half word to the destination address in device memory.
 *  \param dest - Write destination pointer address
 *  \param src  - 16 bit data half word to write to memory
 */
#define soc_write_hword(dest, src)      (*SOC_CAST(volatile uint16_t *, (dest)) = (src))

/*! Read and return the 16 bit half word from the source address in device memory.
 *  \param src    Read source pointer address
 *  \returns      16 bit data half word value
 */
#define soc_read_hword(src)             (*SOC_CAST(volatile uint16_t *, (src)))

/*! Write the 32 bit word to the destination address in device memory.
 *  \param dest - Write destination pointer address
 *  \param src  - 32 bit data word to write to memory
 */
#define soc_write_word(dest, src)       (*SOC_CAST(volatile uint32_t *, (dest)) = (src))

/*! Read and return the 32 bit word from the source address in device memory.
 *  \param src    Read source pointer address
 *  \returns      32 bit data word value
 */
#define soc_read_word(src)              (*SOC_CAST(volatile uint32_t *, (src)))

/*! Write the 64 bit double word to the destination address in device memory.
 *  \param dest - Write destination pointer address
 *  \param src  - 64 bit data double word to write to memory
 */
#define soc_write_dword(dest, src)      (*SOC_CAST(volatile uint64_t *, (dest)) = (src))

/*! Read and return the 64 bit double word from the source address in device memory.
 *  \param src    Read source pointer address
 *  \returns      64 bit data double word value
 */
#define soc_read_dword(src)             (*SOC_CAST(volatile uint64_t *, (src)))

/*! @} */

/*!
 * \addtogroup SOC_SOCAL_UTIL_SC_FUNC SoCAL Memory Bit Set/Clr/XOR/Replace Utilities
 *
 * This section implements useful macros to set, clear, change, and replace
 * selected bits within a word in memory or a memory-mapped register.
 * @{
 *
 */

/*! Set selected bits in the 8 bit byte at the destination address in device memory.
 *  \param dest - Destination pointer address
 *  \param bits - Bits to set in destination byte
 */
#define     soc_setbits_byte(dest, bits)        (soc_write_byte(dest, soc_read_byte(dest) | (bits)))

/*! Clear selected bits in the 8 bit byte at the destination address in device memory.
 *  \param dest - Destination pointer address
 *  \param bits - Bits to clear in destination byte
 */
#define     soc_clrbits_byte(dest, bits)        (soc_write_byte(dest, soc_read_byte(dest) & ~(bits)))

/*! Change or toggle selected bits in the 8 bit byte at the destination address in device memory.
 *  \param dest - Destination pointer address
 *  \param bits - Bits to change in destination byte
 */
#define     soc_xorbits_byte(dest, bits)        (soc_write_byte(dest, soc_read_byte(dest) ^ (bits)))

/*! Replace selected bits in the 8 bit byte at the destination address in device memory.
 *  \param  dest - Destination pointer address
 *  \param  msk  - Bits to replace in destination byte
 *  \param  src  - Source bits to write to cleared bits in destination byte
 */
#define     soc_replbits_byte(dest, msk, src)   (soc_write_byte(dest,(soc_read_byte(dest) & ~(msk)) | ((src) & (msk))))

/*! Set selected bits in the 16 bit halfword at the destination address in device memory.
 *  \param dest - Destination pointer address
 *  \param bits - Bits to set in destination halfword
 */
#define     soc_setbits_hword(dest, bits)       (soc_write_hword(dest, soc_read_hword(dest) | (bits)))

/*! Clear selected bits in the 16 bit halfword at the destination address in device memory.
 *  \param dest - Destination pointer address
 *  \param bits - Bits to clear in destination halfword
 */
#define     soc_clrbits_hword(dest, bits)       (soc_write_hword(dest, soc_read_hword(dest) & ~(bits)))

/*! Change or toggle selected bits in the 16 bit halfword at the destination address in device memory.
 *  \param dest - Destination pointer address
 *  \param bits - Bits to change in destination halfword
 */
#define     soc_xorbits_hword(dest, bits)       (soc_write_hword(dest, soc_read_hword(dest) ^ (bits)))

/*! Replace selected bits in the 16 bit halfword at the destination address in device memory.
 *  \param  dest - Destination pointer address
 *  \param  msk  - Bits to replace in destination halfword
 *  \param  src  - Source bits to write to cleared bits in destination halfword
 */
#define     soc_replbits_hword(dest, msk, src)   (soc_write_hword(dest,(soc_read_hword(dest) & ~(msk)) | ((src) & (msk))))

/*! Set selected bits in the 32 bit word at the destination address in device memory.
 *  \param dest - Destination pointer address
 *  \param bits - Bits to set in destination word
 */
#define     soc_setbits_word(dest, bits)        (soc_write_word(dest, soc_read_word(dest) | (bits)))

/*! Clear selected bits in the 32 bit word at the destination address in device memory.
 *  \param dest - Destination pointer address
 *  \param bits - Bits to clear in destination word
 */
#define     soc_clrbits_word(dest, bits)        (soc_write_word(dest, soc_read_word(dest) & ~(bits)))

/*! Change or toggle selected bits in the 32 bit word at the destination address in device memory.
 *  \param dest - Destination pointer address
 *  \param bits - Bits to change in destination word
 */
#define     soc_xorbits_word(dest, bits)        (soc_write_word(dest, soc_read_word(dest) ^ (bits)))

/*! Replace selected bits in the 32 bit word at the destination address in device memory.
 *  \param  dest - Destination pointer address
 *  \param  msk  - Bits to replace in destination word
 *  \param  src  - Source bits to write to cleared bits in destination word
 */
#define     soc_replbits_word(dest, msk, src)   (soc_write_word(dest,(soc_read_word(dest) & ~(msk)) | ((src) & (msk))))

/*! Set selected bits in the 64 bit doubleword at the destination address in device memory.
 *  \param dest - Destination pointer address
 *  \param bits - Bits to set in destination doubleword
 */
#define     soc_setbits_dword(dest, bits)       (soc_write_dword(dest, soc_read_dword(dest) | (bits)))

/*! Clear selected bits in the 64 bit doubleword at the destination address in device memory.
 *  \param dest - Destination pointer address
 *  \param bits - Bits to clear in destination doubleword
 */
#define     soc_clrbits_dword(dest, bits)       (soc_write_dword(dest, soc_read_dword(dest) & ~(bits)))

/*! Change or toggle selected bits in the 64 bit doubleword at the destination address in device memory.
 *  \param dest - Destination pointer address
 *  \param bits - Bits to change in destination doubleword
 */
#define     soc_xorbits_dword(dest, bits)       (soc_write_dword(dest, soc_read_dword(dest) ^ (bits)))

/*! Replace selected bits in the 64 bit doubleword at the destination address in device memory.
 *  \param  dest - Destination pointer address
 *  \param  msk  - Bits to replace in destination doubleword
 *  \param  src  - Source bits to write to cleared bits in destination word
 */
#define     soc_replbits_dword(dest, msk, src)   (soc_write_dword(dest,(soc_read_dword(dest) & ~(msk)) | ((src) & (msk))))

/*! @} */

/*! @} */

#ifdef __cplusplus
}

#endif /* SOCFPGAHAL_INC_SOC_REGACC_H_ */
