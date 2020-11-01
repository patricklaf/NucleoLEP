// LEP
// © 2019 Patrick Lafarguette
//
// LEP file structure
//
// See http://dcmoto.free.fr/bricolage/sdlep-reader/index.html

#ifndef LEP_H
#define LEP_H

#include <stdint.h>

#define LEP_DEFAULT 0

// Header
typedef struct lep_header {
	uint32_t magic;
	uint32_t version;
	// Computer
	uint8_t brand;
	uint8_t family;
	uint32_t compatibily;
	// Loading
	uint8_t period; // µs
	uint8_t method;
	uint32_t duration; // ms
	// Block
	uint8_t blocks;
} __attribute__((packed)) lep_header;

// Block
typedef struct lep_block {
	uint32_t size; // Up to 4 GB
	uint32_t duration;
} __attribute__((packed)) lep_block;

// Magic
#define LEP_MAGIC 0x4650454C // LEPF

// Version
#define LEP_VERSION_MAJOR 1
#define LEP_VERSION_MINOR 0
#define LEP_VERSION_REVISION 0

#define LEP_VERSION ((LEP_VERSION_MAJOR << 16) | (LEP_VERSION_MINOR << 8) | LEP_VERSION_REVISION)

// Period
#define LEP_PERIOD 50

// Method
#define LEP_RUN 0 // RUN "
#define LEP_LOADM 1 // LOADM
#define LEP_LOADMR 2 // LOADM"",,R

// Brand
#define LEP_ACORN 1
#define LEP_AMSTRAD 2
#define LEP_APOLLO_7 3
#define LEP_APPLE 4
#define LEP_APPLIED_TECHNOLOGIES 5
#define LEP_ATARI 6
#define LEP_CAMPUTERS 7
#define LEP_CANON 8
#define LEP_COMMODORE 9
#define LEP_DRAGON 10
#define LEP_ENTERPRISE 11
#define LEP_EPSON 12
#define LEP_EXELVISION 13
#define LEP_EXIDY 14
#define LEP_GRUNDY 15
#define LEP_HANIMEX 16
#define LEP_INDATA 17
#define LEP_JUPITER_CANTAB 18
#define LEP_KYOCERA 19
#define LEP_MATRA 20
#define LEP_MATSUSHITA 21
#define LEP_MATTEL_ELECTRONICS 22
#define LEP_MEMOTECH 23
#define LEP_MICRONIQUE 24
#define LEP_MSX 41
#define LEP_NEC 25
#define LEP_OLIVETTI 26
#define LEP_ORIC 27
#define LEP_PHILIPS 28
#define LEP_PROTEUS_INTERNATIONAL 29
#define LEP_SANYO 30
#define LEP_SEGA 31
#define LEP_SHARP 32
#define LEP_SINCLAIR 33
#define LEP_SMT 34
#define LEP_SORD 35
#define LEP_SPECTRAVIDEO 36
#define LEP_TANDY 37
#define LEP_TEXAS_INSTRUMENTS 38
#define LEP_THOMSON 39
#define LEP_VIDEO_TECHNOLOGY 40

// Family
// 0 LEP_DEFAULT
// 1..127 brand
// 128..255 multi brands families
#define LEP_MSX1 128
#define LEP_MSX2 129
#define LEP_MSX2PLUS 130

// Amstrad
#define LEP_CPC464 (1 << 0)
#define LEP_CPC664 (1 << 1)
#define LEP_CPC6128 (1 << 2)
#define LEP_464PLUS (1 << 3)
#define LEP_6128PLUS (1 << 4)

// Thomson
#define LEP_MO 1
#define LEP_TO 2

// MO
#define LEP_MO5 (1 << 0)
#define LEP_MO6 (1 << 1)

// TO
#define LEP_TO7 (1 << 0)
#define LEP_TO7_16K (1 << 1)
#define LEP_TO770 (1 << 2)
#define LEP_TO770_64K (1 << 3)
#define LEP_TO8 (1 << 4)
#define LEP_TO8_256K (1 << 5)
#define LEP_TO8D (1 << 6)
#define LEP_TO8D_256K (1 << 7)
#define LEP_TO9 (1 << 8)
#define LEP_TO9_64K (1 << 9)
#define LEP_TO9PLUS (1 << 10)

#endif /* LEP_H */
