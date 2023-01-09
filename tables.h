/*
 * tables.h
 *
 *  Created on: Jan 9, 2023
 *      Author: John
 */

#ifndef INC_TABLES_H_
#define INC_TABLES_H_

// Look-up table of pixel grayscale level
static const uint32_t GS_LUT[] = {
		0x00000000, // 0 (white)
		0x01000000, // 1
		0x00010000, // 2
		0x01010000, // 3
		0x00000100, // 4
		0x01000100, // 5
		0x00010100, // 6
		0x01010100, // 7
		0x00000001, // 8
		0x01000001, // 9
		0x00010001, // 10
		0x01010001, // 11
		0x00000101, // 12
		0x01000101, // 13
		0x00010101, // 14
		0x01010101  // 15 (black)
};

// Look-up table of masks to clear pixel
static const uint32_t POC_LUT[] = {
		0xFEFEFEFE, // 0
		0xFDFDFDFD, // 1
		0xFBFBFBFB, // 2
		0xF7F7F7F7, // 3
		0xEFEFEFEF, // 4
		0xDFDFDFDF, // 5
		0xBFBFBFBF, // 6
		0x7F7F7F7F  // 7
};

// Look-up table of mask for partial page filing
static const uint32_t LUT_PPM[] = {
		0x00000000, // 0
		0x80808080, // 1
		0xC0C0C0C0, // 2
		0xE0E0E0E0, // 3
		0xF0F0F0F0, // 4
		0xF8F8F8F8, // 5
		0xFCFCFCFC, // 6
		0xFEFEFEFE  // 7
};

// Look-up table of full page by grayscale level
static const uint32_t LUT_SBC[] = {
		0x00000000, // 0 (white)
		0xFF000000, // 1
		0x00FF0000, // 2
		0xFFFF0000, // 3
		0x0000FF00, // 4
		0xFF00FF00, // 5
		0x00FFFF00, // 6
		0xFFFFFF00, // 7
		0x000000FF, // 8
		0xFF0000FF, // 9
		0x00FF00FF, // 10
		0xFFFF00FF, // 11
		0x0000FFFF, // 12
		0xFF00FFFF, // 13
		0x00FFFFFF, // 14
		0xFFFFFFFF  // 15 (black)
};

// Grayscale palette (4 bytes for each level of gray, 4 * 14 bytes total)
static const uint8_t GrayPalette[] = {
		0x06,0x06,0x06,0x06, // level 1
		0x0b,0x0b,0x0b,0x0b, // level 2
		0x10,0x10,0x10,0x10, // level 3
		0x15,0x15,0x15,0x15, // level 4
		0x1a,0x1a,0x1a,0x1a, // level 5
		0x1e,0x1e,0x1e,0x1e, // level 6
		0x23,0x23,0x23,0x23, // level 7
		0x27,0x27,0x27,0x27, // level 8
		0x2b,0x2b,0x2b,0x2b, // level 9
		0x2f,0x2f,0x2f,0x2f, // level 10
		0x32,0x32,0x32,0x32, // level 11
		0x35,0x35,0x35,0x35, // level 12
		0x38,0x38,0x38,0x38, // level 13
		0x3a,0x3a,0x3a,0x3a  // level 14
};



#endif /* INC_TABLES_H_ */
