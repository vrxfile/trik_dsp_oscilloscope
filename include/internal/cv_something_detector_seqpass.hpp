/*
 * cv_something_detector_seqpass.hpp
 *
 *  Created on: Apr 28, 2015
 *      Author: Rostislav Varzar
 */

#ifndef CV_SOMETHING_DETECTOR_SEQPASS_HPP_
#define CV_SOMETHING_DETECTOR_SEQPASS_HPP_

#ifndef __cplusplus
#error C++-only header
#endif

#include <cassert>
#include <cmath>
#include <c6x.h>
#include <stdio.h>

#include "internal/stdcpp.hpp"
#include "internal/cv_font_1.hpp"
#include "trik_vidtranscode_cv.h"

using namespace std;

/* **** **** **** **** **** */namespace trik /* **** **** **** **** **** */
{

/* **** **** **** **** **** */namespace cv /* **** **** **** **** **** */
{

#warning Eliminate global var
static uint64_t s_rgb888hsv[640 * 480];
static uint32_t s_wi2wo[640];
static uint32_t s_hi2ho[480];

template<>
class SomethingDetector<TRIK_VIDTRANSCODE_CV_VIDEO_FORMAT_YUV422,
		TRIK_VIDTRANSCODE_CV_VIDEO_FORMAT_RGB565X> : public CVAlgorithm
{
private:
	static const int m_detectZoneScale = 6;

	TrikCvImageDesc m_inImageDesc;
	TrikCvImageDesc m_outImageDesc;

	// Colors
	#define CL_RED		0xFF0000
	#define CL_GREEN	0x00FF00
	#define CL_BLUE		0x0000FF
	#define CL_BLACK	0x000000
	#define CL_WHITE	0xFFFFFF
	#define CL_GRAY		0x7F7F7F
	#define CL_YELLOW	0xFFFF00

	// Font params
	#define FONT_SPACE	10
	#define FONT_X		16
	#define FONT_Y		8

	static uint16_t* restrict s_mult43_div; // allocated from fast ram
	static uint16_t* restrict s_mult255_div; // allocated from fast ram

	static void __attribute__((always_inline)) writeOutputPixel(uint16_t* restrict _rgb565ptr,
			const uint32_t _rgb888)
	{
		*_rgb565ptr = ((_rgb888>>19)&0x001f) | ((_rgb888>>5)&0x07e0) | ((_rgb888<<8)&0xf800);
	}

	// Put pixel
	void __attribute__((always_inline)) drawOutputPixelBound(
			const int32_t _srcCol, const int32_t _srcRow,
			const int32_t _srcColBot, const int32_t _srcColTop,
			const int32_t _srcRowBot, const int32_t _srcRowTop,
			const TrikCvImageBuffer& _outImage, const uint32_t _rgb888) const
	{
		const int32_t srcCol = range<int32_t> (_srcColBot, _srcCol, _srcColTop);
		const int32_t srcRow = range<int32_t> (_srcRowBot, _srcRow, _srcRowTop);

		const int32_t dstRow = s_hi2ho[srcRow];
		const int32_t dstCol = s_wi2wo[srcCol];

		const uint32_t dstOfs = dstRow * m_outImageDesc.m_lineLength + dstCol
				* sizeof(uint16_t);
		writeOutputPixel(
				reinterpret_cast<uint16_t*> (_outImage.m_ptr + dstOfs), _rgb888);
	}

	// Draw dot
	void __attribute__((always_inline)) _draw_point(const int32_t x,
			const int32_t y, const TrikCvImageBuffer& _outImage,
			const uint32_t fgcolor)
	{
		const int32_t widthBot = 0;
		const int32_t widthTop = m_inImageDesc.m_width - 1;
		const int32_t heightBot = 0;
		const int32_t heightTop = m_inImageDesc.m_height - 1;
		drawOutputPixelBound(x, y, widthBot, widthTop, heightBot, heightTop,
				_outImage, fgcolor);
	}

	// Fill rectangle region
	void __attribute__((always_inline)) drawFillRectangle(const int32_t x,
			const int32_t y, const uint32_t width, const uint32_t height,
			const TrikCvImageBuffer& _outImage, const uint32_t fgcolor)
	{
		int32_t xi, yi;
		for (yi = y; yi < y + height; yi++)
		{
			for (xi = x; xi < x + width; xi++)
			{
				_draw_point(xi, yi, _outImage, fgcolor);
			}
		}
	}

	// Draw line
	void __attribute__((always_inline)) drawLine(int32_t x0,
			int32_t y0, const int32_t x1, const int32_t y1,
			const TrikCvImageBuffer& _outImage, const uint32_t fgcolor)
	{
		int32_t x = x1 - x0;
		int32_t y = y1 - y0;
		int32_t dx = abs(x), sx = x0 < x1 ? 1 : -1;
		int32_t dy = -abs(y), sy = y0 < y1 ? 1 : -1;
		int32_t err = dx + dy, e2;
		for (;;)
		{
			_draw_point(x0, y0, _outImage, fgcolor);
			e2 = 2 * err;
			if (e2 >= dy)
			{
				if (x0 == x1)
					break;
				err += dy;
				x0 += sx;
			}
			if (e2 <= dx)
			{
				if (y0 == y1)
					break;
				err += dx;
				y0 += sy;
			}
		}
	}

	// Draw char
	void __attribute__((always_inline)) drawChar(uint8_t ascii,
			const int32_t poX, const int32_t poY, const uint32_t size,
			const TrikCvImageBuffer& _outImage, const uint32_t fgcolor)
	{
		uint8_t temp = 0, j, k;
		if ((ascii >= 32) && (ascii <= 255))
		{
			;
		}
		else
		{
			ascii = '?' - 32;
		}
		for (int i = 0; i < FONT_X; i++)
		//for (int i = FONT_X - 1 ; i >= 0; i -- )
		{
			j = FONT_X - (i + 1);
			if ((ascii >= 0x20) && (ascii <= 0x7F))
			{
				temp = Font16x16[ascii - 0x20][j];
			}
			else if (ascii >= 0xC0)
			{
				temp = Font16x16[ascii - 0x65][j];
			}
			k = j / 8;
			for (uint8_t f = 0; f < FONT_Y; f++)
			{
				if ((temp >> f) & 0x01)
				{
					if (size == 0)
						_draw_point(poX + j - (k * 8), poY + f + (k * 8),
								_outImage, fgcolor);
					else
						drawFillRectangle(poX + j * size - (k * 8) * size,
								poY + f * size + (k * 8) * size, size, size,
								_outImage, fgcolor);
				}
			}
		}
	}

	// Draw string
	void __attribute__((always_inline)) drawString(const char *in_str,
			int32_t poX, int32_t poY, const uint32_t size,
			const TrikCvImageBuffer& _outImage, const uint32_t fgcolor)
	{
		while (*in_str)
		{
			if ((poX + FONT_SPACE) > m_inImageDesc.m_width)
			{
				poX = 1;
				poY = poY + FONT_X * size;
			}
			drawChar(*in_str, poX, poY, size, _outImage, fgcolor);
			if (size > 0)
				poX += FONT_SPACE * size;
			else
				poX += FONT_SPACE;
			*in_str++;
		}
	}


public:
	virtual bool setup(const TrikCvImageDesc& _inImageDesc,
			const TrikCvImageDesc& _outImageDesc, int8_t* _fastRam,
			size_t _fastRamSize)
	{
		m_inImageDesc = _inImageDesc;
		m_outImageDesc = _outImageDesc;

		if (m_inImageDesc.m_width < 0 || m_inImageDesc.m_height < 0
				|| m_inImageDesc.m_width % 32 != 0 || m_inImageDesc.m_height
				% 4 != 0)
			return false;

#define min(x,y) x < y ? x : y;
		const double
				srcToDstShift =
						min(static_cast<double>(m_outImageDesc.m_width)/m_inImageDesc.m_width,
								static_cast<double>(m_outImageDesc.m_height)/m_inImageDesc.m_height);

		const uint32_t widthIn = _inImageDesc.m_width;
		const uint32_t widthOut = _outImageDesc.m_width;
		uint32_t* restrict p_wi2wo = s_wi2wo;
		for (int i = 0; i < widthIn; i++)
		{
			*(p_wi2wo++) = i * srcToDstShift;
		}

		const uint32_t heightIn = _inImageDesc.m_height;
		const uint32_t heightOut = _outImageDesc.m_height;
		uint32_t* restrict p_hi2ho = s_hi2ho;
		for (uint32_t i = 0; i < heightIn; i++)
		{
			*(p_hi2ho++) = i * srcToDstShift;
		}

		/* Static member initialization on first instance creation */
		if (s_mult43_div == NULL || s_mult255_div == NULL)
		{
			if (_fastRamSize < (1u << 8) * sizeof(*s_mult43_div) + (1u << 8)
					* sizeof(*s_mult255_div))
				return false;

			s_mult43_div = reinterpret_cast<typeof(s_mult43_div)> (_fastRam);
			_fastRam += (1u << 8) * sizeof(*s_mult43_div);
			s_mult255_div = reinterpret_cast<typeof(s_mult255_div)> (_fastRam);
			_fastRam += (1u << 8) * sizeof(*s_mult255_div);

			s_mult43_div[0] = 0;
			s_mult255_div[0] = 0;
			for (uint32_t idx = 1; idx < (1u << 8); ++idx)
			{
				s_mult43_div[idx] = (43u * (1u << 8)) / idx;
				s_mult255_div[idx] = (255u * (1u << 8)) / idx;
			}
		}

		return true;
	}

	virtual bool run(const TrikCvImageBuffer& _inImage,
			TrikCvImageBuffer& _outImage, const TrikCvAlgInArgs& _inArgs,
			TrikCvAlgOutArgs& _outArgs)
	{
		const int8_t* restrict srcImageRow = _inImage.m_ptr;		// Pointer to inImage
		char s1[128];												// Temp string
		int32_t x1, y1, oldx1, oldy1;								// Parameters to draw osc
		int32_t x2, y2, oldx2, oldy2;								// Parameters to draw osc
		int32_t idata1, idata2, idata3, idata4;						// Data read from inImage
		const nsamples = 512;										// Number of samples to read
		if (m_inImageDesc.m_height * m_inImageDesc.m_lineLength
				> _inImage.m_size)
			return false;
		if (m_outImageDesc.m_height * m_outImageDesc.m_lineLength
				> _outImage.m_size)
			return false;
		_outImage.m_size = m_outImageDesc.m_height
				* m_outImageDesc.m_lineLength;

		// Draw some things
		//sprintf(s1, "b0 = %d, b5 = %d", *(srcImageRow + 0), *(srcImageRow + 5));
		sprintf(s1, "");
		drawString(s1, 0, 0, 1, _outImage, CL_YELLOW);
		x1 = x2 = y1 = y2 = oldx1 = oldx2 = oldy1 = oldy2 = 0;
		for (int32_t i = 0; i < nsamples; i ++)
		{
			x1 = i * 319 / nsamples;
			idata1 = *(srcImageRow + 0);
			idata2 = *(srcImageRow + 1);
			idata3 = *(srcImageRow + 2);
			idata4 = *(srcImageRow + 3);
			y1 = ((idata1 << 8) + idata2) * 120 / 65535 + 119 - 32;
			y2 = ((idata3 << 8) + idata4) * 120 / 65535 + 119 + 32;
			drawLine(oldx1, oldy1, x1, y1, _outImage, CL_YELLOW);
			drawLine(oldx1, oldy2, x1, y2, _outImage, CL_WHITE);
			oldx1 = x1;
			oldy1 = y1;
			oldy2 = y2;
			srcImageRow+=4;
		}

		return true;
	}
};

uint16_t* restrict SomethingDetector<TRIK_VIDTRANSCODE_CV_VIDEO_FORMAT_YUV422,
TRIK_VIDTRANSCODE_CV_VIDEO_FORMAT_RGB565X>::s_mult43_div = NULL;
uint16_t* restrict SomethingDetector<TRIK_VIDTRANSCODE_CV_VIDEO_FORMAT_YUV422,
TRIK_VIDTRANSCODE_CV_VIDEO_FORMAT_RGB565X>::s_mult255_div = NULL;

} /* **** **** **** **** **** * namespace cv * **** **** **** **** **** */

} /* **** **** **** **** **** * namespace trik * **** **** **** **** **** */

#endif /* CV_SOMETHING_DETECTOR_SEQPASS_HPP_ */
