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

#include "internal/stdcpp.hpp"
#include "trik_vidtranscode_cv.h"


/* **** **** **** **** **** */ namespace trik /* **** **** **** **** **** */ {

/* **** **** **** **** **** */ namespace cv /* **** **** **** **** **** */ {

#warning Eliminate global var
static uint64_t s_rgb888hsv[640*480];
static uint32_t s_wi2wo[640];
static uint32_t s_hi2ho[480];

template <>
class SomethingDetector<TRIK_VIDTRANSCODE_CV_VIDEO_FORMAT_YUV422, TRIK_VIDTRANSCODE_CV_VIDEO_FORMAT_RGB565X> : public CVAlgorithm
{
  private:
    static const int m_detectZoneScale = 6;

    uint64_t m_detectRange;
    uint32_t m_detectExpected;

    int32_t  m_targetX;
    int32_t  m_targetY;
    uint32_t m_targetPoints;

    TrikCvImageDesc m_inImageDesc;
    TrikCvImageDesc m_outImageDesc;

    static uint16_t* restrict s_mult43_div;  // allocated from fast ram
    static uint16_t* restrict s_mult255_div; // allocated from fast ram

    static void __attribute__((always_inline)) writeOutputPixel(uint16_t* restrict _rgb565ptr,
                                                                const uint32_t _rgb888)
    {
      *_rgb565ptr = ((_rgb888>>19)&0x001f) | ((_rgb888>>5)&0x07e0) | ((_rgb888<<8)&0xf800);
    }

    void __attribute__((always_inline)) drawOutputPixelBound(const int32_t _srcCol,
                                                             const int32_t _srcRow,
                                                             const int32_t _srcColBot,
                                                             const int32_t _srcColTop,
                                                             const int32_t _srcRowBot,
                                                             const int32_t _srcRowTop,
                                                             const TrikCvImageBuffer& _outImage,
                                                             const uint32_t _rgb888) const
    {
      const int32_t srcCol = range<int32_t>(_srcColBot, _srcCol, _srcColTop);
      const int32_t srcRow = range<int32_t>(_srcRowBot, _srcRow, _srcRowTop);

      const int32_t dstRow = s_hi2ho[srcRow];
      const int32_t dstCol = s_wi2wo[srcCol];

      const uint32_t dstOfs = dstRow*m_outImageDesc.m_lineLength + dstCol*sizeof(uint16_t);
      writeOutputPixel(reinterpret_cast<uint16_t*>(_outImage.m_ptr+dstOfs), _rgb888);
    }

  public:
    virtual bool setup(const TrikCvImageDesc& _inImageDesc,
                       const TrikCvImageDesc& _outImageDesc,
                       int8_t* _fastRam, size_t _fastRamSize)
    {
      m_inImageDesc  = _inImageDesc;
      m_outImageDesc = _outImageDesc;

      if (   m_inImageDesc.m_width < 0
          || m_inImageDesc.m_height < 0
          || m_inImageDesc.m_width  % 32 != 0
          || m_inImageDesc.m_height % 4  != 0)
        return false;

      #define min(x,y) x < y ? x : y;
      const double srcToDstShift = min(static_cast<double>(m_outImageDesc.m_width)/m_inImageDesc.m_width,
                                 static_cast<double>(m_outImageDesc.m_height)/m_inImageDesc.m_height);

      const uint32_t widthIn  = _inImageDesc.m_width;
      const uint32_t widthOut = _outImageDesc.m_width;
      uint32_t* restrict p_wi2wo = s_wi2wo;
      for(int i = 0; i < widthIn; i++) {
          *(p_wi2wo++) = i*srcToDstShift;
      }

      const uint32_t heightIn  = _inImageDesc.m_height;
      const uint32_t heightOut = _outImageDesc.m_height;
      uint32_t* restrict p_hi2ho = s_hi2ho;
      for(uint32_t i = 0; i < heightIn; i++) {
          *(p_hi2ho++) = i*srcToDstShift;
      }

      /* Static member initialization on first instance creation */
      if (s_mult43_div == NULL || s_mult255_div == NULL)
      {
        if (_fastRamSize < (1u<<8)*sizeof(*s_mult43_div) + (1u<<8)*sizeof(*s_mult255_div))
          return false;

        s_mult43_div  = reinterpret_cast<typeof(s_mult43_div)>(_fastRam);
        _fastRam += (1u<<8)*sizeof(*s_mult43_div);
        s_mult255_div = reinterpret_cast<typeof(s_mult255_div)>(_fastRam);
        _fastRam += (1u<<8)*sizeof(*s_mult255_div);

        s_mult43_div[0] = 0;
        s_mult255_div[0] = 0;
        for (uint32_t idx = 1; idx < (1u<<8); ++idx)
        {
          s_mult43_div[ idx] = (43u  * (1u<<8)) / idx;
          s_mult255_div[idx] = (255u * (1u<<8)) / idx;
        }
      }

      return true;
    }

    virtual bool run(const TrikCvImageBuffer& _inImage, TrikCvImageBuffer& _outImage,
                     const TrikCvAlgInArgs& _inArgs, TrikCvAlgOutArgs& _outArgs)
    {
      if (m_inImageDesc.m_height * m_inImageDesc.m_lineLength > _inImage.m_size)
        return false;
      if (m_outImageDesc.m_height * m_outImageDesc.m_lineLength > _outImage.m_size)
        return false;
      _outImage.m_size = m_outImageDesc.m_height * m_outImageDesc.m_lineLength;

      // Draw some things


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
