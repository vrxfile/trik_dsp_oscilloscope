#ifndef TRIK_VIDTRANSCODE_CV_INTERNAL_CV_BALL_DETECTOR_REFERENCE_HPP_
#define TRIK_VIDTRANSCODE_CV_INTERNAL_CV_BALL_DETECTOR_REFERENCE_HPP_

#ifndef __cplusplus
#error C++-only header
#endif

#include <cassert>
#include <cmath>
#include <vector>

#include "internal/stdcpp.hpp"
#include "trik_vidtranscode_cv.h"


/* **** **** **** **** **** */ namespace trik /* **** **** **** **** **** */ {

/* **** **** **** **** **** */ namespace cv /* **** **** **** **** **** */ {


template <>
class BallDetector<TRIK_VIDTRANSCODE_CV_VIDEO_FORMAT_YUV422, TRIK_VIDTRANSCODE_CV_VIDEO_FORMAT_RGB565X> : public CVAlgorithm
{
  private:
    TrikCvImageDesc m_inImageDesc;
    TrikCvImageDesc m_outImageDesc;

    XDAS_UInt8 m_detectHueFrom;
    XDAS_UInt8 m_detectHueTo;
    XDAS_UInt8 m_detectSatFrom;
    XDAS_UInt8 m_detectSatTo;
    XDAS_UInt8 m_detectValFrom;
    XDAS_UInt8 m_detectValTo;
    XDAS_Int32  m_targetX;
    XDAS_Int32  m_targetY;
    XDAS_UInt32 m_targetPoints;

    std::vector<TrikCvImageDimension> m_srcToDstColConv;
    std::vector<TrikCvImageDimension> m_srcToDstRowConv;
    std::vector<XDAS_UInt16> m_mult255_div;
    std::vector<XDAS_UInt16> m_mult43_div;

    bool testifyRgbPixel(const XDAS_UInt8 _r,
                         const XDAS_UInt8 _g,
                         const XDAS_UInt8 _b)
    {
      XDAS_UInt8 max;
      XDAS_UInt8 med;
      XDAS_UInt8 min;
      XDAS_UInt16 hsv_base;
      bool        hsv_is_incr;

#define DEF_MIN_MED_MAX(_max, _med, _min, _hsv_base, _hsv_is_incr) \
      do { \
        max = _max; \
        med = _med; \
        min = _min; \
        hsv_base    = _hsv_base; \
        hsv_is_incr = _hsv_is_incr; \
      } while (0)

      if (_r > _b)
      {
        if (_b > _g) // r>b, b>g --> r>b>g
          DEF_MIN_MED_MAX(_r, _b, _g, 255, false);
        else if (_r > _g) // r>b, b<=g, r>g --> r>g>=b
          DEF_MIN_MED_MAX(_r, _g, _b, 0, true);
        else // r>b, b<=g, r<=g --> g>=r>b
          DEF_MIN_MED_MAX(_g, _r, _b, 85, false);
      }
      else if (_g > _b) // r<=b, g>b --> g>b>=r
        DEF_MIN_MED_MAX(_g, _b, _r, 85, true);
      else if (_r > _g) // r<=b, g<=b, r>g --> b>=r>g
        DEF_MIN_MED_MAX(_b, _r, _g, 171, true);
      else // r<=b, g<=b, r<=g --> b>=g>=r
        DEF_MIN_MED_MAX(_b, _g, _r, 171, false);
#undef DEF_MIN_MED_MAX

      const XDAS_UInt16 hsv_v = max;
      const bool hsv_v_det = (m_detectValFrom <= hsv_v) && (m_detectValTo >= hsv_v);

      /* optimized by table based multiplication with power-2 divisor, simulate 255*(max-min)/max */
      const XDAS_UInt16 hsv_s = (static_cast<XDAS_UInt16>(m_mult255_div[max]) * static_cast<XDAS_UInt16>(max-min)) >> 8;
      const bool hsv_s_det = (m_detectSatFrom <= hsv_s) && (m_detectSatTo >= hsv_s);

      /* optimized by table based multiplication with power-2 divisor, simulate 43*(med-min)/(max-min) */
      const XDAS_UInt16 hsv_incr = (static_cast<XDAS_UInt16>(m_mult43_div[max-min]) * static_cast<XDAS_UInt16>(med-min)) >> 8;
      const XDAS_UInt16 hsv_h = hsv_is_incr ? hsv_base + hsv_incr : hsv_base - hsv_incr;
      const bool hsv_h_det = (m_detectHueFrom <= m_detectHueTo)
                           ? (m_detectHueFrom <= hsv_h) && (m_detectHueTo >= hsv_h)
                           : (m_detectHueFrom <= hsv_h) || (m_detectHueTo >= hsv_h);

      return hsv_h_det && hsv_v_det && hsv_s_det;
    }


    void writeRgbPixel(XDAS_UInt16* _rgb, const XDAS_UInt8 _r, const XDAS_UInt8 _g, const XDAS_UInt8 _b)
    {
      *_rgb = (static_cast<XDAS_UInt16>(_r)     >>3)
            | (static_cast<XDAS_UInt16>(_g&0xfc)<<3)
            | (static_cast<XDAS_UInt16>(_b&0xf8)<<8);
    }

    void writeRgbPixel(TrikCvImageDimension _srcCol, XDAS_UInt16* _rgbRow,
                       const XDAS_UInt8 _r, const XDAS_UInt8 _g, const XDAS_UInt8 _b)
    {
      assert(_srcCol < m_srcToDstColConv.size());
      const TrikCvImageDimension dstCol = m_srcToDstColConv[_srcCol];
      writeRgbPixel(&_rgbRow[dstCol], _r, _g, _b);
    }

    void drawRgbPixel(TrikCvImageDimension _srcCol, TrikCvImageDimension _srcRow,
                      const TrikCvImageBuffer& _outImage,
                      const XDAS_UInt8 _r, const XDAS_UInt8 _g, const XDAS_UInt8 _b)
    {
      assert(_srcRow < m_srcToDstRowConv.size());
      const TrikCvImageDimension dstRow = m_srcToDstRowConv[_srcRow];

      assert(_srcCol < m_srcToDstColConv.size());
      const TrikCvImageDimension dstCol = m_srcToDstColConv[_srcCol];

      const TrikCvImageSize dstOfs = dstRow*m_outImageDesc.m_lineLength + dstCol*sizeof(XDAS_UInt16);
      XDAS_UInt16* rgbPtr = reinterpret_cast<XDAS_UInt16*>(_outImage.m_ptr + dstOfs);
      writeRgbPixel(rgbPtr, _r, _g, _b);
    }

    void drawRgbTargetCross(TrikCvImageDimension _srcCol, TrikCvImageDimension _srcRow,
                            const TrikCvImageBuffer& _outImage,
                            const XDAS_UInt8 _r, const XDAS_UInt8 _g, const XDAS_UInt8 _b)
    {
      for (int adj = 10; adj < 20; ++adj)
      {
        drawRgbPixel(range<TrikCvImageDimension>(0, _srcCol-adj, m_inImageDesc.m_width-1),
                     range<TrikCvImageDimension>(0, _srcRow-1  , m_inImageDesc.m_height-1),
                     _outImage, _r, _g, _b);
        drawRgbPixel(range<TrikCvImageDimension>(0, _srcCol-adj, m_inImageDesc.m_width-1),
                     range<TrikCvImageDimension>(0, _srcRow    , m_inImageDesc.m_height-1),
                     _outImage, _r, _g, _b);
        drawRgbPixel(range<TrikCvImageDimension>(0, _srcCol-adj, m_inImageDesc.m_width-1),
                     range<TrikCvImageDimension>(0, _srcRow+1  , m_inImageDesc.m_height-1),
                     _outImage, _r, _g, _b);
        drawRgbPixel(range<TrikCvImageDimension>(0, _srcCol+adj, m_inImageDesc.m_width-1),
                     range<TrikCvImageDimension>(0, _srcRow-1  , m_inImageDesc.m_height-1),
                     _outImage, _r, _g, _b);
        drawRgbPixel(range<TrikCvImageDimension>(0, _srcCol+adj, m_inImageDesc.m_width-1),
                     range<TrikCvImageDimension>(0, _srcRow    , m_inImageDesc.m_height-1),
                     _outImage, _r, _g, _b);
        drawRgbPixel(range<TrikCvImageDimension>(0, _srcCol+adj, m_inImageDesc.m_width-1),
                     range<TrikCvImageDimension>(0, _srcRow+1  , m_inImageDesc.m_height-1),
                     _outImage, _r, _g, _b);
        drawRgbPixel(range<TrikCvImageDimension>(0, _srcCol-1  , m_inImageDesc.m_width-1),
                     range<TrikCvImageDimension>(0, _srcRow-adj, m_inImageDesc.m_height-1),
                     _outImage, _r, _g, _b);
        drawRgbPixel(range<TrikCvImageDimension>(0, _srcCol    , m_inImageDesc.m_width-1),
                     range<TrikCvImageDimension>(0, _srcRow-adj, m_inImageDesc.m_height-1),
                     _outImage, _r, _g, _b);
        drawRgbPixel(range<TrikCvImageDimension>(0, _srcCol+1  , m_inImageDesc.m_width-1),
                     range<TrikCvImageDimension>(0, _srcRow-adj, m_inImageDesc.m_height-1),
                     _outImage, _r, _g, _b);
        drawRgbPixel(range<TrikCvImageDimension>(0, _srcCol-1  , m_inImageDesc.m_width-1),
                     range<TrikCvImageDimension>(0, _srcRow+adj, m_inImageDesc.m_height-1),
                     _outImage, _r, _g, _b);
        drawRgbPixel(range<TrikCvImageDimension>(0, _srcCol    , m_inImageDesc.m_width-1),
                     range<TrikCvImageDimension>(0, _srcRow+adj, m_inImageDesc.m_height-1),
                     _outImage, _r, _g, _b);
        drawRgbPixel(range<TrikCvImageDimension>(0, _srcCol+1  , m_inImageDesc.m_width-1),
                     range<TrikCvImageDimension>(0, _srcRow+adj, m_inImageDesc.m_height-1),
                     _outImage, _r, _g, _b);
      }
    }

    void proceedRgbPixel(const TrikCvImageDimension _srcCol, const TrikCvImageDimension _srcRow,
                         XDAS_UInt16* _rgbRow,
                         const XDAS_UInt8 _r, const XDAS_UInt8 _g, const XDAS_UInt8 _b)
    {
      if (testifyRgbPixel(_r, _g, _b))
      {
        m_targetX += _srcCol;
        m_targetY += _srcRow;
        ++m_targetPoints;
        writeRgbPixel(_srcCol, _rgbRow, 0xff, 0xff, 0x00);
      }
      else
        writeRgbPixel(_srcCol, _rgbRow, _r, _g, _b);
    }

    void proceedTwoYuyvPixels(const XDAS_UInt32 _yuyv,
                              const TrikCvImageDimension _srcCol,
                              const TrikCvImageDimension _srcRow,
                              XDAS_UInt16* _rgbRow)
    {
      typedef XDAS_Int16 Int16;
      union YUYV
      {
        struct
        {
          XDAS_UInt8 m_y1;
          XDAS_UInt8 m_u;
          XDAS_UInt8 m_y2;
          XDAS_UInt8 m_v;
        } m_unpacked;
        XDAS_UInt32 m_packed;
      } m_yuyv;
      m_yuyv.m_packed = _yuyv;

      const Int16 y1 = static_cast<Int16>(m_yuyv.m_unpacked.m_y1) - static_cast<Int16>(16);
      const Int16 d  = static_cast<Int16>(m_yuyv.m_unpacked.m_u)  - static_cast<Int16>(128);
      const Int16 y2 = static_cast<Int16>(m_yuyv.m_unpacked.m_y2) - static_cast<Int16>(16);
      const Int16 e  = static_cast<Int16>(m_yuyv.m_unpacked.m_v)  - static_cast<Int16>(128);

      const Int16 r12 =
                      + static_cast<Int16>(409/4) * e
                      + static_cast<Int16>(128/4);
      const Int16 g12 =
                      - static_cast<Int16>(100/4) * d
                      - static_cast<Int16>(208/4) * e
                      + static_cast<Int16>(128/4);
      const Int16 b12 =
                      + static_cast<Int16>(516/4) * d
                      + static_cast<Int16>(128/4);

      const Int16 y1c = static_cast<Int16>(298/4) * y1;
      const Int16 y2c = static_cast<Int16>(298/4) * y2;

      proceedRgbPixel(_srcCol+0, _srcRow, _rgbRow,
                      range<XDAS_Int16>(0, (r12 + y1c) >> 6, 255),
                      range<XDAS_Int16>(0, (g12 + y1c) >> 6, 255),
                      range<XDAS_Int16>(0, (b12 + y1c) >> 6, 255));

      proceedRgbPixel(_srcCol+1, _srcRow, _rgbRow,
                      range<XDAS_Int16>(0, (r12 + y2c) >> 6, 255),
                      range<XDAS_Int16>(0, (g12 + y2c) >> 6, 255),
                      range<XDAS_Int16>(0, (b12 + y2c) >> 6, 255));
    }

  public:
    virtual bool setup(const TrikCvImageDesc& _inImageDesc, const TrikCvImageDesc& _outImageDesc, int8_t* _fastRam, size_t _fastRamSize)
    {
      m_inImageDesc  = _inImageDesc;
      m_outImageDesc = _outImageDesc;

      if (   m_inImageDesc.m_width < 0
          || m_inImageDesc.m_height < 0
          || m_inImageDesc.m_width  % 32 != 0
          || m_inImageDesc.m_height % 4  != 0)
        return false;

      m_srcToDstColConv.resize(m_inImageDesc.m_width);
      for (TrikCvImageDimension srcCol=0; srcCol < m_srcToDstColConv.size(); ++srcCol)
        m_srcToDstColConv[srcCol] = (srcCol*m_outImageDesc.m_width) / m_inImageDesc.m_width;

      m_srcToDstRowConv.resize(m_inImageDesc.m_height);
      for (TrikCvImageDimension srcRow=0; srcRow < m_srcToDstRowConv.size(); ++srcRow)
        m_srcToDstRowConv[srcRow] = (srcRow*m_outImageDesc.m_height) / m_inImageDesc.m_height;

      m_mult255_div.resize(0x100);
      m_mult255_div[0] = 0;
      for (XDAS_UInt16 idx = 1; idx < m_mult255_div.size(); ++idx)
        m_mult255_div[idx] = (static_cast<XDAS_UInt16>(255) * static_cast<XDAS_UInt16>(1u<<8)) / idx;

      m_mult43_div.resize(0x100);
      m_mult43_div[0] = 0;
      for (XDAS_UInt16 idx = 1; idx < m_mult43_div.size(); ++idx)
        m_mult43_div[idx] = (static_cast<XDAS_UInt16>(43) * static_cast<XDAS_UInt16>(1u<<8)) / idx;

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

      m_detectHueFrom = range<XDAS_Int16>(0, (_inArgs.detectHueFrom * 255) / 359, 255); // scaling 0..359 to 0..255
      m_detectHueTo   = range<XDAS_Int16>(0, (_inArgs.detectHueTo   * 255) / 359, 255); // scaling 0..359 to 0..255
      m_detectSatFrom = range<XDAS_Int16>(0, (_inArgs.detectSatFrom * 255) / 100, 255); // scaling 0..100 to 0..255
      m_detectSatTo   = range<XDAS_Int16>(0, (_inArgs.detectSatTo   * 255) / 100, 255); // scaling 0..100 to 0..255
      m_detectValFrom = range<XDAS_Int16>(0, (_inArgs.detectValFrom * 255) / 100, 255); // scaling 0..100 to 0..255
      m_detectValTo   = range<XDAS_Int16>(0, (_inArgs.detectValTo   * 255) / 100, 255); // scaling 0..100 to 0..255

#ifdef DEBUG_REPEAT
      for (unsigned repeat = 0; repeat < DEBUG_REPEAT; ++repeat) {
#endif

      m_targetX = 0;
      m_targetY = 0;
      m_targetPoints = 0;

      for (TrikCvImageDimension srcRow=0; srcRow < m_inImageDesc.m_height; ++srcRow)
      {
        assert(srcRow < m_srcToDstRowConv.size());
        const TrikCvImageDimension dstRow = m_srcToDstRowConv[srcRow];

        const TrikCvImageSize srcRowOfs = srcRow*m_inImageDesc.m_lineLength;
        const XDAS_UInt32* srcImage = reinterpret_cast<XDAS_UInt32*>(_inImage.m_ptr + srcRowOfs);

        const TrikCvImageSize dstRowOfs = dstRow*m_outImageDesc.m_lineLength;
        XDAS_UInt16* dstImageRow = reinterpret_cast<XDAS_UInt16*>(_outImage.m_ptr + dstRowOfs);

        assert(m_inImageDesc.m_width % 32 == 0); // verified in setup
        for (TrikCvImageDimension srcCol=0; srcCol < m_inImageDesc.m_width; srcCol+=2)
          proceedTwoYuyvPixels(*srcImage++, srcCol, srcRow, dstImageRow);
      }


#ifdef DEBUG_REPEAT
      } // repeat
#endif

      const XDAS_UInt32 inImagePixels = m_inImageDesc.m_width * m_inImageDesc.m_height;
      if (inImagePixels > 0)
      {
        const XDAS_UInt32 targetRadius = std::ceil(std::sqrt(static_cast<float>(m_targetPoints) / 3.1415927f));
        _outArgs.targetSize = static_cast<XDAS_UInt32>(targetRadius*100*4)
                            / inImagePixels; // scaling to 0..100
      }
      else
        _outArgs.targetSize = 0;

      if (m_targetPoints > 0)
      {
        XDAS_Int32 targetX = m_targetX/m_targetPoints;
        XDAS_Int32 targetY = m_targetY/m_targetPoints;
        drawRgbTargetCross(targetX, targetY, _outImage, 0xff, 0x00, 0xff);
        _outArgs.targetX = ((targetX - static_cast<XDAS_Int32>(m_inImageDesc.m_width) /2) * 100*2) / static_cast<XDAS_Int32>(m_inImageDesc.m_width);
        _outArgs.targetY = ((targetY - static_cast<XDAS_Int32>(m_inImageDesc.m_height)/2) * 100*2) / static_cast<XDAS_Int32>(m_inImageDesc.m_height);
      }
      else
      {
        _outArgs.targetX = 0;
        _outArgs.targetY = 0;
      }

      return true;
    }
};


} /* **** **** **** **** **** * namespace cv * **** **** **** **** **** */

} /* **** **** **** **** **** * namespace trik * **** **** **** **** **** */


#endif // !TRIK_VIDTRANSCODE_CV_INTERNAL_CV_BALL_DETECTOR_REFERENCE_HPP_
