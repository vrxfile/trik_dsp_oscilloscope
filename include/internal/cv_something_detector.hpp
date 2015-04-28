/*
 * cv_something_detector.hpp
 *
 *  Created on: Apr 28, 2015
 *      Author: Rostislav Varzar
 */

#ifndef CV_SOMETHING_DETECTOR_HPP_
#define CV_SOMETHING_DETECTOR_HPP_

#ifndef __cplusplus
#error C++-only header
#endif

#include <cassert>
#include <vector>
#include <c6x.h>

#include "internal/stdcpp.hpp"
#include "trik_vidtranscode_cv.h"


/* **** **** **** **** **** */ namespace trik /* **** **** **** **** **** */ {

/* **** **** **** **** **** */ namespace cv /* **** **** **** **** **** */ {


template <TrikCvImageFormat _inFormat, TrikCvImageFormat _outFormat>
class SomethingDetector : public CVAlgorithm,
                     private assert_inst<false> // Generic instance, non-functional
{
  public:
    virtual bool setup(const TrikCvImageDesc& _inImageDesc,
                       const TrikCvImageDesc& _outImageDesc,
                       int8_t* _fastRam, size_t _fastRamSize) { return false; }
    virtual bool run(const TrikCvImageBuffer& _inImage, TrikCvImageBuffer& _outImage,
                     const TrikCvAlgInArgs& _inArgs, TrikCvAlgOutArgs& _outArgs);

  private:
};

#if 0
#define DEBUG_INLINE __attribute__((noinline))
#else
#define DEBUG_INLINE __attribute__((always_inline))
#endif
//#define DEBUG_REPEAT 20


} /* **** **** **** **** **** * namespace cv * **** **** **** **** **** */

} /* **** **** **** **** **** * namespace trik * **** **** **** **** **** */

// Include one of implementations
//#include "internal/cv_ball_detector_reference.hpp"
//#include "internal/cv_ball_detector_singlepass.hpp"
//#include "internal/cv_ball_detector_seqpass.hpp"
#include "internal/cv_something_detector_seqpass.hpp"



#endif /* CV_SOMETHING_DETECTOR_HPP_ */
