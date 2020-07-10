/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2017, ITU/ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

/** \file     TEncTop.h
    \brief    encoder class (header)
*/

#ifndef __TENCTOP__
#define __TENCTOP__

// Include files
#include "TLibCommon/TComList.h"
#include "TLibCommon/TComPrediction.h"
#include "TLibCommon/TComTrQuant.h"
#include "TLibCommon/TComLoopFilter.h"
#include "TLibCommon/AccessUnit.h"

#include "TLibVideoIO/TVideoIOYuv.h"

#include "TEncCfg.h"
#include "TEncGOP.h"
#include "TEncSlice.h"
#include "TEncEntropy.h"
#include "TEncCavlc.h"
#include "TEncSbac.h"
#include "TEncSearch.h"
#include "TEncSampleAdaptiveOffset.h"
#include "TEncPreanalyzer.h"
#include "TEncRateCtrl.h"
//! \ingroup TLibEncoder
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// encoder class

// 尝试，读取视频显著性图，是论文第二章的内容
class saliency
{
public:
    static float* weightCurrFrame;
    static float weightSumCurrFrame;
    static int frameWidthInCtus;
    static int frameHeightInCtus;
    static float*** videoWeight;
    static double ratioInFrame; // 当前CTU的权重占帧全部CTU的权重和的比例
    static void initial(int frameNum, int heightNum, int widthNum);
    static void destroy(int frameNum, int heightNum, int widthNum);
    static float*** readVideoWeight(char* filename, int heightNum, int widthNum);
};
//Mao
class maoTimer
{
public:
    static double targetComp; //目标复杂度(0-1)
    static int adaptDepth; // 每个CTU分配的深度
    static double** timeInGOP3; 
    static int** numberInGOP;
    static double** weightInGOP3;
    static bool** isModeOn;
    static double weightSum; // 权重和
    static double compCTURef; // 不进行复杂度控制时，平均每个CTU的编码复杂度(CLOCK)
    static double weightExtraCTU;
    static double timeAllMode;
    static double timeM20CTU; // 第一个GOP处理完以后，需要保存平均每个CTU选择深度20、2Nx2N的时间，作为分目将分配到CTU的时间（clock域）转换到weight域
    static bool isClassLow; // 是否为class low
    // static int POCOfCurrGOPStart; //当前GOP的第一帧的POC
    static double compLeftLastFrame; //上一帧剩余的复杂度
    static double compPerFramePre; // 预分配的每帧复杂度
    static double compCurrFrameAct; // 实际上这一帧的目标复杂度 CCFA = CLLF + CPFP（就是这个等于上面两个加起来）
    static double compCurrFrameLeft; // 该帧内，剩余的复杂度，每帧的初始值为compCurrFrameAct，然后减去该帧内花费的时间
    static double compGOPTarget; //
    static double compGOPTargetAdj; //根据公式9调整得到的GOP实际的目标复杂度
    static double ksi1; //希腊字母ξ，公式(8)
    static double ksi2;
    static long start; // 每个模式的开始时间
    static long CTUStart; // 每个CTU的开始时间
    static long totalTimeCurrCTU; // 当前已编码CTU的时间
    static long frameStart; // 每帧的开始时间
    static long seqStart; // 序列开始的时间
    static long totalTimeCurrFrame;
    static long totalTimeOfGOP3; // 第3个GOP的总时间
    static long* totalTimeEveryGOP;
    static long GOPStart; // GOP的开始时间
    static double compCTUTarget;
    static int* numberList; //读取前一个GOP的Mij的数量，转换为一维数组
    static int* sortingList; //用于保存降序排序的原下标
    static void initial(int x, int y);
    static void destroy(int x, int y);
    static int* sortIndex(int s[], int pos[], int left, int right);
    static bool* goToNextDepth; // 用于确认下一深度是否所有模式都跳过，不进行这个判断的话程序会崩溃
    static bool maoBoundary; // CTU是否处在最后一行
    static double* frameTimeInGOP3; // 第三个GOP每帧的时间
    static double* ratioInGOP3; // 该帧编码时间在GOP3中的比值
    static double timeFixer; // 每四个GOP的最后一帧不加以限制，用于修正序列编码时间的预测值
    static double timeSeqPred; // 序列的预测时间，POC=8编码完成后可获取，并可结合上一个变量进行自适应调整
    static double timeFirst9Frames;
    // static void modeCounter(TComDataCU *pCtu, UInt CUZAddr);
    
};
class TEncTop : public TEncCfg
{
private:
  // picture
  Int                     m_iPOCLast;                     ///< time index (POC)
  Int                     m_iNumPicRcvd;                  ///< number of received pictures
  UInt                    m_uiNumAllPicCoded;             ///< number of coded pictures
  TComList<TComPic*>      m_cListPic;                     ///< dynamic list of pictures

  // encoder search
  TEncSearch              m_cSearch;                      ///< encoder search class
  //TEncEntropy*            m_pcEntropyCoder;                     ///< entropy encoder
  TEncCavlc*              m_pcCavlcCoder;                       ///< CAVLC encoder
  // coding tool
  TComTrQuant             m_cTrQuant;                     ///< transform & quantization class
  TComLoopFilter          m_cLoopFilter;                  ///< deblocking filter class
  TEncSampleAdaptiveOffset m_cEncSAO;                     ///< sample adaptive offset class
  TEncEntropy             m_cEntropyCoder;                ///< entropy encoder
  TEncCavlc               m_cCavlcCoder;                  ///< CAVLC encoder
  TEncSbac                m_cSbacCoder;                   ///< SBAC encoder
  TEncBinCABAC            m_cBinCoderCABAC;               ///< bin coder CABAC

  // processing unit
  TEncGOP                 m_cGOPEncoder;                  ///< GOP encoder
  TEncSlice               m_cSliceEncoder;                ///< slice encoder
  TEncCu                  m_cCuEncoder;                   ///< CU encoder
  // SPS
  ParameterSetMap<TComSPS> m_spsMap;                      ///< SPS. This is the base value. This is copied to TComPicSym
  ParameterSetMap<TComPPS> m_ppsMap;                      ///< PPS. This is the base value. This is copied to TComPicSym
  // RD cost computation
  TComRdCost              m_cRdCost;                      ///< RD cost computation class
  TEncSbac***             m_pppcRDSbacCoder;              ///< temporal storage for RD computation
  TEncSbac                m_cRDGoOnSbacCoder;             ///< going on SBAC model for RD stage
#if FAST_BIT_EST
  TEncBinCABACCounter***  m_pppcBinCoderCABAC;            ///< temporal CABAC state storage for RD computation
  TEncBinCABACCounter     m_cRDGoOnBinCoderCABAC;         ///< going on bin coder CABAC for RD stage
#else
  TEncBinCABAC***         m_pppcBinCoderCABAC;            ///< temporal CABAC state storage for RD computation
  TEncBinCABAC            m_cRDGoOnBinCoderCABAC;         ///< going on bin coder CABAC for RD stage
#endif

  // quality control
  TEncPreanalyzer         m_cPreanalyzer;                 ///< image characteristics analyzer for TM5-step3-like adaptive QP

  TEncRateCtrl            m_cRateCtrl;                    ///< Rate control class

protected:
  Void  xGetNewPicBuffer  ( TComPic*& rpcPic, Int ppsId ); ///< get picture buffer which will be processed. If ppsId<0, then the ppsMap will be queried for the first match.
  Void  xInitVPS          (TComVPS &vps, const TComSPS &sps); ///< initialize VPS from encoder options
  Void  xInitSPS          (TComSPS &sps);                 ///< initialize SPS from encoder options
  Void  xInitPPS          (TComPPS &pps, const TComSPS &sps); ///< initialize PPS from encoder options
  Void  xInitScalingLists (TComSPS &sps, TComPPS &pps);   ///< initialize scaling lists
  Void  xInitHrdParameters(TComSPS &sps);                 ///< initialize HRD parameters

  Void  xInitPPSforTiles  (TComPPS &pps);
  Void  xInitRPS          (TComSPS &sps, Bool isFieldCoding);           ///< initialize PPS from encoder options

public:
  TEncTop();
  virtual ~TEncTop();

  Void      create          ();
  Void      destroy         ();
  Void      init            (Bool isFieldCoding);
  Void      deletePicBuffer ();

  // -------------------------------------------------------------------------------------------------------------------
  // member access functions
  // -------------------------------------------------------------------------------------------------------------------

  TComList<TComPic*>*     getListPic            () { return  &m_cListPic;             }
  TEncSearch*             getPredSearch         () { return  &m_cSearch;              }

  TComTrQuant*            getTrQuant            () { return  &m_cTrQuant;             }
  TComLoopFilter*         getLoopFilter         () { return  &m_cLoopFilter;          }
  TEncSampleAdaptiveOffset* getSAO              () { return  &m_cEncSAO;              }
  TEncGOP*                getGOPEncoder         () { return  &m_cGOPEncoder;          }
  TEncSlice*              getSliceEncoder       () { return  &m_cSliceEncoder;        }
  TEncCu*                 getCuEncoder          () { return  &m_cCuEncoder;           }
  TEncEntropy*            getEntropyCoder       () { return  &m_cEntropyCoder;        }
  TEncCavlc*              getCavlcCoder         () { return  &m_cCavlcCoder;          }
  TEncSbac*               getSbacCoder          () { return  &m_cSbacCoder;           }
  TEncBinCABAC*           getBinCABAC           () { return  &m_cBinCoderCABAC;       }

  TComRdCost*             getRdCost             () { return  &m_cRdCost;              }
  TEncSbac***             getRDSbacCoder        () { return  m_pppcRDSbacCoder;       }
  TEncSbac*               getRDGoOnSbacCoder    () { return  &m_cRDGoOnSbacCoder;     }
  TEncRateCtrl*           getRateCtrl           () { return &m_cRateCtrl;             }
  Void selectReferencePictureSet(TComSlice* slice, Int POCCurr, Int GOPid );
  Int getReferencePictureSetIdxForSOP(Int POCCurr, Int GOPid );

#if JCTVC_Y0038_PARAMS
  Void                   setParamSetChanged(Int spsId, Int ppsId);
#endif
  Bool                   PPSNeedsWriting(Int ppsId);
  Bool                   SPSNeedsWriting(Int spsId);

  // -------------------------------------------------------------------------------------------------------------------
  // encoder function
  // -------------------------------------------------------------------------------------------------------------------

  /// encode several number of pictures until end-of-sequence
  Void encode( Bool bEos,
               TComPicYuv* pcPicYuvOrg,
               TComPicYuv* pcPicYuvTrueOrg, const InputColourSpaceConversion snrCSC, // used for SNR calculations. Picture in original colour space.
               TComList<TComPicYuv*>& rcListPicYuvRecOut,
               std::list<AccessUnit>& accessUnitsOut, Int& iNumEncoded );

  /// encode several number of pictures until end-of-sequence
  Void encode( Bool bEos, TComPicYuv* pcPicYuvOrg,
               TComPicYuv* pcPicYuvTrueOrg, const InputColourSpaceConversion snrCSC, // used for SNR calculations. Picture in original colour space.
               TComList<TComPicYuv*>& rcListPicYuvRecOut,
               std::list<AccessUnit>& accessUnitsOut, Int& iNumEncoded, Bool isTff);

#if JVET_F0064_MSSSIM
  Void printSummary(Bool isField) { m_cGOPEncoder.printOutSummary (m_uiNumAllPicCoded, isField, m_printMSEBasedSequencePSNR, m_printSequenceMSE, m_printMSSSIM, m_spsMap.getFirstPS()->getBitDepths()); }
#else
  Void printSummary(Bool isField) { m_cGOPEncoder.printOutSummary (m_uiNumAllPicCoded, isField, m_printMSEBasedSequencePSNR, m_printSequenceMSE, m_spsMap.getFirstPS()->getBitDepths()); }
#endif

};

//! \}

#endif // __TENCTOP__

