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

// ���ԣ���ȡ��Ƶ������ͼ�������ĵڶ��µ�����
class saliency
{
public:
    static float* weightCurrFrame;
    static float weightSumCurrFrame;
    static int frameWidthInCtus;
    static int frameHeightInCtus;
    static float*** videoWeight;
    static double ratioInFrame; // ��ǰCTU��Ȩ��ռ֡ȫ��CTU��Ȩ�غ͵ı���
    static void initial(int frameNum, int heightNum, int widthNum);
    static void destroy(int frameNum, int heightNum, int widthNum);
    static float*** readVideoWeight(char* filename, int heightNum, int widthNum);
};
//Mao
class maoTimer
{
public:
    static double targetComp; //Ŀ�긴�Ӷ�(0-1)
    static int adaptDepth; // ÿ��CTU��������
    static double** timeInGOP3; 
    static int** numberInGOP;
    static double** weightInGOP3;
    static bool** isModeOn;
    static double weightSum; // Ȩ�غ�
    static double compCTURef; // �����и��Ӷȿ���ʱ��ƽ��ÿ��CTU�ı��븴�Ӷ�(CLOCK)
    static double weightExtraCTU;
    static double timeAllMode;
    static double timeM20CTU; // ��һ��GOP�������Ժ���Ҫ����ƽ��ÿ��CTUѡ�����20��2Nx2N��ʱ�䣬��Ϊ��Ŀ�����䵽CTU��ʱ�䣨clock��ת����weight��
    static bool isClassLow; // �Ƿ�Ϊclass low
    // static int POCOfCurrGOPStart; //��ǰGOP�ĵ�һ֡��POC
    static double compLeftLastFrame; //��һ֡ʣ��ĸ��Ӷ�
    static double compPerFramePre; // Ԥ�����ÿ֡���Ӷ�
    static double compCurrFrameAct; // ʵ������һ֡��Ŀ�긴�Ӷ� CCFA = CLLF + CPFP�����������������������������
    static double compCurrFrameLeft; // ��֡�ڣ�ʣ��ĸ��Ӷȣ�ÿ֡�ĳ�ʼֵΪcompCurrFrameAct��Ȼ���ȥ��֡�ڻ��ѵ�ʱ��
    static double compGOPTarget; //
    static double compGOPTargetAdj; //���ݹ�ʽ9�����õ���GOPʵ�ʵ�Ŀ�긴�Ӷ�
    static double ksi1; //ϣ����ĸ�Σ���ʽ(8)
    static double ksi2;
    static long start; // ÿ��ģʽ�Ŀ�ʼʱ��
    static long CTUStart; // ÿ��CTU�Ŀ�ʼʱ��
    static long totalTimeCurrCTU; // ��ǰ�ѱ���CTU��ʱ��
    static long frameStart; // ÿ֡�Ŀ�ʼʱ��
    static long seqStart; // ���п�ʼ��ʱ��
    static long totalTimeCurrFrame;
    static long totalTimeOfGOP3; // ��3��GOP����ʱ��
    static long* totalTimeEveryGOP;
    static long GOPStart; // GOP�Ŀ�ʼʱ��
    static double compCTUTarget;
    static int* numberList; //��ȡǰһ��GOP��Mij��������ת��Ϊһά����
    static int* sortingList; //���ڱ��潵�������ԭ�±�
    static void initial(int x, int y);
    static void destroy(int x, int y);
    static int* sortIndex(int s[], int pos[], int left, int right);
    static bool* goToNextDepth; // ����ȷ����һ����Ƿ�����ģʽ������������������жϵĻ���������
    static bool maoBoundary; // CTU�Ƿ������һ��
    static double* frameTimeInGOP3; // ������GOPÿ֡��ʱ��
    static double* ratioInGOP3; // ��֡����ʱ����GOP3�еı�ֵ
    static double timeFixer; // ÿ�ĸ�GOP�����һ֡���������ƣ������������б���ʱ���Ԥ��ֵ
    static double timeSeqPred; // ���е�Ԥ��ʱ�䣬POC=8������ɺ�ɻ�ȡ�����ɽ����һ��������������Ӧ����
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

