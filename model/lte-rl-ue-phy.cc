/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2010 TELEMATICS LAB, DEE - Politecnico di Bari
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Giuseppe Piro  <g.piro@poliba.it>
 *         Marco Miozzo <marco.miozzo@cttc.es>
 *         Nicola Baldo <nbaldo@cttc.es>
 */

#include <ns3/object-factory.h>
#include <ns3/log.h>
#include <ns3/node.h>
#include <cfloat>
#include <cmath>
#include <ns3/simulator.h>
#include <ns3/double.h>
#include "lte-rl-ue-phy.h"
#include "lte-enb-phy.h"
#include "lte-net-device.h"
#include "lte-ue-net-device.h"
#include "lte-enb-net-device.h"
#include "lte-spectrum-value-helper.h"
#include "lte-amc.h"
#include "lte-ue-mac.h"
#include "ff-mac-common.h"
#include "lte-chunk-processor.h"
#include <ns3/lte-common.h>
#include <ns3/pointer.h>
#include <ns3/boolean.h>
#include <ns3/lte-ue-power-control.h>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("LteRlUePhy");



/**
 * Duration of the data portion of a UL subframe.
 * Equals to "TTI length - 1 symbol length for SRS - margin".
 * The margin is 1 nanosecond and is intended to avoid overlapping simulator
 * events. The duration of one symbol is TTI/14 (rounded). In other words,
 * duration of data portion of UL subframe = 1 ms * (13/14) - 1 ns.
 */
static const Time UL_DATA_DURATION = NanoSeconds (1e6 - 71429 - 1); 

/**
 * Delay from subframe start to transmission of SRS.
 * Equals to "TTI length - 1 symbol for SRS".
 */
static const Time UL_SRS_DELAY_FROM_SUBFRAME_START = NanoSeconds (1e6 - 71429); 




////////////////////////////////////////
// member SAP forwarders
////////////////////////////////////////


class UeMemberLteRlUePhySapProvider : public LteRlUePhySapProvider
{
public:
  UeMemberLteRlUePhySapProvider (LteRlUePhy* phy);

  // inherited from LtePhySapProvider
  virtual void SendMacPdu (Ptr<Packet> p);
  virtual void SendLteControlMessage (Ptr<LteControlMessage> msg);
  virtual void SendRachPreamble (uint32_t prachId, uint32_t raRnti);

private:
  LteRlUePhy* m_phy;
};

UeMemberLteRlUePhySapProvider::UeMemberLteRlUePhySapProvider (LteRlUePhy* phy) : m_phy (phy)
{

}

void
UeMemberLteRlUePhySapProvider::SendMacPdu (Ptr<Packet> p)
{
  m_phy->DoSendMacPdu (p);
}

void
UeMemberLteRlUePhySapProvider::SendLteControlMessage (Ptr<LteControlMessage> msg)
{
  m_phy->DoSendLteControlMessage (msg);
}

void
UeMemberLteRlUePhySapProvider::SendRachPreamble (uint32_t prachId, uint32_t raRnti)
{
  m_phy->DoSendRachPreamble (prachId, raRnti);
}


////////////////////////////////////////
// LteRlUePhy methods
////////////////////////////////////////

/// Map each of UE PHY states to its string representation.
static const std::string g_uePhyStateName[LteRlUePhy::NUM_STATES] =
{
  "CELL_SEARCH",
  "SYNCHRONIZED"
};

/**
 * \param s The UE PHY state.
 * \return The string representation of the given state.
 */
static inline const std::string & ToString (LteRlUePhy::State s)
{
  return g_uePhyStateName[s];
}


NS_OBJECT_ENSURE_REGISTERED (LteRlUePhy);


LteRlUePhy::LteRlUePhy ()
{
  NS_LOG_FUNCTION (this);
  NS_FATAL_ERROR ("This constructor should not be called");
}

LteRlUePhy::LteRlUePhy (Ptr<LteSpectrumPhy> dlPhy, Ptr<LteSpectrumPhy> ulPhy)
  : LtePhy (dlPhy, ulPhy),
    m_p10CqiPeriodicity (MilliSeconds (1)),  // ideal behavior
    m_a30CqiPeriodicity (MilliSeconds (1)),  // ideal behavior
    m_uePhySapUser (0),
    m_ueCphySapUser (0),
    m_state (CELL_SEARCH),
    m_subframeNo (0),
    m_rsReceivedPowerUpdated (false),
    m_rsInterferencePowerUpdated (false),
    m_dataInterferencePowerUpdated (false),
    m_pssReceived (false),
    m_ueMeasurementsFilterPeriod (MilliSeconds (200)),
    m_ueMeasurementsFilterLast (MilliSeconds (0)),
    m_rsrpSinrSampleCounter (0)
{
  m_amc = CreateObject <LteAmc> ();
  m_powerControl = CreateObject <LteUePowerControl> ();
  m_uePhySapProvider = new UeMemberLteRlUePhySapProvider (this);
  m_ueCphySapProvider = new MemberLteUeCphySapProvider<LteRlUePhy> (this);
  m_macChTtiDelay = UL_PUSCH_TTIS_DELAY;

  NS_ASSERT_MSG (Simulator::Now ().GetNanoSeconds () == 0,
                 "Cannot create UE devices after simulation started");
  Simulator::Schedule (m_ueMeasurementsFilterPeriod, &LteRlUePhy::ReportUeMeasurements, this);

  DoReset ();
}


LteRlUePhy::~LteRlUePhy ()
{
  m_txModeGain.clear ();
}

void
LteRlUePhy::DoDispose ()
{
  NS_LOG_FUNCTION (this);
  delete m_uePhySapProvider;
  delete m_ueCphySapProvider;
  LtePhy::DoDispose ();
}



TypeId
LteRlUePhy::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::LteRlUePhy")
    .SetParent<LtePhy> ()
    .SetGroupName("Lte")
    .AddConstructor<LteRlUePhy> ()
    .AddAttribute ("TxPower",
                   "Transmission power in dBm",
                   DoubleValue (10.0),
                   MakeDoubleAccessor (&LteRlUePhy::SetTxPower, 
                                       &LteRlUePhy::GetTxPower),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("NoiseFigure",
                   "Loss (dB) in the Signal-to-Noise-Ratio due to non-idealities in the receiver."
                   " According to Wikipedia (http://en.wikipedia.org/wiki/Noise_figure), this is "
                   "\"the difference in decibels (dB) between"
                   " the noise output of the actual receiver to the noise output of an "
                   " ideal receiver with the same overall gain and bandwidth when the receivers "
                   " are connected to sources at the standard noise temperature T0.\" "
                   "In this model, we consider T0 = 290K.",
                   DoubleValue (9.0),
                   MakeDoubleAccessor (&LteRlUePhy::SetNoiseFigure, 
                                       &LteRlUePhy::GetNoiseFigure),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("TxMode1Gain",
                   "Transmission mode 1 gain in dB",
                   DoubleValue (0.0),
                   MakeDoubleAccessor (&LteRlUePhy::SetTxMode1Gain),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("TxMode2Gain",
                   "Transmission mode 2 gain in dB",
                   DoubleValue (4.2),
                   MakeDoubleAccessor (&LteRlUePhy::SetTxMode2Gain),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("TxMode3Gain",
                   "Transmission mode 3 gain in dB",
                   DoubleValue (-2.8),
                   MakeDoubleAccessor (&LteRlUePhy::SetTxMode3Gain),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("TxMode4Gain",
                   "Transmission mode 4 gain in dB",
                   DoubleValue (0.0),
                   MakeDoubleAccessor (&LteRlUePhy::SetTxMode4Gain),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("TxMode5Gain",
                   "Transmission mode 5 gain in dB",
                   DoubleValue (0.0),
                   MakeDoubleAccessor (&LteRlUePhy::SetTxMode5Gain),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("TxMode6Gain",
                   "Transmission mode 6 gain in dB",
                   DoubleValue (0.0),
                   MakeDoubleAccessor (&LteRlUePhy::SetTxMode6Gain),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("TxMode7Gain",
                   "Transmission mode 7 gain in dB",
                   DoubleValue (0.0),
                   MakeDoubleAccessor (&LteRlUePhy::SetTxMode7Gain),
                   MakeDoubleChecker<double> ())
    .AddTraceSource ("ReportCurrentCellRsrpSinr",
                     "RSRP and SINR statistics.",
                     MakeTraceSourceAccessor (&LteRlUePhy::m_reportCurrentCellRsrpSinrTrace),
                     "ns3::LteRlUePhy::RsrpSinrTracedCallback")
    .AddAttribute ("RsrpSinrSamplePeriod",
                   "The sampling period for reporting RSRP-SINR stats (default value 1)",
                   UintegerValue (1),
                   MakeUintegerAccessor (&LteRlUePhy::m_rsrpSinrSamplePeriod),
                   MakeUintegerChecker<uint16_t> ())
    .AddTraceSource ("UlPhyTransmission",
                     "DL transmission PHY layer statistics.",
                     MakeTraceSourceAccessor (&LteRlUePhy::m_ulPhyTransmission),
                     "ns3::PhyTransmissionStatParameters::TracedCallback")
    .AddAttribute ("DlSpectrumPhy",
                   "The downlink LteSpectrumPhy associated to this LtePhy",
                   TypeId::ATTR_GET,
                   PointerValue (),
                   MakePointerAccessor (&LteRlUePhy::GetDlSpectrumPhy),
                   MakePointerChecker <LteSpectrumPhy> ())
    .AddAttribute ("UlSpectrumPhy",
                   "The uplink LteSpectrumPhy associated to this LtePhy",
                   TypeId::ATTR_GET,
                   PointerValue (),
                   MakePointerAccessor (&LteRlUePhy::GetUlSpectrumPhy),
                   MakePointerChecker <LteSpectrumPhy> ())
    .AddAttribute ("RsrqUeMeasThreshold",
                   "Receive threshold for PSS on RSRQ [dB]",
                   DoubleValue (-1000.0),
                   MakeDoubleAccessor (&LteRlUePhy::m_pssReceptionThreshold),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("UeMeasurementsFilterPeriod",
                   "Time period for reporting UE measurements, i.e., the"
                   "length of layer-1 filtering.",
                   TimeValue (MilliSeconds (200)),
                   MakeTimeAccessor (&LteRlUePhy::m_ueMeasurementsFilterPeriod),
                   MakeTimeChecker ())
    .AddTraceSource ("ReportUeMeasurements",
                     "Report UE measurements RSRP (dBm) and RSRQ (dB).",
                     MakeTraceSourceAccessor (&LteRlUePhy::m_reportUeMeasurements),
                     "ns3::LteRlUePhy::RsrpRsrqTracedCallback")
    .AddTraceSource ("StateTransition",
                     "Trace fired upon every UE PHY state transition",
                     MakeTraceSourceAccessor (&LteRlUePhy::m_stateTransitionTrace),
                     "ns3::LteRlUePhy::StateTracedCallback")
    .AddAttribute ("EnableUplinkPowerControl",
                   "If true, Uplink Power Control will be enabled.",
                   BooleanValue (true),
                   MakeBooleanAccessor (&LteRlUePhy::m_enableUplinkPowerControl),
                   MakeBooleanChecker ())
  ;
  return tid;
}

void
LteRlUePhy::DoInitialize ()
{
  NS_LOG_FUNCTION (this);
  bool haveNodeId = false;
  uint32_t nodeId = 0;
  if (m_netDevice != 0)
    {
      Ptr<Node> node = m_netDevice->GetNode ();
      if (node != 0)
        {
          nodeId = node->GetId ();
          haveNodeId = true;
        }
    }
  if (haveNodeId)
    {
      Simulator::ScheduleWithContext (nodeId, Seconds (0), &LteRlUePhy::SubframeIndication, this, 1, 1);
    }
  else
    {
      Simulator::ScheduleNow (&LteRlUePhy::SubframeIndication, this, 1, 1);
    }  
  LtePhy::DoInitialize ();
}

void
LteRlUePhy::SetLteRlUePhySapUser (LteRlUePhySapUser* s)
{
  NS_LOG_FUNCTION (this);
  m_uePhySapUser = s;
}

LteRlUePhySapProvider*
LteRlUePhy::GetLteRlUePhySapProvider ()
{
  NS_LOG_FUNCTION (this);
  return (m_uePhySapProvider);
}


void
LteRlUePhy::SetLteUeCphySapUser (LteUeCphySapUser* s)
{
  NS_LOG_FUNCTION (this);
  m_ueCphySapUser = s;
}

LteUeCphySapProvider*
LteRlUePhy::GetLteUeCphySapProvider ()
{
  NS_LOG_FUNCTION (this);
  return (m_ueCphySapProvider);
}

void
LteRlUePhy::SetNoiseFigure (double nf)
{
  NS_LOG_FUNCTION (this << nf);
  m_noiseFigure = nf;
}

double
LteRlUePhy::GetNoiseFigure () const
{
  NS_LOG_FUNCTION (this);
  return m_noiseFigure;
}

void
LteRlUePhy::SetTxPower (double pow)
{
  NS_LOG_FUNCTION (this << pow);
  m_txPower = pow;
  m_powerControl->SetTxPower (pow);
}

double
LteRlUePhy::GetTxPower () const
{
  NS_LOG_FUNCTION (this);
  return m_txPower;
}

Ptr<LteUePowerControl>
LteRlUePhy::GetUplinkPowerControl () const
{
  NS_LOG_FUNCTION (this);
  return m_powerControl;
}

uint8_t
LteRlUePhy::GetMacChDelay (void) const
{
  return (m_macChTtiDelay);
}

Ptr<LteSpectrumPhy>
LteRlUePhy::GetDlSpectrumPhy () const
{
  return m_downlinkSpectrumPhy;
}

Ptr<LteSpectrumPhy>
LteRlUePhy::GetUlSpectrumPhy () const
{
  return m_uplinkSpectrumPhy;
}

void
LteRlUePhy::DoSendMacPdu (Ptr<Packet> p)
{
  NS_LOG_FUNCTION (this);

  SetMacPdu (p);
}


void
LteRlUePhy::PhyPduReceived (Ptr<Packet> p)
{
  m_uePhySapUser->ReceivePhyPdu (p);
}

void
LteRlUePhy::SetSubChannelsForTransmission (std::vector <int> mask)
{
  NS_LOG_FUNCTION (this);

  m_subChannelsForTransmission = mask;

  Ptr<SpectrumValue> txPsd = CreateTxPowerSpectralDensity ();
  m_uplinkSpectrumPhy->SetTxPowerSpectralDensity (txPsd);
}


void
LteRlUePhy::SetSubChannelsForReception (std::vector <int> mask)
{
  NS_LOG_FUNCTION (this);
  m_subChannelsForReception = mask;
}


std::vector <int>
LteRlUePhy::GetSubChannelsForTransmission ()
{
  NS_LOG_FUNCTION (this);
  return m_subChannelsForTransmission;
}


std::vector <int>
LteRlUePhy::GetSubChannelsForReception ()
{
  NS_LOG_FUNCTION (this);
  return m_subChannelsForReception;
}


Ptr<SpectrumValue>
LteRlUePhy::CreateTxPowerSpectralDensity ()
{
  NS_LOG_FUNCTION (this);
  LteSpectrumValueHelper psdHelper;
  Ptr<SpectrumValue> psd = psdHelper.CreateTxPowerSpectralDensity (m_ulEarfcn, m_ulBandwidth, m_txPower, GetSubChannelsForTransmission ());

  return psd;
}

void
LteRlUePhy::GenerateCtrlCqiReport (const SpectrumValue& sinr)
{
  NS_LOG_FUNCTION (this);
  
  GenerateCqiRsrpRsrq (sinr);
}

void
LteRlUePhy::GenerateCqiRsrpRsrq (const SpectrumValue& sinr)
{
  NS_LOG_FUNCTION (this << sinr);

  NS_ASSERT (m_state != CELL_SEARCH);
  NS_ASSERT (m_cellId > 0);

  if (m_dlConfigured && m_ulConfigured && (m_rnti > 0))
    {
      // check periodic wideband CQI
      if (Simulator::Now () > m_p10CqiLast + m_p10CqiPeriodicity)
        {
          Ptr<LteUeNetDevice> thisDevice = GetDevice ()->GetObject<LteUeNetDevice> ();
          Ptr<DlCqiLteControlMessage> msg = CreateDlCqiFeedbackMessage (sinr);
          if (msg)
            {
              DoSendLteControlMessage (msg);
            }
          m_p10CqiLast = Simulator::Now ();
        }
      // check aperiodic high-layer configured subband CQI
      if  (Simulator::Now () > m_a30CqiLast + m_a30CqiPeriodicity)
        {
          Ptr<LteUeNetDevice> thisDevice = GetDevice ()->GetObject<LteUeNetDevice> ();
          Ptr<DlCqiLteControlMessage> msg = CreateDlCqiFeedbackMessage (sinr);
          if (msg)
            {
              DoSendLteControlMessage (msg);
            }
          m_a30CqiLast = Simulator::Now ();
        }
    }

  // Generate PHY trace
  m_rsrpSinrSampleCounter++;
  if (m_rsrpSinrSampleCounter==m_rsrpSinrSamplePeriod)
    {
      NS_ASSERT_MSG (m_rsReceivedPowerUpdated, " RS received power info obsolete");
      // RSRP evaluated as averaged received power among RBs
      double sum = 0.0;
      uint8_t rbNum = 0;
      Values::const_iterator it;
      for (it = m_rsReceivedPower.ConstValuesBegin (); it != m_rsReceivedPower.ConstValuesEnd (); it++)
        {
          // convert PSD [W/Hz] to linear power [W] for the single RE
          // we consider only one RE for the RS since the channel is 
          // flat within the same RB 
          double powerTxW = ((*it) * 180000.0) / 12.0;
          sum += powerTxW;
          rbNum++;
        }
      double rsrp = (rbNum > 0) ? (sum / rbNum) : DBL_MAX;
      // averaged SINR among RBs
      sum = 0.0;
      rbNum = 0;
      for (it = sinr.ConstValuesBegin (); it != sinr.ConstValuesEnd (); it++)
        {
          sum += (*it);
          rbNum++;
        }
      double avSinr = (rbNum > 0) ? (sum / rbNum) : DBL_MAX;
      NS_LOG_INFO (this << " cellId " << m_cellId << " rnti " << m_rnti << " RSRP " << rsrp << " SINR " << avSinr);

      m_reportCurrentCellRsrpSinrTrace (m_cellId, m_rnti, rsrp, avSinr);
      m_rsrpSinrSampleCounter = 0;
    }

  if (m_pssReceived)
    {
      // measure instantaneous RSRQ now
      NS_ASSERT_MSG (m_rsInterferencePowerUpdated, " RS interference power info obsolete");

      std::list <PssElement>::iterator itPss = m_pssList.begin ();
      while (itPss != m_pssList.end ())
        {
          uint16_t rbNum = 0;
          double rssiSum = 0.0;

          Values::const_iterator itIntN = m_rsInterferencePower.ConstValuesBegin ();
          Values::const_iterator itPj = m_rsReceivedPower.ConstValuesBegin ();
          for (itPj = m_rsReceivedPower.ConstValuesBegin ();
               itPj != m_rsReceivedPower.ConstValuesEnd ();
               itIntN++, itPj++)
            {
              rbNum++;
              // convert PSD [W/Hz] to linear power [W] for the single RE
              double interfPlusNoisePowerTxW = ((*itIntN) * 180000.0) / 12.0;
              double signalPowerTxW = ((*itPj) * 180000.0) / 12.0;
              rssiSum += (2 * (interfPlusNoisePowerTxW + signalPowerTxW));
            }

          NS_ASSERT (rbNum == (*itPss).nRB);
          double rsrq_dB = 10 * log10 ((*itPss).pssPsdSum / rssiSum);

          if (rsrq_dB > m_pssReceptionThreshold)
            {
              NS_LOG_INFO (this << " PSS RNTI " << m_rnti << " cellId " << m_cellId
                                << " has RSRQ " << rsrq_dB << " and RBnum " << rbNum);
              // store measurements
              std::map <uint16_t, UeMeasurementsElement>::iterator itMeasMap;
              itMeasMap = m_ueMeasurementsMap.find ((*itPss).cellId);
              if (itMeasMap != m_ueMeasurementsMap.end ())
                {
                  (*itMeasMap).second.rsrqSum += rsrq_dB;
                  (*itMeasMap).second.rsrqNum++;
                }
              else
                {
                  NS_LOG_WARN ("race condition of bug 2091 occurred");
                }
            }

          itPss++;

        } // end of while (itPss != m_pssList.end ())

      m_pssList.clear ();

    } // end of if (m_pssReceived)

} // end of void LteRlUePhy::GenerateCtrlCqiReport (const SpectrumValue& sinr)

void
LteRlUePhy::GenerateDataCqiReport (const SpectrumValue& sinr)
{
  // Not used by UE, CQI are based only on RS
}

void
LteRlUePhy::GenerateMixedCqiReport (const SpectrumValue& sinr)
{
  NS_LOG_FUNCTION (this);

  NS_ASSERT (m_state != CELL_SEARCH);
  NS_ASSERT (m_cellId > 0);
  
  SpectrumValue mixedSinr = (m_rsReceivedPower * m_paLinear);
  if (m_dataInterferencePowerUpdated)
    {
      // we have a measurement of interf + noise for the denominator
      // of SINR = S/(I+N)
      mixedSinr /= m_dataInterferencePower;
      m_dataInterferencePowerUpdated = false;
      NS_LOG_LOGIC ("data interf measurement available, SINR = " << mixedSinr);
    }
  else
    {
      // we did not see any interference on data, so interference is
      // there and we have only noise at the denominator of SINR
      mixedSinr /= (*m_noisePsd);
      NS_LOG_LOGIC ("no data interf measurement available, SINR = " << mixedSinr);
    }

  /*
   * some RBs are not used in PDSCH and their SINR is very high
   * for example with bandwidth 25, last RB is not used
   * it can make avgSinr value very high, what is incorrect
   */
  uint32_t rbgSize = GetRbgSize ();
  uint32_t modulo = m_dlBandwidth % rbgSize;
  double avgMixedSinr = 0;
  uint32_t usedRbgNum = 0;
  for(uint32_t i = 0; i < (m_dlBandwidth-1-modulo); i++) 
    {
      usedRbgNum++;
      avgMixedSinr+=mixedSinr[i];
    }
  avgMixedSinr = avgMixedSinr/usedRbgNum;
  for(uint32_t i = 0; i < modulo; i++) 
    {
      mixedSinr[m_dlBandwidth-1-i] = avgMixedSinr;
    }

  GenerateCqiRsrpRsrq (mixedSinr);
}

void
LteRlUePhy::ReportInterference (const SpectrumValue& interf)
{
  NS_LOG_FUNCTION (this << interf);
  m_rsInterferencePowerUpdated = true;
  m_rsInterferencePower = interf;
}

void
LteRlUePhy::ReportDataInterference (const SpectrumValue& interf)
{
  NS_LOG_FUNCTION (this << interf);

  m_dataInterferencePowerUpdated = true;
  m_dataInterferencePower = interf;
}

void
LteRlUePhy::ReportRsReceivedPower (const SpectrumValue& power)
{
  NS_LOG_FUNCTION (this << power);
  m_rsReceivedPowerUpdated = true;
  m_rsReceivedPower = power;

  if (m_enableUplinkPowerControl)
    {
      double sum = 0;
      uint32_t rbNum = 0;
      Values::const_iterator it;
      for (it = m_rsReceivedPower.ConstValuesBegin (); it != m_rsReceivedPower.ConstValuesEnd (); it++)
        {
          double powerTxW = ((*it) * 180000);
          sum += powerTxW;
          rbNum++;
        }
      double rsrp = 10 * log10 (sum) + 30;

      NS_LOG_INFO ("RSRP: " << rsrp);
      m_powerControl->SetRsrp (rsrp);
    }
}

Ptr<DlCqiLteControlMessage>
LteRlUePhy::CreateDlCqiFeedbackMessage (const SpectrumValue& sinr)
{
  NS_LOG_FUNCTION (this);


  // apply transmission mode gain
  NS_ASSERT (m_transmissionMode < m_txModeGain.size ());
  SpectrumValue newSinr = sinr;
  newSinr *= m_txModeGain.at (m_transmissionMode);

  // CREATE DlCqiLteControlMessage
  Ptr<DlCqiLteControlMessage> msg = Create<DlCqiLteControlMessage> ();
  CqiListElement_s dlcqi;
  std::vector<int> cqi;
  if (Simulator::Now () > m_p10CqiLast + m_p10CqiPeriodicity)
    {
      cqi = m_amc->CreateCqiFeedbacks (newSinr, m_dlBandwidth);

      int nLayer = TransmissionModesLayers::TxMode2LayerNum (m_transmissionMode);
      int nbSubChannels = cqi.size ();
      double cqiSum = 0.0;
      int activeSubChannels = 0;
      // average the CQIs of the different RBs
      for (int i = 0; i < nbSubChannels; i++)
        {
          if (cqi.at (i) != -1)
            {
              cqiSum += cqi.at (i);
              activeSubChannels++;
            }
          NS_LOG_DEBUG (this << " subch " << i << " cqi " <<  cqi.at (i));
        }
      dlcqi.m_rnti = m_rnti;
      dlcqi.m_ri = 1; // not yet used
      dlcqi.m_cqiType = CqiListElement_s::P10; // Peridic CQI using PUCCH wideband
      NS_ASSERT_MSG (nLayer > 0, " nLayer negative");
      NS_ASSERT_MSG (nLayer < 3, " nLayer limit is 2s");
      for (int i = 0; i < nLayer; i++)
        {
          if (activeSubChannels > 0)
            {
              dlcqi.m_wbCqi.push_back ((uint16_t) cqiSum / activeSubChannels);
            }
          else
            {
              // approximate with the worst case -> CQI = 1
              dlcqi.m_wbCqi.push_back (1);
            }
        }
      //NS_LOG_DEBUG (this << " Generate P10 CQI feedback " << (uint16_t) cqiSum / activeSubChannels);
      dlcqi.m_wbPmi = 0; // not yet used
      // dl.cqi.m_sbMeasResult others CQI report modes: not yet implemented
    }
  else if (Simulator::Now () > m_a30CqiLast + m_a30CqiPeriodicity)
    {
      cqi = m_amc->CreateCqiFeedbacks (newSinr, GetRbgSize ());
      int nLayer = TransmissionModesLayers::TxMode2LayerNum (m_transmissionMode);
      int nbSubChannels = cqi.size ();
      int rbgSize = GetRbgSize ();
      double cqiSum = 0.0;
      int cqiNum = 0;
      SbMeasResult_s rbgMeas;
      //NS_LOG_DEBUG (this << " Create A30 CQI feedback, RBG " << rbgSize << " cqiNum " << nbSubChannels << " band "  << (uint16_t)m_dlBandwidth);
      for (int i = 0; i < nbSubChannels; i++)
        {
          if (cqi.at (i) != -1)
            {
              cqiSum += cqi.at (i);
            }
          // else "nothing" no CQI is treated as CQI = 0 (worst case scenario)
          cqiNum++;
          if (cqiNum == rbgSize)
            {
              // average the CQIs of the different RBGs
              //NS_LOG_DEBUG (this << " RBG CQI "  << (uint16_t) cqiSum / rbgSize);
              HigherLayerSelected_s hlCqi;
              hlCqi.m_sbPmi = 0; // not yet used
              for (int i = 0; i < nLayer; i++)
                {
                  hlCqi.m_sbCqi.push_back ((uint16_t) cqiSum / rbgSize);
                }
              rbgMeas.m_higherLayerSelected.push_back (hlCqi);
              cqiSum = 0.0;
              cqiNum = 0;
            }
        }
      dlcqi.m_rnti = m_rnti;
      dlcqi.m_ri = 1; // not yet used
      dlcqi.m_cqiType = CqiListElement_s::A30; // Aperidic CQI using PUSCH
      //dlcqi.m_wbCqi.push_back ((uint16_t) cqiSum / nbSubChannels);
      dlcqi.m_wbPmi = 0; // not yet used
      dlcqi.m_sbMeasResult = rbgMeas;
    }

  msg->SetDlCqi (dlcqi);
  return msg;
}


void
LteRlUePhy::ReportUeMeasurements ()
{
  NS_LOG_FUNCTION (this << Simulator::Now ());
  NS_LOG_DEBUG (this << " Report UE Measurements ");

  LteUeCphySapUser::UeMeasurementsParameters ret;

  std::map <uint16_t, UeMeasurementsElement>::iterator it;
  for (it = m_ueMeasurementsMap.begin (); it != m_ueMeasurementsMap.end (); it++)
    {
      double avg_rsrp = (*it).second.rsrpSum / (double)(*it).second.rsrpNum;
      double avg_rsrq = (*it).second.rsrqSum / (double)(*it).second.rsrqNum;
      /*
       * In CELL_SEARCH state, this may result in avg_rsrq = 0/0 = -nan.
       * UE RRC must take this into account when receiving measurement reports.
       * TODO remove this shortcoming by calculating RSRQ during CELL_SEARCH
       */
      NS_LOG_DEBUG (this << " CellId " << (*it).first
                         << " RSRP " << avg_rsrp
                         << " (nSamples " << (uint16_t)(*it).second.rsrpNum << ")"
                         << " RSRQ " << avg_rsrq
                         << " (nSamples " << (uint16_t)(*it).second.rsrqNum << ")");

      LteUeCphySapUser::UeMeasurementsElement newEl;
      newEl.m_cellId = (*it).first;
      newEl.m_rsrp = avg_rsrp;
      newEl.m_rsrq = avg_rsrq;
      ret.m_ueMeasurementsList.push_back (newEl);

      // report to UE measurements trace
      m_reportUeMeasurements (m_rnti, (*it).first, avg_rsrp, avg_rsrq, ((*it).first == m_cellId ? 1 : 0));
    }

  // report to RRC
  m_ueCphySapUser->ReportUeMeasurements (ret);

  m_ueMeasurementsMap.clear ();
  Simulator::Schedule (m_ueMeasurementsFilterPeriod, &LteRlUePhy::ReportUeMeasurements, this);
}

void
LteRlUePhy::DoSendLteControlMessage (Ptr<LteControlMessage> msg)
{
  NS_LOG_FUNCTION (this << msg);

  SetControlMessages (msg);
}

void 
LteRlUePhy::DoSendRachPreamble (uint32_t raPreambleId, uint32_t raRnti)
{
  NS_LOG_FUNCTION (this << raPreambleId);

  // unlike other control messages, RACH preamble is sent ASAP
  Ptr<RachPreambleLteControlMessage> msg = Create<RachPreambleLteControlMessage> ();
  msg->SetRapId (raPreambleId);
  m_raPreambleId = raPreambleId;
  m_raRnti = raRnti;
  m_controlMessagesQueue.at (0).push_back (msg);
}


void
LteRlUePhy::ReceiveLteControlMessageList (std::list<Ptr<LteControlMessage> > msgList)
{
  NS_LOG_FUNCTION (this);

  std::list<Ptr<LteControlMessage> >::iterator it;
  for (it = msgList.begin (); it != msgList.end (); it++)
    {
      Ptr<LteControlMessage> msg = (*it);

      if (msg->GetMessageType () == LteControlMessage::DL_DCI)
        {
          Ptr<DlDciLteControlMessage> msg2 = DynamicCast<DlDciLteControlMessage> (msg);

          DlDciListElement_s dci = msg2->GetDci ();
          if (dci.m_rnti != m_rnti)
            {
              // DCI not for me
              continue;
            }

          if (dci.m_resAlloc != 0)
            {
              NS_FATAL_ERROR ("Resource Allocation type not implemented");
            }

          std::vector <int> dlRb;

          // translate the DCI to Spectrum framework
          uint32_t mask = 0x1;
          for (int i = 0; i < 32; i++)
            {
              if (((dci.m_rbBitmap & mask) >> i) == 1)
                {
                  for (int k = 0; k < GetRbgSize (); k++)
                    {
                      dlRb.push_back ((i * GetRbgSize ()) + k);
//             NS_LOG_DEBUG(this << " RNTI " << m_rnti << " RBG " << i << " DL-DCI allocated PRB " << (i*GetRbgSize()) + k);
                    }
                }
              mask = (mask << 1);
            }
          if (m_enableUplinkPowerControl)
            {
              m_powerControl->ReportTpc (dci.m_tpc);
            }


          // send TB info to LteSpectrumPhy
          NS_LOG_DEBUG (this << " UE " << m_rnti << " DL-DCI " << dci.m_rnti << " bitmap "  << dci.m_rbBitmap);
          for (uint8_t i = 0; i < dci.m_tbsSize.size (); i++)
            {
              m_downlinkSpectrumPhy->AddExpectedTb (dci.m_rnti, dci.m_ndi.at (i), dci.m_tbsSize.at (i), dci.m_mcs.at (i), dlRb, i, dci.m_harqProcess, dci.m_rv.at (i), true /* DL */);
            }

          SetSubChannelsForReception (dlRb);


        }
      else if (msg->GetMessageType () == LteControlMessage::UL_DCI)
        {
          // set the uplink bandwidth according to the UL-CQI
          Ptr<UlDciLteControlMessage> msg2 = DynamicCast<UlDciLteControlMessage> (msg);
          UlDciListElement_s dci = msg2->GetDci ();
          if (dci.m_rnti != m_rnti)
            {
              // DCI not for me
              continue;
            }
          NS_LOG_INFO (this << " UL DCI");
          std::vector <int> ulRb;
          for (int i = 0; i < dci.m_rbLen; i++)
            {
              ulRb.push_back (i + dci.m_rbStart);
              //NS_LOG_DEBUG (this << " UE RB " << i + dci.m_rbStart);
            }
          QueueSubChannelsForTransmission (ulRb);
          // fire trace of UL Tx PHY stats
          HarqProcessInfoList_t harqInfoList = m_harqPhyModule->GetHarqProcessInfoUl (m_rnti, 0);
          PhyTransmissionStatParameters params;
          params.m_cellId = m_cellId;
          params.m_imsi = 0; // it will be set by DlPhyTransmissionCallback in LteHelper
          params.m_timestamp = Simulator::Now ().GetMilliSeconds () + UL_PUSCH_TTIS_DELAY;
          params.m_rnti = m_rnti;
          params.m_txMode = 0; // always SISO for UE
          params.m_layer = 0;
          params.m_mcs = dci.m_mcs;
          params.m_size = dci.m_tbSize;
          params.m_rv = harqInfoList.size ();
          params.m_ndi = dci.m_ndi;
          m_ulPhyTransmission (params);
          // pass the info to the MAC
          m_uePhySapUser->ReceiveLteControlMessage (msg);
        }
      else if (msg->GetMessageType () == LteControlMessage::RAR)
        {
          Ptr<RarLteControlMessage> rarMsg = DynamicCast<RarLteControlMessage> (msg);
          if (rarMsg->GetRaRnti () == m_raRnti)
            {
              for (std::list<RarLteControlMessage::Rar>::const_iterator it = rarMsg->RarListBegin (); it != rarMsg->RarListEnd (); ++it)
                {
                  if (it->rapId != m_raPreambleId)
                    {
                      // UL grant not for me
                      continue;
                    }
                  else
                    {
                      NS_LOG_INFO ("received RAR RNTI " << m_raRnti);
                      // set the uplink bandwidht according to the UL grant
                      std::vector <int> ulRb;
                      for (int i = 0; i < it->rarPayload.m_grant.m_rbLen; i++)
                        {
                          ulRb.push_back (i + it->rarPayload.m_grant.m_rbStart);
                        }

                      QueueSubChannelsForTransmission (ulRb);
                      // pass the info to the MAC
                      m_uePhySapUser->ReceiveLteControlMessage (msg);
                      // reset RACH variables with out of range values
                      m_raPreambleId = 255;
                      m_raRnti = 11;
                    }
                }
            }
        }
      else if (msg->GetMessageType () == LteControlMessage::MIB)
        {
          NS_LOG_INFO ("received MIB");
          NS_ASSERT (m_cellId > 0);
          Ptr<MibLteControlMessage> msg2 = DynamicCast<MibLteControlMessage> (msg);
          m_ueCphySapUser->RecvMasterInformationBlock (m_cellId, msg2->GetMib ());
        }
      else if (msg->GetMessageType () == LteControlMessage::SIB1)
        {
          NS_LOG_INFO ("received SIB1");
          NS_ASSERT (m_cellId > 0);
          Ptr<Sib1LteControlMessage> msg2 = DynamicCast<Sib1LteControlMessage> (msg);
          m_ueCphySapUser->RecvSystemInformationBlockType1 (m_cellId, msg2->GetSib1 ());
        }
      else
        {
          // pass the message to UE-MAC
          m_uePhySapUser->ReceiveLteControlMessage (msg);
        }

    }


}


void
LteRlUePhy::ReceivePss (uint16_t cellId, Ptr<SpectrumValue> p)
{
  NS_LOG_FUNCTION (this << cellId << (*p));

  double sum = 0.0;
  uint16_t nRB = 0;
  Values::const_iterator itPi;
  for (itPi = p->ConstValuesBegin (); itPi != p->ConstValuesEnd (); itPi++)
    {
      // convert PSD [W/Hz] to linear power [W] for the single RE
      double powerTxW = ((*itPi) * 180000.0) / 12.0;
      sum += powerTxW;
      nRB++;
    }

  // measure instantaneous RSRP now
  double rsrp_dBm = 10 * log10 (1000 * (sum / (double)nRB));
  NS_LOG_INFO (this << " PSS RNTI " << m_rnti << " cellId " << m_cellId
                    << " has RSRP " << rsrp_dBm << " and RBnum " << nRB);
  // note that m_pssReceptionThreshold does not apply here

  // store measurements
  std::map <uint16_t, UeMeasurementsElement>::iterator itMeasMap = m_ueMeasurementsMap.find (cellId);
  if (itMeasMap == m_ueMeasurementsMap.end ())
    {
      // insert new entry
      UeMeasurementsElement newEl;
      newEl.rsrpSum = rsrp_dBm;
      newEl.rsrpNum = 1;
      newEl.rsrqSum = 0;
      newEl.rsrqNum = 0;
      m_ueMeasurementsMap.insert (std::pair <uint16_t, UeMeasurementsElement> (cellId, newEl));
    }
  else
    {
      (*itMeasMap).second.rsrpSum += rsrp_dBm;
      (*itMeasMap).second.rsrpNum++;
    }

  /*
   * Collect the PSS for later processing in GenerateCtrlCqiReport()
   * (to be called from ChunkProcessor after RX is finished).
   */
  m_pssReceived = true;
  PssElement el;
  el.cellId = cellId;
  el.pssPsdSum = sum;
  el.nRB = nRB;
  m_pssList.push_back (el);

} // end of void LteRlUePhy::ReceivePss (uint16_t cellId, Ptr<SpectrumValue> p)


void
LteRlUePhy::QueueSubChannelsForTransmission (std::vector <int> rbMap)
{
  m_subChannelsForTransmissionQueue.at (m_macChTtiDelay - 1) = rbMap;
}


void
LteRlUePhy::SubframeIndication (uint32_t frameNo, uint32_t subframeNo)
{
  NS_LOG_FUNCTION (this << frameNo << subframeNo);

  NS_ASSERT_MSG (frameNo > 0, "the SRS index check code assumes that frameNo starts at 1");

  // refresh internal variables
  m_rsReceivedPowerUpdated = false;
  m_rsInterferencePowerUpdated = false;
  m_pssReceived = false;

  if (m_ulConfigured)
    {
      // update uplink transmission mask according to previous UL-CQIs
      std::vector <int> rbMask = m_subChannelsForTransmissionQueue.at (0);
      SetSubChannelsForTransmission (m_subChannelsForTransmissionQueue.at (0));

      // shift the queue
      for (uint8_t i = 1; i < m_macChTtiDelay; i++)
        {
          m_subChannelsForTransmissionQueue.at (i-1) = m_subChannelsForTransmissionQueue.at (i);
        }
      m_subChannelsForTransmissionQueue.at (m_macChTtiDelay-1).clear ();

      if (m_srsConfigured && (m_srsStartTime <= Simulator::Now ()))
        {

          NS_ASSERT_MSG (subframeNo > 0 && subframeNo <= 10, "the SRS index check code assumes that subframeNo starts at 1");
          if ((((frameNo-1)*10 + (subframeNo-1)) % m_srsPeriodicity) == m_srsSubframeOffset)
            {
              NS_LOG_INFO ("frame " << frameNo << " subframe " << subframeNo << " sending SRS (offset=" << m_srsSubframeOffset << ", period=" << m_srsPeriodicity << ")");
              m_sendSrsEvent = Simulator::Schedule (UL_SRS_DELAY_FROM_SUBFRAME_START, 
                                                    &LteRlUePhy::SendSrs,
                                                    this);
            }
        }

      std::list<Ptr<LteControlMessage> > ctrlMsg = GetControlMessages ();
      // send packets in queue
      NS_LOG_LOGIC (this << " UE - start slot for PUSCH + PUCCH - RNTI " << m_rnti << " CELLID " << m_cellId);
      // send the current burts of packets
      Ptr<PacketBurst> pb = GetPacketBurst ();
      if (pb)
        {
          if (m_enableUplinkPowerControl)
            {
              m_txPower = m_powerControl->GetPuschTxPower (rbMask);
              SetSubChannelsForTransmission (rbMask);
            }
          m_uplinkSpectrumPhy->StartTxDataFrame (pb, ctrlMsg, UL_DATA_DURATION);
        }
      else
        {
          // send only PUCCH (ideal: fake null bandwidth signal)
          if (ctrlMsg.size ()>0)
            {
              NS_LOG_LOGIC (this << " UE - start TX PUCCH (NO PUSCH)");
              std::vector <int> dlRb;

              if (m_enableUplinkPowerControl)
                {
                  m_txPower = m_powerControl->GetPucchTxPower (dlRb);
                }

              SetSubChannelsForTransmission (dlRb);
              m_uplinkSpectrumPhy->StartTxDataFrame (pb, ctrlMsg, UL_DATA_DURATION);
            }
          else
            {
              NS_LOG_LOGIC (this << " UE - UL NOTHING TO SEND");
            }
        }
    }  // m_configured

  // trigger the MAC
  m_uePhySapUser->SubframeIndication (frameNo, subframeNo);

  m_subframeNo = subframeNo;
  ++subframeNo;
  if (subframeNo > 10)
    {
      ++frameNo;
      subframeNo = 1;
    }

  // schedule next subframe indication
  Simulator::Schedule (Seconds (GetTti ()), &LteRlUePhy::SubframeIndication, this, frameNo, subframeNo);
}


void
LteRlUePhy::SendSrs ()
{
  NS_LOG_FUNCTION (this << " UE " << m_rnti << " start tx SRS, cell Id " << (uint32_t) m_cellId);
  NS_ASSERT (m_cellId > 0);
  // set the current tx power spectral density (full bandwidth)
  std::vector <int> dlRb;
  for (uint8_t i = 0; i < m_ulBandwidth; i++)
    {
      dlRb.push_back (i);
    }

  if (m_enableUplinkPowerControl)
    {
      m_txPower = m_powerControl->GetSrsTxPower (dlRb);
    }

  SetSubChannelsForTransmission (dlRb);
  m_uplinkSpectrumPhy->StartTxUlSrsFrame ();
}


void
LteRlUePhy::DoReset ()
{
  NS_LOG_FUNCTION (this);

  m_rnti = 0;
  m_transmissionMode = 0;
  m_srsPeriodicity = 0;
  m_srsConfigured = false;
  m_dlConfigured = false;
  m_ulConfigured = false;
  m_raPreambleId = 255; // value out of range
  m_raRnti = 11; // value out of range
  m_rsrpSinrSampleCounter = 0;
  m_p10CqiLast = Simulator::Now ();
  m_a30CqiLast = Simulator::Now ();
  m_paLinear = 1;

  m_packetBurstQueue.clear ();
  m_controlMessagesQueue.clear ();
  m_subChannelsForTransmissionQueue.clear ();
  for (int i = 0; i < m_macChTtiDelay; i++)
    {
      Ptr<PacketBurst> pb = CreateObject <PacketBurst> ();
      m_packetBurstQueue.push_back (pb);
      std::list<Ptr<LteControlMessage> > l;
      m_controlMessagesQueue.push_back (l);
    }
  std::vector <int> ulRb;
  m_subChannelsForTransmissionQueue.resize (m_macChTtiDelay, ulRb);

  m_sendSrsEvent.Cancel ();
  m_downlinkSpectrumPhy->Reset ();
  m_uplinkSpectrumPhy->Reset ();

} // end of void LteRlUePhy::DoReset ()

void
LteRlUePhy::DoStartCellSearch (uint16_t dlEarfcn)
{
  NS_LOG_FUNCTION (this << dlEarfcn);
  m_dlEarfcn = dlEarfcn;
  DoSetDlBandwidth (6); // configure DL for receiving PSS
  SwitchToState (CELL_SEARCH);
}

void
LteRlUePhy::DoSynchronizeWithEnb (uint16_t cellId, uint16_t dlEarfcn)
{
  NS_LOG_FUNCTION (this << cellId << dlEarfcn);
  m_dlEarfcn = dlEarfcn;
  DoSynchronizeWithEnb (cellId);
}

void
LteRlUePhy::DoSynchronizeWithEnb (uint16_t cellId)
{
  NS_LOG_FUNCTION (this << cellId);

  if (cellId == 0)
    {
      NS_FATAL_ERROR ("Cell ID shall not be zero");
    }

  m_cellId = cellId;
  m_downlinkSpectrumPhy->SetCellId (cellId);
  m_uplinkSpectrumPhy->SetCellId (cellId);

  // configure DL for receiving the BCH with the minimum bandwidth
  DoSetDlBandwidth (6);

  m_dlConfigured = false;
  m_ulConfigured = false;

  SwitchToState (SYNCHRONIZED);
}

void
LteRlUePhy::DoSetDlBandwidth (uint8_t dlBandwidth)
{
  NS_LOG_FUNCTION (this << (uint32_t) dlBandwidth);
  if (m_dlBandwidth != dlBandwidth or !m_dlConfigured)
    {
      m_dlBandwidth = dlBandwidth;

      static const int Type0AllocationRbg[4] = {
        10,     // RGB size 1
        26,     // RGB size 2
        63,     // RGB size 3
        110     // RGB size 4
      };  // see table 7.1.6.1-1 of 36.213
      for (int i = 0; i < 4; i++)
        {
          if (dlBandwidth < Type0AllocationRbg[i])
            {
              m_rbgSize = i + 1;
              break;
            }
        }

      m_noisePsd = LteSpectrumValueHelper::CreateNoisePowerSpectralDensity (m_dlEarfcn, m_dlBandwidth, m_noiseFigure);
      m_downlinkSpectrumPhy->SetNoisePowerSpectralDensity (m_noisePsd);
      m_downlinkSpectrumPhy->GetChannel ()->AddRx (m_downlinkSpectrumPhy);
    }
  m_dlConfigured = true;
}


void 
LteRlUePhy::DoConfigureUplink (uint16_t ulEarfcn, uint8_t ulBandwidth)
{
  m_ulEarfcn = ulEarfcn;
  m_ulBandwidth = ulBandwidth;
  m_ulConfigured = true;
}

void
LteRlUePhy::DoConfigureReferenceSignalPower (int8_t referenceSignalPower)
{
  NS_LOG_FUNCTION (this);
  m_powerControl->ConfigureReferenceSignalPower (referenceSignalPower);
}
 
void
LteRlUePhy::DoSetRnti (uint16_t rnti)
{
  NS_LOG_FUNCTION (this << rnti);
  m_rnti = rnti;

  m_powerControl->SetCellId (m_cellId);
  m_powerControl->SetRnti (m_rnti);
}
 
void
LteRlUePhy::DoSetTransmissionMode (uint8_t txMode)
{
  NS_LOG_FUNCTION (this << (uint16_t)txMode);
  m_transmissionMode = txMode;
  m_downlinkSpectrumPhy->SetTransmissionMode (txMode);
}

void
LteRlUePhy::DoSetSrsConfigurationIndex (uint16_t srcCi)
{
  NS_LOG_FUNCTION (this << srcCi);
  m_srsPeriodicity = GetSrsPeriodicity (srcCi);
  m_srsSubframeOffset = GetSrsSubframeOffset (srcCi);
  m_srsConfigured = true;

  // a guard time is needed for the case where the SRS periodicity is changed dynamically at run time
  // if we use a static one, we can have a 0ms guard time
  m_srsStartTime = Simulator::Now () + MilliSeconds (0);
  NS_LOG_DEBUG (this << " UE SRS P " << m_srsPeriodicity << " RNTI " << m_rnti << " offset " << m_srsSubframeOffset << " cellId " << m_cellId << " CI " << srcCi);
}

void
LteRlUePhy::DoSetPa (double pa)
{
  NS_LOG_FUNCTION (this << pa);
  m_paLinear = pow (10,(pa/10));
}

void 
LteRlUePhy::SetTxMode1Gain (double gain)
{
  SetTxModeGain (1, gain);
}

void 
LteRlUePhy::SetTxMode2Gain (double gain)
{
  SetTxModeGain (2, gain);
}

void 
LteRlUePhy::SetTxMode3Gain (double gain)
{
  SetTxModeGain (3, gain);
}

void 
LteRlUePhy::SetTxMode4Gain (double gain)
{
  SetTxModeGain (4, gain);
}

void 
LteRlUePhy::SetTxMode5Gain (double gain)
{
  SetTxModeGain (5, gain);
}

void 
LteRlUePhy::SetTxMode6Gain (double gain)
{
  SetTxModeGain (6, gain);
}

void 
LteRlUePhy::SetTxMode7Gain (double gain)
{
  SetTxModeGain (7, gain);
}


void
LteRlUePhy::SetTxModeGain (uint8_t txMode, double gain)
{
  NS_LOG_FUNCTION (this << gain);
  // convert to linear
  double gainLin = std::pow (10.0, (gain / 10.0));
  if (m_txModeGain.size () < txMode)
    {
      m_txModeGain.resize (txMode);
    }
  std::vector <double> temp;
  temp = m_txModeGain;
  m_txModeGain.clear ();
  for (uint8_t i = 0; i < temp.size (); i++)
    {
      if (i==txMode-1)
        {
          m_txModeGain.push_back (gainLin);
        }
      else
        {
          m_txModeGain.push_back (temp.at (i));
        }
    }
  // forward the info to DL LteSpectrumPhy
  m_downlinkSpectrumPhy->SetTxModeGain (txMode, gain);
}



void
LteRlUePhy::ReceiveLteDlHarqFeedback (DlInfoListElement_s m)
{
  NS_LOG_FUNCTION (this);
  // generate feedback to eNB and send it through ideal PUCCH
  Ptr<DlHarqFeedbackLteControlMessage> msg = Create<DlHarqFeedbackLteControlMessage> ();
  msg->SetDlHarqFeedback (m);
  SetControlMessages (msg);
}

void
LteRlUePhy::SetHarqPhyModule (Ptr<LteHarqPhy> harq)
{
  m_harqPhyModule = harq;
}


LteRlUePhy::State
LteRlUePhy::GetState () const
{
  NS_LOG_FUNCTION (this);
  return m_state;
}


void
LteRlUePhy::SwitchToState (State newState)
{
  NS_LOG_FUNCTION (this << newState);
  State oldState = m_state;
  m_state = newState;
  NS_LOG_INFO (this << " cellId=" << m_cellId << " rnti=" << m_rnti
                    << " UePhy " << ToString (oldState)
                    << " --> " << ToString (newState));
  m_stateTransitionTrace (m_cellId, m_rnti, oldState, newState);
}


} // namespace ns3
