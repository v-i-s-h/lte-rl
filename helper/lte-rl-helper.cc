/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2011 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
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
 * Author: Nicola Baldo <nbaldo@cttc.es> (re-wrote from scratch this helper)
 *         Giuseppe Piro <g.piro@poliba.it> (parts of the PHY & channel  creation & configuration copied from the GSoC 2011 code)
 */


#include "lte-rl-helper.h"
#include <ns3/string.h>
#include <ns3/log.h>
#include <ns3/abort.h>
#include <ns3/pointer.h>
#include <ns3/lte-enb-rrc.h>
#include <ns3/epc-ue-nas.h>
#include <ns3/epc-enb-application.h>
#include <ns3/lte-ue-rrc.h>
#include <ns3/lte-ue-mac.h>
#include <ns3/lte-enb-mac.h>
#include <ns3/lte-enb-net-device.h>
#include <ns3/lte-enb-phy.h>
#include <ns3/lte-rl-ue-phy.h>
#include <ns3/lte-rl-spectrum-phy.h>
#include <ns3/lte-chunk-processor.h>
#include <ns3/multi-model-spectrum-channel.h>
#include <ns3/friis-spectrum-propagation-loss.h>
#include <ns3/trace-fading-loss-model.h>
#include <ns3/isotropic-antenna-model.h>
#include <ns3/lte-enb-net-device.h>
#include <ns3/lte-rl-ue-net-device.h>
#include <ns3/ff-mac-scheduler.h>
#include <ns3/lte-ffr-algorithm.h>
#include <ns3/lte-handover-algorithm.h>
#include <ns3/lte-anr.h>
#include <ns3/lte-rlc.h>
#include <ns3/lte-rlc-um.h>
#include <ns3/lte-rlc-am.h>
#include <ns3/epc-enb-s1-sap.h>
#include <ns3/lte-rrc-protocol-ideal.h>
#include <ns3/lte-rrc-protocol-real.h>
#include <ns3/mac-stats-calculator.h>
#include <ns3/phy-stats-calculator.h>
#include <ns3/phy-tx-stats-calculator.h>
#include <ns3/phy-rx-stats-calculator.h>
#include <ns3/epc-helper.h>
#include <iostream>
#include <ns3/buildings-propagation-loss-model.h>
#include <ns3/lte-spectrum-value-helper.h>
#include <ns3/epc-x2.h>

#include <ns3/lte-rl-ue-net-device.h>
#include <ns3/lte-rl-enb-phy.h>
#include <ns3/lte-rl-enb-net-device.h>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("LteRlHelper");

NS_OBJECT_ENSURE_REGISTERED (LteRlHelper);

LteRlHelper::LteRlHelper (void)
  : m_fadingStreamsAssigned (false),
    m_imsiCounter (0),
    m_cellIdCounter (0)
{
  NS_LOG_FUNCTION (this);
  m_enbNetDeviceFactory.SetTypeId (LteRlEnbNetDevice::GetTypeId ());
  m_enbAntennaModelFactory.SetTypeId (IsotropicAntennaModel::GetTypeId ());
  m_ueNetDeviceFactory.SetTypeId (LteRlUeNetDevice::GetTypeId ());
  m_ueAntennaModelFactory.SetTypeId (IsotropicAntennaModel::GetTypeId ());
  m_channelFactory.SetTypeId (MultiModelSpectrumChannel::GetTypeId ());
}

void 
LteRlHelper::DoInitialize (void)
{
  NS_LOG_FUNCTION (this);
  m_downlinkChannel = m_channelFactory.Create<SpectrumChannel> ();
  m_uplinkChannel = m_channelFactory.Create<SpectrumChannel> ();

  m_downlinkPathlossModel = m_dlPathlossModelFactory.Create ();
  Ptr<SpectrumPropagationLossModel> dlSplm = m_downlinkPathlossModel->GetObject<SpectrumPropagationLossModel> ();
  if (dlSplm != 0)
    {
      NS_LOG_LOGIC (this << " using a SpectrumPropagationLossModel in DL");
      m_downlinkChannel->AddSpectrumPropagationLossModel (dlSplm);
    }
  else
    {
      NS_LOG_LOGIC (this << " using a PropagationLossModel in DL");
      Ptr<PropagationLossModel> dlPlm = m_downlinkPathlossModel->GetObject<PropagationLossModel> ();
      NS_ASSERT_MSG (dlPlm != 0, " " << m_downlinkPathlossModel << " is neither PropagationLossModel nor SpectrumPropagationLossModel");
      m_downlinkChannel->AddPropagationLossModel (dlPlm);
    }

  m_uplinkPathlossModel = m_ulPathlossModelFactory.Create ();
  Ptr<SpectrumPropagationLossModel> ulSplm = m_uplinkPathlossModel->GetObject<SpectrumPropagationLossModel> ();
  if (ulSplm != 0)
    {
      NS_LOG_LOGIC (this << " using a SpectrumPropagationLossModel in UL");
      m_uplinkChannel->AddSpectrumPropagationLossModel (ulSplm);
    }
  else
    {
      NS_LOG_LOGIC (this << " using a PropagationLossModel in UL");
      Ptr<PropagationLossModel> ulPlm = m_uplinkPathlossModel->GetObject<PropagationLossModel> ();
      NS_ASSERT_MSG (ulPlm != 0, " " << m_uplinkPathlossModel << " is neither PropagationLossModel nor SpectrumPropagationLossModel");
      m_uplinkChannel->AddPropagationLossModel (ulPlm);
    }
  if (!m_fadingModelType.empty ())
    {
      m_fadingModule = m_fadingModelFactory.Create<SpectrumPropagationLossModel> ();
      m_fadingModule->Initialize ();
      m_downlinkChannel->AddSpectrumPropagationLossModel (m_fadingModule);
      m_uplinkChannel->AddSpectrumPropagationLossModel (m_fadingModule);
    }
  m_phyStats = CreateObject<PhyStatsCalculator> ();
  m_phyTxStats = CreateObject<PhyTxStatsCalculator> ();
  m_phyRxStats = CreateObject<PhyRxStatsCalculator> ();
  m_macStats = CreateObject<MacStatsCalculator> ();
  Object::DoInitialize ();

}

LteRlHelper::~LteRlHelper (void)
{
  NS_LOG_FUNCTION (this);
}

TypeId LteRlHelper::GetTypeId (void)
{
  static TypeId
    tid =
    TypeId ("ns3::LteRlHelper")
    .SetParent<Object> ()
    .AddConstructor<LteRlHelper> ()
    .AddAttribute ("Scheduler",
                   "The type of scheduler to be used for eNBs. "
                   "The allowed values for this attributes are the type names "
                   "of any class inheriting from ns3::FfMacScheduler.",
                   StringValue ("ns3::PfFfMacScheduler"),
                   MakeStringAccessor (&LteRlHelper::SetSchedulerType,
                                       &LteRlHelper::GetSchedulerType),
                   MakeStringChecker ())
    .AddAttribute ("FfrAlgorithm",
                   "The type of FFR algorithm to be used for eNBs. "
                   "The allowed values for this attributes are the type names "
                   "of any class inheriting from ns3::LteFfrAlgorithm.",
                   StringValue ("ns3::LteFrNoOpAlgorithm"),
                   MakeStringAccessor (&LteRlHelper::SetFfrAlgorithmType,
                                       &LteRlHelper::GetFfrAlgorithmType),
                   MakeStringChecker ())
    .AddAttribute ("HandoverAlgorithm",
                   "The type of handover algorithm to be used for eNBs. "
                   "The allowed values for this attributes are the type names "
                   "of any class inheriting from ns3::LteHandoverAlgorithm.",
                   StringValue ("ns3::NoOpHandoverAlgorithm"),
                   MakeStringAccessor (&LteRlHelper::SetHandoverAlgorithmType,
                                       &LteRlHelper::GetHandoverAlgorithmType),
                   MakeStringChecker ())
    .AddAttribute ("PathlossModel",
                   "The type of pathloss model to be used. "
                   "The allowed values for this attributes are the type names "
                   "of any class inheriting from ns3::PropagationLossModel.",
                   StringValue ("ns3::FriisPropagationLossModel"),
                   MakeStringAccessor (&LteRlHelper::SetPathlossModelType),
                   MakeStringChecker ())
    .AddAttribute ("FadingModel",
                   "The type of fading model to be used."
                   "The allowed values for this attributes are the type names "
                   "of any class inheriting from ns3::SpectrumPropagationLossModel."
                   "If the type is set to an empty string, no fading model is used.",
                   StringValue (""),
                   MakeStringAccessor (&LteRlHelper::SetFadingModel),
                   MakeStringChecker ())
    .AddAttribute ("UseIdealRrc",
                   "If true, LteRrcProtocolIdeal will be used for RRC signaling. "
                   "If false, LteRrcProtocolReal will be used.",
                   BooleanValue (true), 
                   MakeBooleanAccessor (&LteRlHelper::m_useIdealRrc),
                   MakeBooleanChecker ())
    .AddAttribute ("AnrEnabled",
                   "Activate or deactivate Automatic Neighbour Relation function",
                   BooleanValue (true),
                   MakeBooleanAccessor (&LteRlHelper::m_isAnrEnabled),
                   MakeBooleanChecker ())
    .AddAttribute ("UsePdschForCqiGeneration",
                   "If true, DL-CQI will be calculated from PDCCH as signal and PDSCH as interference "
                   "If false, DL-CQI will be calculated from PDCCH as signal and PDCCH as interference  ",
                   BooleanValue (true),
                   MakeBooleanAccessor (&LteRlHelper::m_usePdschForCqiGeneration),
                   MakeBooleanChecker ())
  ;
  return tid;
}

void
LteRlHelper::DoDispose ()
{
  NS_LOG_FUNCTION (this);
  m_downlinkChannel = 0;
  m_uplinkChannel = 0;
  Object::DoDispose ();
}


void 
LteRlHelper::SetEpcHelper (Ptr<EpcHelper> h)
{
  NS_LOG_FUNCTION (this << h);
  m_epcHelper = h;
}

void 
LteRlHelper::SetSchedulerType (std::string type) 
{
  NS_LOG_FUNCTION (this << type);
  m_schedulerFactory = ObjectFactory ();
  m_schedulerFactory.SetTypeId (type);
}

std::string
LteRlHelper::GetSchedulerType () const
{
  return m_schedulerFactory.GetTypeId ().GetName ();
} 

void 
LteRlHelper::SetSchedulerAttribute (std::string n, const AttributeValue &v)
{
  NS_LOG_FUNCTION (this << n);
  m_schedulerFactory.Set (n, v);
}

std::string
LteRlHelper::GetFfrAlgorithmType () const
{
  return m_ffrAlgorithmFactory.GetTypeId ().GetName ();
}

void
LteRlHelper::SetFfrAlgorithmType (std::string type)
{
  NS_LOG_FUNCTION (this << type);
  m_ffrAlgorithmFactory = ObjectFactory ();
  m_ffrAlgorithmFactory.SetTypeId (type);
}

void
LteRlHelper::SetFfrAlgorithmAttribute (std::string n, const AttributeValue &v)
{
  NS_LOG_FUNCTION (this << n);
  m_ffrAlgorithmFactory.Set (n, v);
}

std::string
LteRlHelper::GetHandoverAlgorithmType () const
{
  return m_handoverAlgorithmFactory.GetTypeId ().GetName ();
}

void
LteRlHelper::SetHandoverAlgorithmType (std::string type)
{
  NS_LOG_FUNCTION (this << type);
  m_handoverAlgorithmFactory = ObjectFactory ();
  m_handoverAlgorithmFactory.SetTypeId (type);
}

void
LteRlHelper::SetHandoverAlgorithmAttribute (std::string n, const AttributeValue &v)
{
  NS_LOG_FUNCTION (this << n);
  m_handoverAlgorithmFactory.Set (n, v);
}


void 
LteRlHelper::SetPathlossModelType (std::string type) 
{
  NS_LOG_FUNCTION (this << type);
  m_dlPathlossModelFactory = ObjectFactory ();
  m_dlPathlossModelFactory.SetTypeId (type);
  m_ulPathlossModelFactory = ObjectFactory ();
  m_ulPathlossModelFactory.SetTypeId (type);
}

void 
LteRlHelper::SetPathlossModelAttribute (std::string n, const AttributeValue &v)
{
  NS_LOG_FUNCTION (this << n);
  m_dlPathlossModelFactory.Set (n, v);
  m_ulPathlossModelFactory.Set (n, v);
}

void
LteRlHelper::SetEnbDeviceAttribute (std::string n, const AttributeValue &v)
{
  NS_LOG_FUNCTION (this);
  m_enbNetDeviceFactory.Set (n, v);
}


void 
LteRlHelper::SetEnbAntennaModelType (std::string type)
{
  NS_LOG_FUNCTION (this);
  m_enbAntennaModelFactory.SetTypeId (type);
}

void 
LteRlHelper::SetEnbAntennaModelAttribute (std::string n, const AttributeValue &v)
{
  NS_LOG_FUNCTION (this);
  m_enbAntennaModelFactory.Set (n, v);
}

void
LteRlHelper::SetUeDeviceAttribute (std::string n, const AttributeValue &v)
{
  NS_LOG_FUNCTION (this);
  m_ueNetDeviceFactory.Set (n, v);
}

void 
LteRlHelper::SetUeAntennaModelType (std::string type)
{
  NS_LOG_FUNCTION (this);
  m_ueAntennaModelFactory.SetTypeId (type);
}

void 
LteRlHelper::SetUeAntennaModelAttribute (std::string n, const AttributeValue &v)
{
  NS_LOG_FUNCTION (this);
  m_ueAntennaModelFactory.Set (n, v);
}

void 
LteRlHelper::SetFadingModel (std::string type) 
{
  NS_LOG_FUNCTION (this << type);
  m_fadingModelType = type;
  if (!type.empty ())
    {
      m_fadingModelFactory = ObjectFactory ();
      m_fadingModelFactory.SetTypeId (type);
    }
}

void 
LteRlHelper::SetFadingModelAttribute (std::string n, const AttributeValue &v)
{
  m_fadingModelFactory.Set (n, v);
}

void 
LteRlHelper::SetSpectrumChannelType (std::string type) 
{
  NS_LOG_FUNCTION (this << type);
  m_channelFactory.SetTypeId (type);
}

void 
LteRlHelper::SetSpectrumChannelAttribute (std::string n, const AttributeValue &v)
{
  m_channelFactory.Set (n, v);
}


NetDeviceContainer
LteRlHelper::InstallEnbDevice (NodeContainer c)
{
  NS_LOG_FUNCTION (this);
  Initialize ();  // will run DoInitialize () if necessary
  NetDeviceContainer devices;
  for (NodeContainer::Iterator i = c.Begin (); i != c.End (); ++i)
    {
      Ptr<Node> node = *i;
      Ptr<NetDevice> device = InstallSingleEnbDevice (node);
      devices.Add (device);
    }
  return devices;
}

NetDeviceContainer
LteRlHelper::InstallUeDevice (NodeContainer c)
{
  NS_LOG_FUNCTION (this);
  NetDeviceContainer devices;
  for (NodeContainer::Iterator i = c.Begin (); i != c.End (); ++i)
    {
      Ptr<Node> node = *i;
      Ptr<NetDevice> device = InstallSingleUeDevice (node);
      devices.Add (device);
    }
  return devices;
}


Ptr<NetDevice>
LteRlHelper::InstallSingleEnbDevice (Ptr<Node> n)
{

  NS_ABORT_MSG_IF (m_cellIdCounter == 65535, "max num eNBs exceeded");
  uint16_t cellId = ++m_cellIdCounter;

  Ptr<LteRlSpectrumPhy> dlPhy = CreateObject<LteRlSpectrumPhy> ();
  Ptr<LteRlSpectrumPhy> ulPhy = CreateObject<LteRlSpectrumPhy> ();

  Ptr<LteRlEnbPhy> phy = CreateObject<LteRlEnbPhy> (dlPhy, ulPhy);

  Ptr<LteHarqPhy> harq = Create<LteHarqPhy> ();
  dlPhy->SetHarqPhyModule (harq);
  ulPhy->SetHarqPhyModule (harq);
  phy->SetHarqPhyModule (harq);

  Ptr<LteChunkProcessor> pCtrl = Create<LteChunkProcessor> ();
  pCtrl->AddCallback (MakeCallback (&LteRlEnbPhy::GenerateCtrlCqiReport, phy));
  ulPhy->AddCtrlSinrChunkProcessor (pCtrl); // for evaluating SRS UL-CQI

  Ptr<LteChunkProcessor> pData = Create<LteChunkProcessor> ();
  pData->AddCallback (MakeCallback (&LteRlEnbPhy::GenerateDataCqiReport, phy));
  pData->AddCallback (MakeCallback (&LteRlSpectrumPhy::UpdateSinrPerceived, ulPhy));
  ulPhy->AddDataSinrChunkProcessor (pData); // for evaluating PUSCH UL-CQI

  Ptr<LteChunkProcessor> pInterf = Create<LteChunkProcessor> ();
  pInterf->AddCallback (MakeCallback (&LteRlEnbPhy::ReportInterference, phy));
  ulPhy->AddInterferenceDataChunkProcessor (pInterf); // for interference power tracing

  dlPhy->SetChannel (m_downlinkChannel);
  ulPhy->SetChannel (m_uplinkChannel);

  Ptr<MobilityModel> mm = n->GetObject<MobilityModel> ();
  NS_ASSERT_MSG (mm, "MobilityModel needs to be set on node before calling LteRlHelper::InstallUeDevice ()");
  dlPhy->SetMobility (mm);
  ulPhy->SetMobility (mm);

  Ptr<AntennaModel> antenna = (m_enbAntennaModelFactory.Create ())->GetObject<AntennaModel> ();
  NS_ASSERT_MSG (antenna, "error in creating the AntennaModel object");
  dlPhy->SetAntenna (antenna);
  ulPhy->SetAntenna (antenna);

  Ptr<LteEnbMac> mac = CreateObject<LteEnbMac> ();
  Ptr<FfMacScheduler> sched = m_schedulerFactory.Create<FfMacScheduler> ();
  Ptr<LteFfrAlgorithm> ffrAlgorithm = m_ffrAlgorithmFactory.Create<LteFfrAlgorithm> ();
  Ptr<LteHandoverAlgorithm> handoverAlgorithm = m_handoverAlgorithmFactory.Create<LteHandoverAlgorithm> ();
  Ptr<LteEnbRrc> rrc = CreateObject<LteEnbRrc> ();

  if (m_useIdealRrc)
    {
      Ptr<LteEnbRrcProtocolIdeal> rrcProtocol = CreateObject<LteEnbRrcProtocolIdeal> ();
      rrcProtocol->SetLteEnbRrcSapProvider (rrc->GetLteEnbRrcSapProvider ());
      rrc->SetLteEnbRrcSapUser (rrcProtocol->GetLteEnbRrcSapUser ());
      rrc->AggregateObject (rrcProtocol);
      rrcProtocol->SetCellId (cellId);
    }
  else
    {
      Ptr<LteEnbRrcProtocolReal> rrcProtocol = CreateObject<LteEnbRrcProtocolReal> ();
      rrcProtocol->SetLteEnbRrcSapProvider (rrc->GetLteEnbRrcSapProvider ());
      rrc->SetLteEnbRrcSapUser (rrcProtocol->GetLteEnbRrcSapUser ());
      rrc->AggregateObject (rrcProtocol);
      rrcProtocol->SetCellId (cellId);
    }

  if (m_epcHelper != 0)
    {
      EnumValue epsBearerToRlcMapping;
      rrc->GetAttribute ("EpsBearerToRlcMapping", epsBearerToRlcMapping);
      // it does not make sense to use RLC/SM when also using the EPC
      if (epsBearerToRlcMapping.Get () == LteEnbRrc::RLC_SM_ALWAYS)
        {
          rrc->SetAttribute ("EpsBearerToRlcMapping", EnumValue (LteEnbRrc::RLC_UM_ALWAYS));
        }
    }

  rrc->SetLteEnbCmacSapProvider (mac->GetLteEnbCmacSapProvider ());
  mac->SetLteEnbCmacSapUser (rrc->GetLteEnbCmacSapUser ());
  rrc->SetLteMacSapProvider (mac->GetLteMacSapProvider ());

  rrc->SetLteHandoverManagementSapProvider (handoverAlgorithm->GetLteHandoverManagementSapProvider ());
  handoverAlgorithm->SetLteHandoverManagementSapUser (rrc->GetLteHandoverManagementSapUser ());

  mac->SetFfMacSchedSapProvider (sched->GetFfMacSchedSapProvider ());
  mac->SetFfMacCschedSapProvider (sched->GetFfMacCschedSapProvider ());

  sched->SetFfMacSchedSapUser (mac->GetFfMacSchedSapUser ());
  sched->SetFfMacCschedSapUser (mac->GetFfMacCschedSapUser ());

  phy->SetLteEnbPhySapUser (mac->GetLteEnbPhySapUser ());
  mac->SetLteEnbPhySapProvider (phy->GetLteEnbPhySapProvider ());

  phy->SetLteEnbCphySapUser (rrc->GetLteEnbCphySapUser ());
  rrc->SetLteEnbCphySapProvider (phy->GetLteEnbCphySapProvider ());

  //FFR SAP
  sched->SetLteFfrSapProvider (ffrAlgorithm->GetLteFfrSapProvider ());
  ffrAlgorithm->SetLteFfrSapUser (sched->GetLteFfrSapUser ());

  rrc->SetLteFfrRrcSapProvider (ffrAlgorithm->GetLteFfrRrcSapProvider ());
  ffrAlgorithm->SetLteFfrRrcSapUser (rrc->GetLteFfrRrcSapUser ());
  //FFR SAP END

  Ptr<LteRlEnbNetDevice> dev = m_enbNetDeviceFactory.Create<LteRlEnbNetDevice> ();
  dev->SetNode (n);
  dev->SetAttribute ("CellId", UintegerValue (cellId)); 
  dev->SetAttribute ("LteEnbPhy", PointerValue (phy));
  dev->SetAttribute ("LteEnbMac", PointerValue (mac));
  dev->SetAttribute ("FfMacScheduler", PointerValue (sched));
  dev->SetAttribute ("LteEnbRrc", PointerValue (rrc)); 
  dev->SetAttribute ("LteHandoverAlgorithm", PointerValue (handoverAlgorithm));
  dev->SetAttribute ("LteFfrAlgorithm", PointerValue (ffrAlgorithm));

  if (m_isAnrEnabled)
    {
      Ptr<LteAnr> anr = CreateObject<LteAnr> (cellId);
      rrc->SetLteAnrSapProvider (anr->GetLteAnrSapProvider ());
      anr->SetLteAnrSapUser (rrc->GetLteAnrSapUser ());
      dev->SetAttribute ("LteAnr", PointerValue (anr));
    }

  phy->SetDevice (dev);
  dlPhy->SetDevice (dev);
  ulPhy->SetDevice (dev);

  n->AddDevice (dev);
  ulPhy->SetLtePhyRxDataEndOkCallback (MakeCallback (&LteRlEnbPhy::PhyPduReceived, phy));
  ulPhy->SetLtePhyRxCtrlEndOkCallback (MakeCallback (&LteRlEnbPhy::ReceiveLteControlMessageList, phy));
  ulPhy->SetLtePhyUlHarqFeedbackCallback (MakeCallback (&LteRlEnbPhy::ReceiveLteUlHarqFeedback, phy));
  rrc->SetForwardUpCallback (MakeCallback (&LteRlEnbNetDevice::Receive, dev));

  NS_LOG_LOGIC ("set the propagation model frequencies");
  double dlFreq = LteSpectrumValueHelper::GetCarrierFrequency (dev->GetDlEarfcn ());
  NS_LOG_LOGIC ("DL freq: " << dlFreq);
  bool dlFreqOk = m_downlinkPathlossModel->SetAttributeFailSafe ("Frequency", DoubleValue (dlFreq));
  if (!dlFreqOk)
    {
      NS_LOG_WARN ("DL propagation model does not have a Frequency attribute");
    }
  double ulFreq = LteSpectrumValueHelper::GetCarrierFrequency (dev->GetUlEarfcn ());
  NS_LOG_LOGIC ("UL freq: " << ulFreq);
  bool ulFreqOk = m_uplinkPathlossModel->SetAttributeFailSafe ("Frequency", DoubleValue (ulFreq));
  if (!ulFreqOk)
    {
      NS_LOG_WARN ("UL propagation model does not have a Frequency attribute");
    }

  dev->Initialize ();

  m_uplinkChannel->AddRx (ulPhy);

  if (m_epcHelper != 0)
    {
      NS_LOG_INFO ("adding this eNB to the EPC");
      m_epcHelper->AddEnb (n, dev, dev->GetCellId ());
      Ptr<EpcEnbApplication> enbApp = n->GetApplication (0)->GetObject<EpcEnbApplication> ();
      NS_ASSERT_MSG (enbApp != 0, "cannot retrieve EpcEnbApplication");

      // S1 SAPs
      rrc->SetS1SapProvider (enbApp->GetS1SapProvider ());
      enbApp->SetS1SapUser (rrc->GetS1SapUser ());

      // X2 SAPs
      Ptr<EpcX2> x2 = n->GetObject<EpcX2> ();
      x2->SetEpcX2SapUser (rrc->GetEpcX2SapUser ());
      rrc->SetEpcX2SapProvider (x2->GetEpcX2SapProvider ());
    }

  return dev;
}

Ptr<NetDevice>
LteRlHelper::InstallSingleUeDevice (Ptr<Node> n)
{
  NS_LOG_FUNCTION (this);
  Ptr<LteRlSpectrumPhy> dlPhy = CreateObject<LteRlSpectrumPhy> ();
  Ptr<LteRlSpectrumPhy> ulPhy = CreateObject<LteRlSpectrumPhy> ();

  Ptr<LteRlUePhy> phy = CreateObject<LteRlUePhy> (dlPhy, ulPhy);

  Ptr<LteHarqPhy> harq = Create<LteHarqPhy> ();
  dlPhy->SetHarqPhyModule (harq);
  ulPhy->SetHarqPhyModule (harq);
  phy->SetHarqPhyModule (harq);

  Ptr<LteChunkProcessor> pRs = Create<LteChunkProcessor> ();
  pRs->AddCallback (MakeCallback (&LteRlUePhy::ReportRsReceivedPower, phy));
  dlPhy->AddRsPowerChunkProcessor (pRs);

  Ptr<LteChunkProcessor> pInterf = Create<LteChunkProcessor> ();
  pInterf->AddCallback (MakeCallback (&LteRlUePhy::ReportInterference, phy));
  dlPhy->AddInterferenceCtrlChunkProcessor (pInterf); // for RSRQ evaluation of UE Measurements

  Ptr<LteChunkProcessor> pCtrl = Create<LteChunkProcessor> ();
  pCtrl->AddCallback (MakeCallback (&LteRlSpectrumPhy::UpdateSinrPerceived, dlPhy));
  dlPhy->AddCtrlSinrChunkProcessor (pCtrl);

  Ptr<LteChunkProcessor> pData = Create<LteChunkProcessor> ();
  pData->AddCallback (MakeCallback (&LteRlSpectrumPhy::UpdateSinrPerceived, dlPhy));
  dlPhy->AddDataSinrChunkProcessor (pData);

  if (m_usePdschForCqiGeneration)
    {
      // CQI calculation based on PDCCH for signal and PDSCH for interference
      pCtrl->AddCallback (MakeCallback (&LteRlUePhy::GenerateMixedCqiReport, phy));
      Ptr<LteChunkProcessor> pDataInterf = Create<LteChunkProcessor> ();      
      pDataInterf->AddCallback (MakeCallback (&LteRlUePhy::ReportDataInterference, phy));
      dlPhy->AddInterferenceDataChunkProcessor (pDataInterf);
    }
  else
    {
      // CQI calculation based on PDCCH for both signal and interference
      pCtrl->AddCallback (MakeCallback (&LteRlUePhy::GenerateCtrlCqiReport, phy));
    }



  dlPhy->SetChannel (m_downlinkChannel);
  ulPhy->SetChannel (m_uplinkChannel);

  Ptr<MobilityModel> mm = n->GetObject<MobilityModel> ();
  NS_ASSERT_MSG (mm, "MobilityModel needs to be set on node before calling LteRlHelper::InstallUeDevice ()");
  dlPhy->SetMobility (mm);
  ulPhy->SetMobility (mm);

  Ptr<AntennaModel> antenna = (m_ueAntennaModelFactory.Create ())->GetObject<AntennaModel> ();
  NS_ASSERT_MSG (antenna, "error in creating the AntennaModel object");
  dlPhy->SetAntenna (antenna);
  ulPhy->SetAntenna (antenna);

  Ptr<LteUeMac> mac = CreateObject<LteUeMac> ();
  Ptr<LteUeRrc> rrc = CreateObject<LteUeRrc> ();

  if (m_useIdealRrc)
    {
      Ptr<LteUeRrcProtocolIdeal> rrcProtocol = CreateObject<LteUeRrcProtocolIdeal> ();
      rrcProtocol->SetUeRrc (rrc);
      rrc->AggregateObject (rrcProtocol);
      rrcProtocol->SetLteUeRrcSapProvider (rrc->GetLteUeRrcSapProvider ());
      rrc->SetLteUeRrcSapUser (rrcProtocol->GetLteUeRrcSapUser ());
    }
  else
    {
      Ptr<LteUeRrcProtocolReal> rrcProtocol = CreateObject<LteUeRrcProtocolReal> ();
      rrcProtocol->SetUeRrc (rrc);
      rrc->AggregateObject (rrcProtocol);
      rrcProtocol->SetLteUeRrcSapProvider (rrc->GetLteUeRrcSapProvider ());
      rrc->SetLteUeRrcSapUser (rrcProtocol->GetLteUeRrcSapUser ());
    }

  if (m_epcHelper != 0)
    {
      rrc->SetUseRlcSm (false);
    }
  Ptr<EpcUeNas> nas = CreateObject<EpcUeNas> ();
 
  nas->SetAsSapProvider (rrc->GetAsSapProvider ());
  rrc->SetAsSapUser (nas->GetAsSapUser ());

  rrc->SetLteUeCmacSapProvider (mac->GetLteUeCmacSapProvider ());
  mac->SetLteUeCmacSapUser (rrc->GetLteUeCmacSapUser ());
  rrc->SetLteMacSapProvider (mac->GetLteMacSapProvider ());

  phy->SetLteUePhySapUser (mac->GetLteUePhySapUser ());
  mac->SetLteUePhySapProvider (phy->GetLteUePhySapProvider ());

  phy->SetLteUeCphySapUser (rrc->GetLteUeCphySapUser ());
  rrc->SetLteUeCphySapProvider (phy->GetLteUeCphySapProvider ());

  NS_ABORT_MSG_IF (m_imsiCounter >= 0xFFFFFFFF, "max num UEs exceeded");
  uint64_t imsi = ++m_imsiCounter;

  Ptr<LteRlUeNetDevice> dev = m_ueNetDeviceFactory.Create<LteRlUeNetDevice> ();
  dev->SetNode (n);
  dev->SetAttribute ("Imsi", UintegerValue (imsi));
  dev->SetAttribute ("LteUePhy", PointerValue (phy));
  dev->SetAttribute ("LteUeMac", PointerValue (mac));
  dev->SetAttribute ("LteUeRrc", PointerValue (rrc));
  dev->SetAttribute ("EpcUeNas", PointerValue (nas));

  phy->SetDevice (dev);
  dlPhy->SetDevice (dev);
  ulPhy->SetDevice (dev);
  nas->SetDevice (dev);

  n->AddDevice (dev);
  dlPhy->SetLtePhyRxDataEndOkCallback (MakeCallback (&LteRlUePhy::PhyPduReceived, phy));
  dlPhy->SetLtePhyRxCtrlEndOkCallback (MakeCallback (&LteRlUePhy::ReceiveLteControlMessageList, phy));
  dlPhy->SetLtePhyRxPssCallback (MakeCallback (&LteRlUePhy::ReceivePss, phy));
  dlPhy->SetLtePhyDlHarqFeedbackCallback (MakeCallback (&LteRlUePhy::ReceiveLteDlHarqFeedback, phy));
  nas->SetForwardUpCallback (MakeCallback (&LteRlUeNetDevice::Receive, dev));

  if (m_epcHelper != 0)
    {
      m_epcHelper->AddUe (dev, dev->GetImsi ());
    }

  dev->Initialize ();

  return dev;
}


void
LteRlHelper::Attach (NetDeviceContainer ueDevices)
{
  NS_LOG_FUNCTION (this);
  for (NetDeviceContainer::Iterator i = ueDevices.Begin (); i != ueDevices.End (); ++i)
    {
      Attach (*i);
    }
}

void
LteRlHelper::Attach (Ptr<NetDevice> ueDevice)
{
  NS_LOG_FUNCTION (this);

  if (m_epcHelper == 0)
    {
      NS_FATAL_ERROR ("This function is not valid without properly configured EPC");
    }

  Ptr<LteRlUeNetDevice> ueLteDevice = ueDevice->GetObject<LteRlUeNetDevice> ();
  if (ueLteDevice == 0)
    {
      NS_FATAL_ERROR ("The passed NetDevice must be an LteRlUeNetDevice");
    }

  // initiate cell selection
  Ptr<EpcUeNas> ueNas = ueLteDevice->GetNas ();
  NS_ASSERT (ueNas != 0);
  uint16_t dlEarfcn = ueLteDevice->GetDlEarfcn ();
  ueNas->StartCellSelection (dlEarfcn);

  // instruct UE to immediately enter CONNECTED mode after camping
  ueNas->Connect ();

  // activate default EPS bearer
  m_epcHelper->ActivateEpsBearer (ueDevice, ueLteDevice->GetImsi (),
                                  EpcTft::Default (),
                                  EpsBearer (EpsBearer::NGBR_VIDEO_TCP_DEFAULT));
}

void
LteRlHelper::Attach (NetDeviceContainer ueDevices, Ptr<NetDevice> enbDevice)
{
  NS_LOG_FUNCTION (this);
  for (NetDeviceContainer::Iterator i = ueDevices.Begin (); i != ueDevices.End (); ++i)
    {
      Attach (*i, enbDevice);
    }
}

void
LteRlHelper::Attach (Ptr<NetDevice> ueDevice, Ptr<NetDevice> enbDevice)
{
  NS_LOG_FUNCTION (this);
  //enbRrc->SetCellId (enbDevice->GetObject<LteRlEnbNetDevice> ()->GetCellId ());

  Ptr<LteRlUeNetDevice> ueLteDevice = ueDevice->GetObject<LteRlUeNetDevice> ();
  Ptr<LteRlEnbNetDevice> enbLteDevice = enbDevice->GetObject<LteRlEnbNetDevice> ();

  Ptr<EpcUeNas> ueNas = ueLteDevice->GetNas ();
  ueNas->Connect (enbLteDevice->GetCellId (), enbLteDevice->GetDlEarfcn ());

  if (m_epcHelper != 0)
    {
      // activate default EPS bearer
      m_epcHelper->ActivateEpsBearer (ueDevice, ueLteDevice->GetImsi (), EpcTft::Default (), EpsBearer (EpsBearer::NGBR_VIDEO_TCP_DEFAULT));
    }

  // tricks needed for the simplified LTE-only simulations 
  if (m_epcHelper == 0)
    {
      ueDevice->GetObject<LteRlUeNetDevice> ()->SetTargetEnb (enbDevice->GetObject<LteRlEnbNetDevice> ());
    }
}

void
LteRlHelper::AttachToClosestEnb (NetDeviceContainer ueDevices, NetDeviceContainer enbDevices)
{
  NS_LOG_FUNCTION (this);
  for (NetDeviceContainer::Iterator i = ueDevices.Begin (); i != ueDevices.End (); ++i)
    {
      AttachToClosestEnb (*i, enbDevices);
    }
}

void
LteRlHelper::AttachToClosestEnb (Ptr<NetDevice> ueDevice, NetDeviceContainer enbDevices)
{
  NS_LOG_FUNCTION (this);
  NS_ASSERT_MSG (enbDevices.GetN () > 0, "empty enb device container");
  Vector uepos = ueDevice->GetNode ()->GetObject<MobilityModel> ()->GetPosition ();
  double minDistance = std::numeric_limits<double>::infinity ();
  Ptr<NetDevice> closestEnbDevice;
  for (NetDeviceContainer::Iterator i = enbDevices.Begin (); i != enbDevices.End (); ++i)
    {
      Vector enbpos = (*i)->GetNode ()->GetObject<MobilityModel> ()->GetPosition ();
      double distance = CalculateDistance (uepos, enbpos);
      if (distance < minDistance)
        {
          minDistance = distance;
          closestEnbDevice = *i;
        }
    }
  NS_ASSERT (closestEnbDevice != 0);
  Attach (ueDevice, closestEnbDevice);
}

uint8_t
LteRlHelper::ActivateDedicatedEpsBearer (NetDeviceContainer ueDevices, EpsBearer bearer, Ptr<EpcTft> tft)
{
  NS_LOG_FUNCTION (this);
  for (NetDeviceContainer::Iterator i = ueDevices.Begin (); i != ueDevices.End (); ++i)
    {
      uint8_t bearerId = ActivateDedicatedEpsBearer (*i, bearer, tft);
      return bearerId;
    }
  return 0;
}


uint8_t
LteRlHelper::ActivateDedicatedEpsBearer (Ptr<NetDevice> ueDevice, EpsBearer bearer, Ptr<EpcTft> tft)
{
  NS_LOG_FUNCTION (this);

  NS_ASSERT_MSG (m_epcHelper != 0, "dedicated EPS bearers cannot be set up when the EPC is not used");

  uint64_t imsi = ueDevice->GetObject<LteRlUeNetDevice> ()->GetImsi ();
  uint8_t bearerId = m_epcHelper->ActivateEpsBearer (ueDevice, imsi, tft, bearer);
  return bearerId;
}

/**
 * \ingroup lte
 *
 * DrbActivatior allows user to activate bearers for UEs
 * when EPC is not used. Activation function is hooked to
 * the Enb RRC Connection Estabilished trace source. When
 * UE change its RRC state to CONNECTED_NORMALLY, activation
 * function is called and bearer is activated.
*/
class DrbActivatorRL : public SimpleRefCount<DrbActivatorRL>
{
public:
  /**
  * DrbActivatorRL Constructor
  *
  * \param ueDevice the UeNetDevice for which bearer will be activated
  * \param bearer the bearer configuration
  */
  DrbActivatorRL (Ptr<NetDevice> ueDevice, EpsBearer bearer);

  /**
   * Function hooked to the Enb RRC Connection Established trace source
   * Fired upon successful RRC connection establishment.
   *
   * \param a DrbActivatorRL object
   * \param context
   * \param imsi
   * \param cellId
   * \param rnti
   */
  static void ActivateCallback (Ptr<DrbActivatorRL> a, std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti);

  /**
   * Procedure firstly checks if bearer was not activated, if IMSI
   * from trace source equals configured one and if UE is really
   * in RRC connected state. If all requirements are met, it performs
   * bearer activation.
   *
   * \param imsi
   * \param cellId
   * \param rnti
   */
  void ActivateDrb (uint64_t imsi, uint16_t cellId, uint16_t rnti);
private:
  /**
   * Bearer can be activated only once. This value stores state of
   * bearer. Initially is set to false and changed to true during
   * bearer activation.
   */
  bool m_active;
  /**
   * UeNetDevice for which bearer will be activated
   */
  Ptr<NetDevice> m_ueDevice;
  /**
   * Configuration of bearer which will be activated
   */
  EpsBearer m_bearer;
  /**
   * imsi the unique UE identifier
   */
  uint64_t m_imsi;
};

DrbActivatorRL::DrbActivatorRL (Ptr<NetDevice> ueDevice, EpsBearer bearer)
  : m_active (false),
    m_ueDevice (ueDevice),
    m_bearer (bearer),
    m_imsi (m_ueDevice->GetObject<LteRlUeNetDevice> ()->GetImsi ())
{
}

void
DrbActivatorRL::ActivateCallback (Ptr<DrbActivatorRL> a, std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
  NS_LOG_FUNCTION (a << context << imsi << cellId << rnti);
  a->ActivateDrb (imsi, cellId, rnti);
}

void
DrbActivatorRL::ActivateDrb (uint64_t imsi, uint16_t cellId, uint16_t rnti)
{ 
  NS_LOG_FUNCTION (this << imsi << cellId << rnti << m_active);
  if ((!m_active) && (imsi == m_imsi))
    {
      Ptr<LteUeRrc> ueRrc = m_ueDevice->GetObject<LteRlUeNetDevice> ()->GetRrc ();
      NS_ASSERT (ueRrc->GetState () == LteUeRrc::CONNECTED_NORMALLY);
      uint16_t rnti = ueRrc->GetRnti ();
      Ptr<LteRlEnbNetDevice> enbLteDevice = m_ueDevice->GetObject<LteRlUeNetDevice> ()->GetTargetEnb ();
      Ptr<LteEnbRrc> enbRrc = enbLteDevice->GetObject<LteRlEnbNetDevice> ()->GetRrc ();
      NS_ASSERT (ueRrc->GetCellId () == enbLteDevice->GetCellId ());
      Ptr<UeManager> ueManager = enbRrc->GetUeManager (rnti);
      NS_ASSERT (ueManager->GetState () == UeManager::CONNECTED_NORMALLY
                 || ueManager->GetState () == UeManager::CONNECTION_RECONFIGURATION);
      EpcEnbS1SapUser::DataRadioBearerSetupRequestParameters params;
      params.rnti = rnti;
      params.bearer = m_bearer;
      params.bearerId = 0;
      params.gtpTeid = 0; // don't care
      enbRrc->GetS1SapUser ()->DataRadioBearerSetupRequest (params);
      m_active = true;
    }
}


void 
LteRlHelper::ActivateDataRadioBearer (Ptr<NetDevice> ueDevice, EpsBearer bearer)
{
  NS_LOG_FUNCTION (this << ueDevice);
  NS_ASSERT_MSG (m_epcHelper == 0, "this method must not be used when the EPC is being used");

  // Normally it is the EPC that takes care of activating DRBs
  // when the UE gets connected. When the EPC is not used, we achieve
  // the same behavior by hooking a dedicated DRB activation function
  // to the Enb RRC Connection Established trace source


  Ptr<LteRlEnbNetDevice> enbLteDevice = ueDevice->GetObject<LteRlUeNetDevice> ()->GetTargetEnb ();

  std::ostringstream path;
  path << "/NodeList/" << enbLteDevice->GetNode ()->GetId () 
       << "/DeviceList/" << enbLteDevice->GetIfIndex ()
       << "/LteEnbRrc/ConnectionEstablished";
  Ptr<DrbActivatorRL> arg = Create<DrbActivatorRL> (ueDevice, bearer);
  Config::Connect (path.str (), MakeBoundCallback (&DrbActivatorRL::ActivateCallback, arg));
}

void
LteRlHelper::AddX2Interface (NodeContainer enbNodes)
{
  NS_LOG_FUNCTION (this);

  NS_ASSERT_MSG (m_epcHelper != 0, "X2 interfaces cannot be set up when the EPC is not used");

  for (NodeContainer::Iterator i = enbNodes.Begin (); i != enbNodes.End (); ++i)
    {
      for (NodeContainer::Iterator j = i + 1; j != enbNodes.End (); ++j)
        {
          AddX2Interface (*i, *j);
        }
    }
}

void
LteRlHelper::AddX2Interface (Ptr<Node> enbNode1, Ptr<Node> enbNode2)
{
  NS_LOG_FUNCTION (this);
  NS_LOG_INFO ("setting up the X2 interface");

  m_epcHelper->AddX2Interface (enbNode1, enbNode2);
}

void
LteRlHelper::HandoverRequest (Time hoTime, Ptr<NetDevice> ueDev, Ptr<NetDevice> sourceEnbDev, Ptr<NetDevice> targetEnbDev)
{
  NS_LOG_FUNCTION (this << ueDev << sourceEnbDev << targetEnbDev);
  NS_ASSERT_MSG (m_epcHelper, "Handover requires the use of the EPC - did you forget to call LteRlHelper::SetEpcHelper () ?");
  Simulator::Schedule (hoTime, &LteRlHelper::DoHandoverRequest, this, ueDev, sourceEnbDev, targetEnbDev);
}

void
LteRlHelper::DoHandoverRequest (Ptr<NetDevice> ueDev, Ptr<NetDevice> sourceEnbDev, Ptr<NetDevice> targetEnbDev)
{
  NS_LOG_FUNCTION (this << ueDev << sourceEnbDev << targetEnbDev);

  uint16_t targetCellId = targetEnbDev->GetObject<LteRlEnbNetDevice> ()->GetCellId ();
  Ptr<LteEnbRrc> sourceRrc = sourceEnbDev->GetObject<LteRlEnbNetDevice> ()->GetRrc ();
  uint16_t rnti = ueDev->GetObject<LteRlUeNetDevice> ()->GetRrc ()->GetRnti ();
  sourceRrc->SendHandoverRequest (rnti, targetCellId);
}

void
LteRlHelper::DeActivateDedicatedEpsBearer (Ptr<NetDevice> ueDevice,Ptr<NetDevice> enbDevice, uint8_t bearerId)
{
  NS_LOG_FUNCTION (this << ueDevice << bearerId);
  NS_ASSERT_MSG (m_epcHelper != 0, "Dedicated EPS bearers cannot be de-activated when the EPC is not used");
  NS_ASSERT_MSG (bearerId != 1, "Default bearer cannot be de-activated until and unless and UE is released");

  DoDeActivateDedicatedEpsBearer (ueDevice, enbDevice, bearerId);
}

void
LteRlHelper::DoDeActivateDedicatedEpsBearer (Ptr<NetDevice> ueDevice, Ptr<NetDevice> enbDevice, uint8_t bearerId)
{
  NS_LOG_FUNCTION (this << ueDevice << bearerId);

  //Extract IMSI and rnti
  uint64_t imsi = ueDevice->GetObject<LteRlUeNetDevice> ()->GetImsi ();
  uint16_t rnti = ueDevice->GetObject<LteRlUeNetDevice> ()->GetRrc ()->GetRnti ();


  Ptr<LteEnbRrc> enbRrc = enbDevice->GetObject<LteRlEnbNetDevice> ()->GetRrc ();

  enbRrc->DoSendReleaseDataRadioBearer (imsi,rnti,bearerId);
}


void 
LteRlHelper::ActivateDataRadioBearer (NetDeviceContainer ueDevices, EpsBearer bearer)
{
  NS_LOG_FUNCTION (this);
  for (NetDeviceContainer::Iterator i = ueDevices.Begin (); i != ueDevices.End (); ++i)
    {
      ActivateDataRadioBearer (*i, bearer);
    }
}

void
LteRlHelper::EnableLogComponents (void)
{
  LogComponentEnable ("LteRlHelper", LOG_LEVEL_ALL);
  LogComponentEnable ("LteEnbRrc", LOG_LEVEL_ALL);
  LogComponentEnable ("LteUeRrc", LOG_LEVEL_ALL);
  LogComponentEnable ("LteEnbMac", LOG_LEVEL_ALL);
  LogComponentEnable ("LteUeMac", LOG_LEVEL_ALL);
  LogComponentEnable ("LteRlc", LOG_LEVEL_ALL);
  LogComponentEnable ("LteRlcUm", LOG_LEVEL_ALL);
  LogComponentEnable ("LteRlcAm", LOG_LEVEL_ALL);
  LogComponentEnable ("RrFfMacScheduler", LOG_LEVEL_ALL);
  LogComponentEnable ("PfFfMacScheduler", LOG_LEVEL_ALL);

  LogComponentEnable ("LtePhy", LOG_LEVEL_ALL);
  LogComponentEnable ("LteEnbPhy", LOG_LEVEL_ALL);
  LogComponentEnable ("LteRlUePhy", LOG_LEVEL_ALL);
  LogComponentEnable ("LteSpectrumValueHelper", LOG_LEVEL_ALL);
  LogComponentEnable ("LteRlSpectrumPhy", LOG_LEVEL_ALL);
  LogComponentEnable ("LteInterference", LOG_LEVEL_ALL);
  LogComponentEnable ("LteChunkProcessor", LOG_LEVEL_ALL);

  std::string propModelStr = m_dlPathlossModelFactory.GetTypeId ().GetName ().erase (0,5).c_str ();
  LogComponentEnable ("LteRlNetDevice", LOG_LEVEL_ALL);
  LogComponentEnable ("LteRlUeNetDevice", LOG_LEVEL_ALL);
  LogComponentEnable ("LteRlEnbNetDevice", LOG_LEVEL_ALL);

  LogComponentEnable ("RadioBearerStatsCalculator", LOG_LEVEL_ALL);
  LogComponentEnable ("LteStatsCalculator", LOG_LEVEL_ALL);
  LogComponentEnable ("MacStatsCalculator", LOG_LEVEL_ALL);
  LogComponentEnable ("PhyTxStatsCalculator", LOG_LEVEL_ALL);
  LogComponentEnable ("PhyRxStatsCalculator", LOG_LEVEL_ALL);
  LogComponentEnable ("PhyStatsCalculator", LOG_LEVEL_ALL);


}

void
LteRlHelper::EnableTraces (void)
{
  EnablePhyTraces ();
  EnableMacTraces ();
  EnableRlcTraces ();
  EnablePdcpTraces ();
}

void
LteRlHelper::EnableRlcTraces (void)
{
  NS_ASSERT_MSG (m_rlcStats == 0, "please make sure that LteRlHelper::EnableRlcTraces is called at most once");
  m_rlcStats = CreateObject<RadioBearerStatsCalculator> ("RLC");
  m_radioBearerStatsConnector.EnableRlcStats (m_rlcStats);
}

int64_t
LteRlHelper::AssignStreams (NetDeviceContainer c, int64_t stream)
{
  int64_t currentStream = stream;
  if ((m_fadingModule != 0) && (m_fadingStreamsAssigned == false))
    {
      Ptr<TraceFadingLossModel> tflm = m_fadingModule->GetObject<TraceFadingLossModel> ();
      if (tflm != 0)
        {
          currentStream += tflm->AssignStreams (currentStream);
          m_fadingStreamsAssigned = true;
        }
    }
  Ptr<NetDevice> netDevice;
  for (NetDeviceContainer::Iterator i = c.Begin (); i != c.End (); ++i)
    {
      netDevice = (*i);
      Ptr<LteRlEnbNetDevice> lteEnb = DynamicCast<LteRlEnbNetDevice> (netDevice);
      if (lteEnb)
        {
          Ptr<LteRlSpectrumPhy> dlPhy = lteEnb->GetPhy ()->GetDownlinkSpectrumPhy ();
          Ptr<LteRlSpectrumPhy> ulPhy = lteEnb->GetPhy ()->GetUplinkSpectrumPhy ();
          currentStream += dlPhy->AssignStreams (currentStream);
          currentStream += ulPhy->AssignStreams (currentStream);
        }
      Ptr<LteRlUeNetDevice> lteUe = DynamicCast<LteRlUeNetDevice> (netDevice);
      if (lteUe)
        {
          Ptr<LteRlSpectrumPhy> dlPhy = lteUe->GetPhy ()->GetDownlinkSpectrumPhy ();
          Ptr<LteRlSpectrumPhy> ulPhy = lteUe->GetPhy ()->GetUplinkSpectrumPhy ();
          Ptr<LteUeMac> ueMac = lteUe->GetMac ();
          currentStream += dlPhy->AssignStreams (currentStream);
          currentStream += ulPhy->AssignStreams (currentStream);
          currentStream += ueMac->AssignStreams (currentStream);
        }
    }
  return (currentStream - stream);
}


void
LteRlHelper::EnablePhyTraces (void)
{
  EnableDlPhyTraces ();
  EnableUlPhyTraces ();
  EnableDlTxPhyTraces ();
  EnableUlTxPhyTraces ();
  EnableDlRxPhyTraces ();
  EnableUlRxPhyTraces ();
}

void
LteRlHelper::EnableDlTxPhyTraces (void)
{
  Config::Connect ("/NodeList/*/DeviceList/*/LteEnbPhy/DlPhyTransmission",
                   MakeBoundCallback (&PhyTxStatsCalculator::DlPhyTransmissionCallback, m_phyTxStats));
}

void
LteRlHelper::EnableUlTxPhyTraces (void)
{
  Config::Connect ("/NodeList/*/DeviceList/*/LteUePhy/UlPhyTransmission",
                   MakeBoundCallback (&PhyTxStatsCalculator::UlPhyTransmissionCallback, m_phyTxStats));
}

void
LteRlHelper::EnableDlRxPhyTraces (void)
{
  Config::Connect ("/NodeList/*/DeviceList/*/LteUePhy/DlSpectrumPhy/DlPhyReception",
                   MakeBoundCallback (&PhyRxStatsCalculator::DlPhyReceptionCallback, m_phyRxStats));
}

void
LteRlHelper::EnableUlRxPhyTraces (void)
{
  Config::Connect ("/NodeList/*/DeviceList/*/LteEnbPhy/UlSpectrumPhy/UlPhyReception",
                   MakeBoundCallback (&PhyRxStatsCalculator::UlPhyReceptionCallback, m_phyRxStats));
}


void
LteRlHelper::EnableMacTraces (void)
{
  EnableDlMacTraces ();
  EnableUlMacTraces ();
}


void
LteRlHelper::EnableDlMacTraces (void)
{
  NS_LOG_FUNCTION_NOARGS ();
  Config::Connect ("/NodeList/*/DeviceList/*/LteEnbMac/DlScheduling",
                   MakeBoundCallback (&MacStatsCalculator::DlSchedulingCallback, m_macStats));
}

void
LteRlHelper::EnableUlMacTraces (void)
{
  NS_LOG_FUNCTION_NOARGS ();
  Config::Connect ("/NodeList/*/DeviceList/*/LteEnbMac/UlScheduling",
                   MakeBoundCallback (&MacStatsCalculator::UlSchedulingCallback, m_macStats));
}

void
LteRlHelper::EnableDlPhyTraces (void)
{
  NS_LOG_FUNCTION_NOARGS ();
  Config::Connect ("/NodeList/*/DeviceList/*/LteUePhy/ReportCurrentCellRsrpSinr",
                   MakeBoundCallback (&PhyStatsCalculator::ReportCurrentCellRsrpSinrCallback, m_phyStats));
}

void
LteRlHelper::EnableUlPhyTraces (void)
{
  NS_LOG_FUNCTION_NOARGS ();
  Config::Connect ("/NodeList/*/DeviceList/*/LteEnbPhy/ReportUeSinr",
                   MakeBoundCallback (&PhyStatsCalculator::ReportUeSinr, m_phyStats));
  Config::Connect ("/NodeList/*/DeviceList/*/LteEnbPhy/ReportInterference",
                   MakeBoundCallback (&PhyStatsCalculator::ReportInterference, m_phyStats));

}

Ptr<RadioBearerStatsCalculator>
LteRlHelper::GetRlcStats (void)
{
  return m_rlcStats;
}

void
LteRlHelper::EnablePdcpTraces (void)
{
  NS_ASSERT_MSG (m_pdcpStats == 0, "please make sure that LteRlHelper::EnablePdcpTraces is called at most once");
  m_pdcpStats = CreateObject<RadioBearerStatsCalculator> ("PDCP");
  m_radioBearerStatsConnector.EnablePdcpStats (m_pdcpStats);
}

Ptr<RadioBearerStatsCalculator>
LteRlHelper::GetPdcpStats (void)
{
  return m_pdcpStats;
}

} // namespace ns3
